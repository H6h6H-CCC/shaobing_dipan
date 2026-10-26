#include "remote_control.h"
#include "bsp_remote.h"
#include <string.h>

#define RC_FRAME_LENGTH 18U
#define RC_DMA_BUFFER_LENGTH 36U
#define RC_CHANNEL_OFFSET 1024
#define RC_CHANNEL_MAX_ABS 700
#define RC_FORWARD_HEADER 0xA6U
#define RC_FORWARD_FRAME_LENGTH 20U

extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart3_rx;

static uint8_t g_rx_buffer[2][RC_DMA_BUFFER_LENGTH];
static volatile remote_control_t g_remote;
/* 转发缓冲与DMA发送缓冲分开，避免新遥控帧覆盖正在发送的数据。 */
static uint8_t g_forward_pending_frame[RC_FORWARD_FRAME_LENGTH];
static uint8_t g_forward_dma_frame[RC_FORWARD_FRAME_LENGTH];
static volatile uint8_t g_forward_pending;

static void Remote_Control_Decode(const uint8_t data[RC_FRAME_LENGTH])
{
    remote_control_t decoded = {0};

    decoded.channel[0] = (int16_t)(((data[0] | (data[1] << 8)) & 0x07FF) - RC_CHANNEL_OFFSET);
    decoded.channel[1] = (int16_t)((((data[1] >> 3) | (data[2] << 5)) & 0x07FF) - RC_CHANNEL_OFFSET);
    decoded.channel[2] = (int16_t)((((data[2] >> 6) | (data[3] << 2) |
                                     (data[4] << 10)) & 0x07FF) - RC_CHANNEL_OFFSET);
    decoded.channel[3] = (int16_t)((((data[4] >> 1) | (data[5] << 7)) & 0x07FF) - RC_CHANNEL_OFFSET);
    decoded.switch_position[0] = (uint8_t)((data[5] >> 4) & 0x03);
    decoded.switch_position[1] = (uint8_t)(((data[5] >> 4) & 0x0C) >> 2);
    decoded.mouse_x = (int16_t)(data[6] | (data[7] << 8));
    decoded.mouse_y = (int16_t)(data[8] | (data[9] << 8));
    decoded.mouse_z = (int16_t)(data[10] | (data[11] << 8));
    decoded.mouse_left = data[12];
    decoded.mouse_right = data[13];
    decoded.keyboard = (uint16_t)(data[14] | (data[15] << 8));
    decoded.channel[4] = (int16_t)((data[16] | (data[17] << 8)) - RC_CHANNEL_OFFSET);
    decoded.update_tick_ms = HAL_GetTick();

    g_remote = decoded;

    /* 旧工程协议：0xA6 + 原始DBUS 18字节 + 8位累加和。 */
    g_forward_pending_frame[0] = RC_FORWARD_HEADER;
    memcpy(&g_forward_pending_frame[1], data, RC_FRAME_LENGTH);
    g_forward_pending_frame[RC_FORWARD_FRAME_LENGTH - 1U] = 0U;
    for (uint8_t i = 0U; i < (RC_FORWARD_FRAME_LENGTH - 1U); i++)
    {
        g_forward_pending_frame[RC_FORWARD_FRAME_LENGTH - 1U] =
            (uint8_t)(g_forward_pending_frame[RC_FORWARD_FRAME_LENGTH - 1U] +
                      g_forward_pending_frame[i]);
    }
    g_forward_pending = 1U;
}

void Remote_Control_Init(void)
{
    BSP_Remote_Init(&huart3, &hdma_usart3_rx,
                    g_rx_buffer[0], g_rx_buffer[1], RC_DMA_BUFFER_LENGTH);
}

void Remote_Control_UART_IRQHandler(void)
{
    uint16_t received_length;
    uint8_t *completed_buffer;

    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_ORE) != RESET)
    {
        __HAL_UART_CLEAR_OREFLAG(&huart3);
    }

    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE) == RESET)
    {
        return;
    }

    __HAL_UART_CLEAR_IDLEFLAG(&huart3);
    __HAL_DMA_DISABLE(&hdma_usart3_rx);
    received_length = (uint16_t)(RC_DMA_BUFFER_LENGTH - hdma_usart3_rx.Instance->NDTR);
    hdma_usart3_rx.Instance->NDTR = RC_DMA_BUFFER_LENGTH;

    if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == 0U)
    {
        completed_buffer = g_rx_buffer[0];
        SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_CT);
    }
    else
    {
        completed_buffer = g_rx_buffer[1];
        CLEAR_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_CT);
    }

    __HAL_DMA_ENABLE(&hdma_usart3_rx);

    if (received_length == RC_FRAME_LENGTH)
    {
        Remote_Control_Decode(completed_buffer);
    }
}

void Remote_Control_Forward_Update(void)
{
    uint32_t primask;

    if (g_forward_pending == 0U)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    memcpy(g_forward_dma_frame, g_forward_pending_frame,
           RC_FORWARD_FRAME_LENGTH);
    g_forward_pending = 0U;
    if (primask == 0U)
    {
        __enable_irq();
    }

    /* DMA繁忙时保留pending帧，下一次主循环重试。 */
    if (HAL_UART_Transmit_DMA(&huart1, g_forward_dma_frame,
                              RC_FORWARD_FRAME_LENGTH) != HAL_OK)
    {
        g_forward_pending = 1U;
    }
}

void Remote_Control_Get(remote_control_t *remote)
{
    uint32_t primask;

    if (remote == NULL)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *remote = g_remote;
    if (primask == 0U)
    {
        __enable_irq();
    }
}

uint8_t Remote_Control_IsOnline(uint32_t now_ms, uint32_t timeout_ms)
{
    remote_control_t remote = {0};
    Remote_Control_Get(&remote);
    return (uint8_t)((remote.update_tick_ms != 0U) &&
                     ((uint32_t)(now_ms - remote.update_tick_ms) <= timeout_ms));
}

uint8_t Remote_Control_DataValid(const remote_control_t *remote)
{
    uint8_t i;

    if (remote == NULL)
    {
        return 0U;
    }

    for (i = 0; i < 5; i++)
    {
        if ((remote->channel[i] > RC_CHANNEL_MAX_ABS) ||
            (remote->channel[i] < -RC_CHANNEL_MAX_ABS))
        {
            return 0U;
        }
    }

    if ((remote->switch_position[0] == 0U) ||
        (remote->switch_position[1] == 0U))
    {
        return 0U;
    }
    return 1U;
}
