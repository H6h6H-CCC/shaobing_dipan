#include "gimbal_heading_link.h"

#include <string.h>

#include "bsp_remote.h"
#include "usart.h"

#define GIMBAL_HEADING_FRAME_SIZE    12U
#define GIMBAL_HEADING_DMA_SIZE      64U

extern DMA_HandleTypeDef hdma_usart1_rx;

static uint8_t g_rx_buffer[2][GIMBAL_HEADING_DMA_SIZE];
static uint8_t g_frame[GIMBAL_HEADING_FRAME_SIZE];
static uint8_t g_frame_length;
static uint8_t g_last_sequence;
static uint8_t g_have_sequence;
static volatile gimbal_heading_command_t g_heading;

static uint16_t Gimbal_Heading_ReadU16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static uint16_t Gimbal_Heading_Crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;
    uint8_t bit;

    for (i = 0U; i < length; i++)
    {
        crc ^= data[i];
        for (bit = 0U; bit < 8U; bit++)
        {
            crc = (crc & 1U) ? (uint16_t)((crc >> 1) ^ 0xA001U) :
                               (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

void Gimbal_Heading_Init(void)
{
    g_heading = (gimbal_heading_command_t){0};
    g_frame_length = 0U;
    g_have_sequence = 0U;
    g_last_sequence = 0U;
    /* 修改：原USART1仅发底盘A6/A5，现在同时启动独立RX DMA接云台方向。 */
    BSP_Remote_Init(&huart1, &hdma_usart1_rx,
                    g_rx_buffer[0], g_rx_buffer[1], GIMBAL_HEADING_DMA_SIZE);
}

void Gimbal_Heading_Receive(const uint8_t *data, uint16_t length)
{
    uint16_t i;

    if (data == NULL)
    {
        return;
    }
    for (i = 0U; i < length; i++)
    {
        g_frame[g_frame_length++] = data[i];
        if (g_frame_length < GIMBAL_HEADING_FRAME_SIZE)
        {
            continue;
        }
        if ((g_frame[0] == 0xA7U) && (g_frame[1] == 0x5AU) &&
            ((g_frame[3] == 0U) || (g_frame[3] == 1U)) &&
            (Gimbal_Heading_Crc16(g_frame, 10U) ==
             Gimbal_Heading_ReadU16(&g_frame[10])))
        {
            /* 修改：失效优先；重复序号不能让冻结的相对角一直有效。 */
            if (g_frame[3] == 0U)
            {
                g_heading.valid = 0U;
            }
            if ((g_have_sequence == 0U) || (g_frame[2] != g_last_sequence))
            {
                g_heading.angle_cdeg = (int16_t)Gimbal_Heading_ReadU16(&g_frame[4]);
                g_heading.big_ecd = Gimbal_Heading_ReadU16(&g_frame[6]);
                g_heading.small_ecd = Gimbal_Heading_ReadU16(&g_frame[8]);
                g_heading.valid = g_frame[3];
                g_heading.update_tick_ms = HAL_GetTick();
                g_heading.frame_count++;
                g_last_sequence = g_frame[2];
                g_have_sequence = 1U;
            }
            g_frame_length = 0U;
            continue;
        }
        /* 修改：逐字节滑窗支持半帧、粘包及坏帧后的重新同步。 */
        memmove(g_frame, &g_frame[1], GIMBAL_HEADING_FRAME_SIZE - 1U);
        g_frame_length = GIMBAL_HEADING_FRAME_SIZE - 1U;
    }
}

void Gimbal_Heading_UART_IRQHandler(void)
{
    uint16_t received_length;
    uint8_t *completed_buffer;

    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE) != RESET)
    {
        __HAL_UART_CLEAR_OREFLAG(&huart1);
    }
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) == RESET)
    {
        return;
    }
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    __HAL_DMA_DISABLE(&hdma_usart1_rx);
    received_length = (uint16_t)(GIMBAL_HEADING_DMA_SIZE - hdma_usart1_rx.Instance->NDTR);
    hdma_usart1_rx.Instance->NDTR = GIMBAL_HEADING_DMA_SIZE;
    if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == 0U)
    {
        completed_buffer = g_rx_buffer[0];
        SET_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_CT);
    }
    else
    {
        completed_buffer = g_rx_buffer[1];
        CLEAR_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_CT);
    }
    __HAL_DMA_ENABLE(&hdma_usart1_rx);
    Gimbal_Heading_Receive(completed_buffer, received_length);
}

void Gimbal_Heading_Get(gimbal_heading_command_t *command)
{
    uint32_t primask;

    if (command == NULL)
    {
        return;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    *command = g_heading;
    if (primask == 0U)
    {
        __enable_irq();
    }
}
