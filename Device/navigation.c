#include "navigation.h"
#include "stm32f4xx_hal.h"
#include <string.h>

static volatile navigation_command_t g_navigation;
static uint8_t g_rx_frame[NAVIGATION_FRAME_SIZE];
static uint8_t g_rx_length;

void Navigation_Init(void)
{
    g_navigation = (navigation_command_t){0};
    g_rx_length = 0U;
}

void Navigation_Invalidate(void)
{
    uint8_t i;
    g_navigation.valid = 0U;
    for (i = 0U; i < 4U; i++)
    {
        g_navigation.motor_rpm[i] = 0;
    }
    g_rx_length = 0U;
}

void Navigation_Receive(const uint8_t *data, uint32_t length)
{
    uint32_t offset;
    uint8_t i;
    uint8_t checksum;

    /* 原始数据到达也要计数，便于区分“链路没数据”和“数据协议不对”。 */
    if (length > 0U)
    {
        g_navigation.rx_byte_count += length;
        g_navigation.last_rx_tick_ms = HAL_GetTick();
    }

    for (offset = 0U; offset < length; offset++)
    {
        g_rx_frame[g_rx_length++] = data[offset];
        if (g_rx_length < NAVIGATION_FRAME_SIZE)
        {
            continue;
        }

        checksum = 0U;
        for (i = 0U; i < NAVIGATION_FRAME_SIZE - 1U; i++)
        {
            checksum ^= g_rx_frame[i];
        }

        if ((g_rx_frame[0] == 0xAAU) && (g_rx_frame[1] == 0x56U))
        {
            if (checksum == g_rx_frame[NAVIGATION_FRAME_SIZE - 1U])
            {
                for (i = 0U; i < 4U; i++)
                {
                    g_navigation.motor_rpm[i] = (int16_t)(
                        (uint16_t)g_rx_frame[2U + 2U * i] |
                        ((uint16_t)g_rx_frame[3U + 2U * i] << 8));
                }
                /* 只有完整且校验正确的帧才能续期；后收到的目标覆盖旧目标。 */
                g_navigation.update_tick_ms = HAL_GetTick();
                g_navigation.frame_count++;
                g_navigation.valid = 1U;
                g_rx_length = 0U;
                continue;
            }
            g_navigation.checksum_error_count++;
        }

        /* 逐字节滑窗找帧头，支持拆包、粘包，以及坏帧/杂字节后的重新同步。 */
        memmove(g_rx_frame, g_rx_frame + 1U, NAVIGATION_FRAME_SIZE - 1U);
        g_rx_length = NAVIGATION_FRAME_SIZE - 1U;
    }
}

void Navigation_Get(navigation_command_t *command)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *command = g_navigation;
    __set_PRIMASK(primask);
}
