#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <stdint.h>

#define NAVIGATION_TIMEOUT_MS 300U
#define NAVIGATION_FRAME_SIZE 11U

/*
 * USB CDC 导航下行，固定 11 字节：
 * AA 56 | ID1_L ID1_H | ID2_L ID2_H | ID3_L ID3_H | ID4_L ID4_H | XOR
 * 四路均为 int16 小端电机转子 rpm；XOR 为前 10 字节逐字节异或。
 * 上位机按 50 Hz 持续发送，停车也发全零帧。底盘不再做方向或减速比换算。
 */
typedef struct
{
    int16_t motor_rpm[4];
    uint32_t rx_byte_count;       /* USB 收到的原始字节数，含半帧、坏帧和杂字节。 */
    uint32_t last_rx_tick_ms;     /* 最近收到原始字节的时刻，与有效帧时刻分开。 */
    uint32_t update_tick_ms;
    uint32_t frame_count;
    uint32_t checksum_error_count;
    uint8_t valid;
} navigation_command_t;

/* 必须在 MX_USB_DEVICE_Init 之前调用，避免清掉刚收到的 USB 帧。 */
void Navigation_Init(void);
/* 仅解析字节流并发布最新指令，不在 USB 中断中运行 PID 或发送 CAN。 */
void Navigation_Receive(const uint8_t *data, uint32_t length);
void Navigation_Get(navigation_command_t *command);
/* USB 取消配置时丢弃目标和半帧；累计帧数保留，供主循环判断新帧。 */
void Navigation_Invalidate(void);

#endif
