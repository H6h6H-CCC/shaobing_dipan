#ifndef GIMBAL_HEADING_LINK_H
#define GIMBAL_HEADING_LINK_H

#include <stdint.h>

#define GIMBAL_HEADING_TIMEOUT_MS 100U

typedef struct
{
    int16_t angle_cdeg;
    uint16_t big_ecd;
    uint16_t small_ecd;
    uint32_t update_tick_ms;
    uint32_t frame_count;
    uint8_t valid;
} gimbal_heading_command_t;

/* 修改：USART1接收云台相对朝向，100Hz、逆时针正、单位0.01度。
 * A7 5A | seq | valid | angle_cdeg(i16) | big_ecd(u16) |
 * small_ecd(u16) | CRC16/MODBUS(前10字节)，全部小端，共12字节。
 * 同seq只接收失效通知，不为旧朝向续期；超时由控制入口按快照判断。
 */
void Gimbal_Heading_Init(void);
void Gimbal_Heading_Get(gimbal_heading_command_t *command);
void Gimbal_Heading_UART_IRQHandler(void);
void Gimbal_Heading_Receive(const uint8_t *data, uint16_t length);

#endif
