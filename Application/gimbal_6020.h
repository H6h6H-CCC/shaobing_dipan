#ifndef GIMBAL_6020_H
#define GIMBAL_6020_H

#include <stdint.h>
#include "dji_motor.h"

typedef struct
{
    uint8_t remote_online;
    uint8_t remote_valid;
    uint8_t remote_switch_left;
    uint8_t enabled;
    uint8_t stop_reason;       /* 0=正常，1=遥控器无效/开关非中，2=6020反馈离线 */
    uint8_t motor_online_mask;
    int8_t limit_state[DJI_GIMBAL_MOTOR_COUNT];
    uint8_t send_status;       /* HAL_OK/HAL_BUSY/HAL_ERROR */
    uint8_t can_mailboxes_free;
    int16_t remote_yaw;
    int16_t remote_pitch;
    uint16_t ecd[DJI_GIMBAL_MOTOR_COUNT];
    int16_t angle10[DJI_GIMBAL_MOTOR_COUNT];
    int16_t target10[DJI_GIMBAL_MOTOR_COUNT];
    int16_t error10[DJI_GIMBAL_MOTOR_COUNT];
    int16_t speed_rpm[DJI_GIMBAL_MOTOR_COUNT];
    int16_t target_rpm[DJI_GIMBAL_MOTOR_COUNT];
    int16_t command_current[DJI_GIMBAL_MOTOR_COUNT];
    int16_t feedback_current[DJI_GIMBAL_MOTOR_COUNT];
    uint8_t temperature[DJI_GIMBAL_MOTOR_COUNT];
    uint8_t reserved_byte[DJI_GIMBAL_MOTOR_COUNT];
    uint32_t feedback_age_ms[DJI_GIMBAL_MOTOR_COUNT];
    uint32_t can_tx_fail_count;
    uint32_t can_state;
    uint32_t can_error;
} gimbal6020_debug_t;

void Gimbal6020_Init(void);
void Gimbal6020_Update(void);
void Gimbal6020_Stop(void);
void Gimbal6020_GetDebug(gimbal6020_debug_t *debug);

#endif
