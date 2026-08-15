#ifndef CHASSIS_H
#define CHASSIS_H

#include <stdint.h>

#define CHASSIS_CONTROL_PERIOD_MS 2U

/*
 * 底盘方向角定义（俯视机器人）：
 * 正前方为 0°，顺时针增加，统一归一化到 [0°, 360°)。
 * 0°/360°：前，90°：右，180°：后，270°：左。
 */
#define CHASSIS_DIRECTION_FRONT_DEG 0.0f
#define CHASSIS_DIRECTION_RIGHT_DEG 90.0f
#define CHASSIS_DIRECTION_REAR_DEG  180.0f
#define CHASSIS_DIRECTION_LEFT_DEG  270.0f

typedef struct
{
    float vx_command;             /* m/s */
    float vy_command;             /* m/s */
    float wz_command;             /* rad/s */
    float motor_target_rpm[4];
    int16_t motor_feedback_rpm[4];
    int16_t motor_feedback_current[4];
    int16_t motor_current[4];
    uint32_t motor_feedback_age_ms[4];
    uint32_t can_tx_fail_count;
    uint8_t motor_temperature[4];
    uint8_t motor_error_code[4];
    uint8_t motor_stall_mask;
    uint8_t motor_no_output_mask;
    uint8_t can_tx_mailboxes_free;
    uint8_t enabled;
    uint8_t remote_online;
    uint8_t motor_online_mask;
} chassis_debug_t;

extern volatile chassis_debug_t g_chassis_debug;

void Chassis_Init(void);
void Chassis_Update(void);
void Chassis_Stop(void);

#endif
