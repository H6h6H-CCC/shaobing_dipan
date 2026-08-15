#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

#include "stm32f4xx_hal.h"

#define DJI_CHASSIS_MOTOR_COUNT 4U

typedef struct
{
    uint16_t ecd;
    uint16_t last_ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperature;
    uint8_t error_code;
    uint32_t update_tick_ms;
} dji_motor_feedback_t;

HAL_StatusTypeDef DJI_Motor_SendChassisCurrent(CAN_HandleTypeDef *hcan,
                                               int16_t motor1,
                                               int16_t motor2,
                                               int16_t motor3,
                                               int16_t motor4);
void DJI_Motor_GetFeedback(uint8_t index, dji_motor_feedback_t *feedback);
uint8_t DJI_Motor_IsOnline(uint8_t index, uint32_t now_ms, uint32_t timeout_ms);

#endif
