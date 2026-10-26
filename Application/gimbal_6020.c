#include "gimbal_6020.h"
#include "dji_motor.h"
#include "remote_control.h"
#include "can.h"
#include "pid.h"

#define GIMBAL6020_REMOTE_TIMEOUT_MS 100U
#define GIMBAL6020_MOTOR_TIMEOUT_MS  100U
#define GIMBAL6020_REQUIRED_MASK     0x05U /* ID1 + ID3 */
#define GIMBAL6020_ECD_PER_REV       8192.0f
#define GIMBAL6020_DEG_PER_ECD       (360.0f / GIMBAL6020_ECD_PER_REV)

/* ID1 为 Yaw，ID3 为 Pitch；ID2 完全不参与控制。 */
#define GIMBAL6020_PITCH_MIN_DEG     107.9f
#define GIMBAL6020_PITCH_MAX_DEG     135.4f
#define GIMBAL6020_SOFT_MARGIN_DEG   3.0f

/* 6020 编码器正方向与机械角度正方向一致；若实车相反，只改为 -1.0f。 */
#define GIMBAL6020_YAW_DIRECTION     1.0f
#define GIMBAL6020_PITCH_DIRECTION  -1.0f

#define GIMBAL6020_RC_RANGE          660.0f
#define GIMBAL6020_RC_DEADBAND       20
#define GIMBAL6020_CONTROL_PERIOD_S  0.002f
/* 摇杆满量程时的目标角速度；目标角度采用累加方式，松杆后保持当前位置。 */
#define GIMBAL6020_YAW_RATE_DEG_S    60.0f
#define GIMBAL6020_PITCH_RATE_DEG_S  15.0f

/* yuntai_test 的遥控串级 PID 参数。角度环输出目标转速，速度环输出电流。 */
#define YAW_ANGLE_KP                 1.20f
#define YAW_ANGLE_KI                 0.0f
#define YAW_ANGLE_KD                 0.0f
#define YAW_ANGLE_MAX_RPM            60.0f
#define YAW_SPEED_KP                 45.0f
#define YAW_SPEED_KI                 0.0f
#define YAW_SPEED_KD                 0.0f
#define YAW_CURRENT_MAX              6000.0f

#define PITCH_ANGLE_KP               1.20f
#define PITCH_ANGLE_KI               0.0f
#define PITCH_ANGLE_KD               0.0f
#define PITCH_ANGLE_MAX_RPM          120.0f
#define PITCH_SPEED_KP               105.0f
#define PITCH_SPEED_KI               0.15f
#define PITCH_SPEED_KD               0.8f
#define PITCH_CURRENT_MAX            16000.0f

/* ID2 的控制电流在更新函数中固定为零。 */

typedef enum
{
    GIMBAL6020_PID_POSITION = 0,
    GIMBAL6020_PID_DELTA
} gimbal6020_pid_mode_t;

typedef struct
{
    gimbal6020_pid_mode_t mode;
    float kp;
    float ki;
    float kd;
    float max_integral;
    float max_output;
    float integral;
    float error[3];
    float output;
} gimbal6020_pid_t;

typedef struct
{
    float min_deg;
    float max_deg;
    float soft_margin_deg;
    uint8_t limits_enabled;
    float direction;
    float current_max;
    float target_deg;
    gimbal6020_pid_t angle_pid;
    gimbal6020_pid_t speed_pid;
} gimbal6020_axis_t;

static gimbal6020_axis_t g_axis[DJI_GIMBAL_MOTOR_COUNT];
static uint8_t g_target_initialized[DJI_GIMBAL_MOTOR_COUNT];
static gimbal6020_debug_t g_debug;

static float Gimbal6020_Clamp(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static void Gimbal6020_PIDInit(gimbal6020_pid_t *pid,
                               gimbal6020_pid_mode_t mode,
                               float kp, float ki, float kd,
                               float max_integral, float max_output)
{
    pid->mode = mode;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->max_integral = max_integral;
    pid->max_output = max_output;
    pid->integral = 0.0f;
    pid->error[0] = 0.0f;
    pid->error[1] = 0.0f;
    pid->error[2] = 0.0f;
    pid->output = 0.0f;
}

static void Gimbal6020_PIDClear(gimbal6020_pid_t *pid)
{
    pid->integral = 0.0f;
    pid->error[0] = 0.0f;
    pid->error[1] = 0.0f;
    pid->error[2] = 0.0f;
    pid->output = 0.0f;
}

static float Gimbal6020_PIDUpdate(gimbal6020_pid_t *pid,
                                  float feedback, float target)
{
    float error;
    float output;

    error = target - feedback;
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->error[0] = error;

    if (pid->mode == GIMBAL6020_PID_DELTA)
    {
        pid->integral = pid->ki * error;
        output = pid->output +
                 pid->kp * (pid->error[0] - pid->error[1]) +
                 pid->integral +
                 pid->kd * (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
    }
    else
    {
        pid->integral += pid->ki * error;
        pid->integral = Gimbal6020_Clamp(pid->integral,
                                         -pid->max_integral, pid->max_integral);
        output = pid->kp * error + pid->integral +
                 pid->kd * (pid->error[0] - pid->error[1]);
    }

    output = Gimbal6020_Clamp(output, -pid->max_output, pid->max_output);
    pid->output = output;
    return output;
}

static float Gimbal6020_EcdToDeg(uint16_t ecd)
{
    return (float)ecd * GIMBAL6020_DEG_PER_ECD;
}

static float Gimbal6020_Wrap360(float angle_deg)
{
    while (angle_deg >= 360.0f)
    {
        angle_deg -= 360.0f;
    }
    while (angle_deg < 0.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static float Gimbal6020_ShortestError(float target_deg, float feedback_deg)
{
    float error = target_deg - feedback_deg;

    while (error > 180.0f)
    {
        error -= 360.0f;
    }
    while (error < -180.0f)
    {
        error += 360.0f;
    }
    return error;
}

static int8_t Gimbal6020_GetLimitState(float angle_deg,
                                       const gimbal6020_axis_t *axis)
{
    if (axis->limits_enabled == 0U)
    {
        return 0;
    }

    if (angle_deg <= axis->min_deg)
    {
        return -2;
    }
    if (angle_deg < (axis->min_deg + axis->soft_margin_deg))
    {
        return -1;
    }
    if (angle_deg >= axis->max_deg)
    {
        return 2;
    }
    if (angle_deg > (axis->max_deg - axis->soft_margin_deg))
    {
        return 1;
    }
    return 0;
}

static void Gimbal6020_SendCurrent(int16_t motor1,
                                   int16_t motor2,
                                   int16_t motor3)
{
    HAL_StatusTypeDef status =
        DJI_6020_SendCurrent(&hcan1, motor1, motor2, motor3, 0);

    g_debug.send_status = (uint8_t)status;
    g_debug.can_mailboxes_free =
        (uint8_t)HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
    g_debug.can_state = (uint32_t)HAL_CAN_GetState(&hcan1);
    g_debug.can_error = (uint32_t)HAL_CAN_GetError(&hcan1);
    if (status != HAL_OK)
    {
        g_debug.can_tx_fail_count++;
    }
}

static float Gimbal6020_ApplySoftLimit(float current,
                                       float angle_deg,
                                       const gimbal6020_axis_t *axis)
{
    float ratio;

    if (axis->limits_enabled == 0U)
    {
        return current;
    }

    if ((angle_deg <= axis->min_deg) && (current < 0.0f))
    {
        return 0.0f;
    }
    if ((angle_deg >= axis->max_deg) && (current > 0.0f))
    {
        return 0.0f;
    }

    if ((current < 0.0f) &&
        (angle_deg < (axis->min_deg + axis->soft_margin_deg)))
    {
        ratio = (angle_deg - axis->min_deg) / axis->soft_margin_deg;
        return current * Gimbal6020_Clamp(ratio, 0.0f, 1.0f) *
               Gimbal6020_Clamp(ratio, 0.0f, 1.0f);
    }
    if ((current > 0.0f) &&
        (angle_deg > (axis->max_deg - axis->soft_margin_deg)))
    {
        ratio = (axis->max_deg - angle_deg) / axis->soft_margin_deg;
        return current * Gimbal6020_Clamp(ratio, 0.0f, 1.0f) *
               Gimbal6020_Clamp(ratio, 0.0f, 1.0f);
    }
    return current;
}

static int16_t Gimbal6020_ControlAxis(uint8_t index,
                                      float angle_deg,
                                      int16_t speed_rpm)
{
    gimbal6020_axis_t *axis = &g_axis[index];
    float angle_error = Gimbal6020_ShortestError(axis->target_deg, angle_deg);
    float target_axis_rpm;
    float target_rpm;
    float limit_current;
    float current;

    target_axis_rpm = Gimbal6020_PIDUpdate(&axis->angle_pid,
                                            0.0f, angle_error);
    target_axis_rpm = Gimbal6020_Clamp(target_axis_rpm,
                                       -axis->angle_pid.max_output,
                                       axis->angle_pid.max_output);
    /* Angle and speed feedback share the encoder-positive coordinate. */
    target_rpm = target_axis_rpm;
    limit_current = Gimbal6020_PIDUpdate(&axis->speed_pid,
                                         (float)speed_rpm,
                                         target_rpm);
    limit_current = Gimbal6020_Clamp(limit_current,
                                     -axis->current_max,
                                     axis->current_max);
    limit_current = Gimbal6020_ApplySoftLimit(limit_current, angle_deg, axis);
    /* Convert the logical axis current to the installed motor direction. */
    current = limit_current * axis->direction;

    g_debug.error10[index] = (int16_t)(angle_error * 10.0f);
    g_debug.target_rpm[index] = (int16_t)target_rpm;
    g_debug.command_current[index] = (int16_t)current;

    /* Yaw skips this reset because its mechanical limit is disabled. */
    if ((axis->limits_enabled != 0U) &&
        (current == 0.0f) &&
        ((angle_deg <= axis->min_deg) || (angle_deg >= axis->max_deg)))
    {
        Gimbal6020_PIDClear(&axis->angle_pid);
        Gimbal6020_PIDClear(&axis->speed_pid);
    }
    return (int16_t)current;
}

void Gimbal6020_Init(void)
{
    /* ID1 Yaw: encoder position control over the full 0-360 degree range. */
    g_axis[0].min_deg = 0.0f;
    g_axis[0].max_deg = 360.0f;
    g_axis[0].soft_margin_deg = GIMBAL6020_SOFT_MARGIN_DEG;
    g_axis[0].limits_enabled = 0U;
    g_axis[0].direction = GIMBAL6020_YAW_DIRECTION;
    g_axis[0].current_max = YAW_CURRENT_MAX;
    Gimbal6020_PIDInit(&g_axis[0].angle_pid, GIMBAL6020_PID_POSITION,
                       YAW_ANGLE_KP, YAW_ANGLE_KI, YAW_ANGLE_KD,
                       0.0f, YAW_ANGLE_MAX_RPM);
    /* Non-accumulating speed P loop prevents current from flipping rails. */
    Gimbal6020_PIDInit(&g_axis[0].speed_pid, GIMBAL6020_PID_POSITION,
                       YAW_SPEED_KP, YAW_SPEED_KI, YAW_SPEED_KD,
                       0.0f, YAW_CURRENT_MAX);

    /* ID2 不控制：保留反馈读取，但不产生控制电流。 */
    g_axis[1].min_deg = 0.0f;
    g_axis[1].max_deg = 360.0f;
    g_axis[1].soft_margin_deg = GIMBAL6020_SOFT_MARGIN_DEG;
    g_axis[1].limits_enabled = 0U;
    g_axis[1].direction = 1.0f;
    g_axis[1].current_max = 0.0f;
    Gimbal6020_PIDInit(&g_axis[1].angle_pid, GIMBAL6020_PID_POSITION,
                       0.0f, 0.0f, 0.0f,
                       0.0f, 0.0f);
    Gimbal6020_PIDInit(&g_axis[1].speed_pid, GIMBAL6020_PID_POSITION,
                       0.0f, 0.0f, 0.0f,
                       0.0f, 0.0f);

    /* ID3 Pitch keeps the B-plan mechanical range. */
    g_axis[2].min_deg = GIMBAL6020_PITCH_MIN_DEG;
    g_axis[2].max_deg = GIMBAL6020_PITCH_MAX_DEG;
    g_axis[2].soft_margin_deg = GIMBAL6020_SOFT_MARGIN_DEG;
    g_axis[2].limits_enabled = 1U;
    g_axis[2].direction = GIMBAL6020_PITCH_DIRECTION;
    g_axis[2].current_max = PITCH_CURRENT_MAX;
    Gimbal6020_PIDInit(&g_axis[2].angle_pid, GIMBAL6020_PID_POSITION,
                       PITCH_ANGLE_KP, PITCH_ANGLE_KI, PITCH_ANGLE_KD,
                       0.0f, PITCH_ANGLE_MAX_RPM);
    /* Pitch keeps the original incremental speed PID control. */
    Gimbal6020_PIDInit(&g_axis[2].speed_pid, GIMBAL6020_PID_DELTA,
                       PITCH_SPEED_KP, PITCH_SPEED_KI, PITCH_SPEED_KD,
                       0.0f, PITCH_CURRENT_MAX);

    Gimbal6020_Stop();
}

void Gimbal6020_Stop(void)
{
    Gimbal6020_PIDClear(&g_axis[0].angle_pid);
    Gimbal6020_PIDClear(&g_axis[0].speed_pid);
    Gimbal6020_PIDClear(&g_axis[1].angle_pid);
    Gimbal6020_PIDClear(&g_axis[1].speed_pid);
    Gimbal6020_PIDClear(&g_axis[2].angle_pid);
    Gimbal6020_PIDClear(&g_axis[2].speed_pid);
    g_target_initialized[0] = 0U;
    g_target_initialized[1] = 0U;
    g_target_initialized[2] = 0U;
    g_debug.enabled = 0U;
    g_debug.error10[0] = 0;
    g_debug.error10[1] = 0;
    g_debug.error10[2] = 0;
    g_debug.target_rpm[0] = 0;
    g_debug.target_rpm[1] = 0;
    g_debug.target_rpm[2] = 0;
    g_debug.command_current[0] = 0;
    g_debug.command_current[1] = 0;
    g_debug.command_current[2] = 0;
    Gimbal6020_SendCurrent(0, 0, 0);
}

void Gimbal6020_Update(void)
{
    remote_control_t remote = {0};
    dji_6020_feedback_t motor[DJI_GIMBAL_MOTOR_COUNT] = {0};
    uint32_t now_ms = HAL_GetTick();
    float angle_deg[DJI_GIMBAL_MOTOR_COUNT];
    float rc_yaw;
    float rc_pitch;
    int16_t current[DJI_GIMBAL_MOTOR_COUNT];
    uint8_t i;

    Remote_Control_Get(&remote);
    g_debug.remote_online =
        Remote_Control_IsOnline(now_ms, GIMBAL6020_REMOTE_TIMEOUT_MS);
    g_debug.remote_valid = Remote_Control_DataValid(&remote);
    g_debug.remote_switch_left = remote.switch_position[RC_S_LEFT];
    g_debug.remote_yaw = remote.channel[0];
    g_debug.remote_pitch = remote.channel[1];
    g_debug.motor_online_mask = 0U;
    for (i = 0U; i < DJI_GIMBAL_MOTOR_COUNT; i++)
    {
        DJI_6020_GetFeedback(i, &motor[i]);
        angle_deg[i] = Gimbal6020_EcdToDeg(motor[i].ecd);
        g_debug.ecd[i] = motor[i].ecd;
        g_debug.angle10[i] = (int16_t)(angle_deg[i] * 10.0f);
        g_debug.speed_rpm[i] = motor[i].speed_rpm;
        g_debug.feedback_current[i] = motor[i].given_current;
        g_debug.temperature[i] = motor[i].temperature;
        g_debug.reserved_byte[i] = motor[i].reserved_byte;
        g_debug.feedback_age_ms[i] =
            (motor[i].update_tick_ms == 0U) ? 0xFFFFFFFFU :
            (uint32_t)(now_ms - motor[i].update_tick_ms);
        if ((motor[i].update_tick_ms != 0U) &&
            (g_debug.feedback_age_ms[i] <= GIMBAL6020_MOTOR_TIMEOUT_MS))
        {
            g_debug.motor_online_mask |= (uint8_t)(1U << i);
        }
        g_debug.limit_state[i] =
            Gimbal6020_GetLimitState(angle_deg[i], &g_axis[i]);
    }

    if ((g_debug.remote_online == 0U) ||
        (g_debug.remote_valid == 0U) ||
        (remote.switch_position[RC_S_LEFT] != RC_SW_MID))
    {
        g_debug.stop_reason = 1U;
        Gimbal6020_Stop();
        return;
    }
    if ((g_debug.motor_online_mask & GIMBAL6020_REQUIRED_MASK) !=
        GIMBAL6020_REQUIRED_MASK)
    {
        g_debug.stop_reason = 2U;
        Gimbal6020_Stop();
        return;
    }
    g_debug.stop_reason = 0U;
    g_debug.enabled = 1U;
    g_debug.can_state = (uint32_t)HAL_CAN_GetState(&hcan1);
    g_debug.can_error = (uint32_t)HAL_CAN_GetError(&hcan1);

    if (g_target_initialized[0] == 0U)
    {
        g_axis[0].target_deg = angle_deg[0];
        g_target_initialized[0] = 1U;
    }
    if (g_target_initialized[2] == 0U)
    {
        g_axis[2].target_deg = angle_deg[2];
        g_target_initialized[2] = 1U;
    }

    rc_yaw = ((float)remote.channel[0] / GIMBAL6020_RC_RANGE);
    rc_pitch = ((float)remote.channel[1] / GIMBAL6020_RC_RANGE);
    if ((remote.channel[0] > -GIMBAL6020_RC_DEADBAND) &&
        (remote.channel[0] < GIMBAL6020_RC_DEADBAND))
    {
        rc_yaw = 0.0f;
    }
    if ((remote.channel[1] > -GIMBAL6020_RC_DEADBAND) &&
        (remote.channel[1] < GIMBAL6020_RC_DEADBAND))
    {
        rc_pitch = 0.0f;
    }

    /* 保持原有方向约定：摇杆正值使目标角度减小。 */
    g_axis[0].target_deg -= rc_yaw * GIMBAL6020_YAW_RATE_DEG_S *
                             GIMBAL6020_CONTROL_PERIOD_S;
    g_axis[2].target_deg -= rc_pitch * GIMBAL6020_PITCH_RATE_DEG_S *
                             GIMBAL6020_CONTROL_PERIOD_S;
    g_axis[0].target_deg = Gimbal6020_Wrap360(g_axis[0].target_deg);
    g_axis[2].target_deg = Gimbal6020_Clamp(g_axis[2].target_deg,
                                            g_axis[2].min_deg,
                                            g_axis[2].max_deg);
    for (i = 0U; i < DJI_GIMBAL_MOTOR_COUNT; i++)
    {
        g_debug.target10[i] = (int16_t)(g_axis[i].target_deg * 10.0f);
    }

    current[0] = Gimbal6020_ControlAxis(0U, angle_deg[0], motor[0].speed_rpm);
    current[1] = 0;
    g_debug.error10[1] = 0;
    g_debug.target_rpm[1] = 0;
    g_debug.command_current[1] = 0;
    current[2] = Gimbal6020_ControlAxis(2U, angle_deg[2], motor[2].speed_rpm);
    Gimbal6020_SendCurrent(current[0], current[1], current[2]);
}

void Gimbal6020_GetDebug(gimbal6020_debug_t *debug)
{
    if (debug != 0)
    {
        *debug = g_debug;
    }
}
