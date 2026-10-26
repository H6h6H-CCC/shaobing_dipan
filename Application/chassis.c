#include "chassis.h"
#include "dji_motor.h"
#include "remote_control.h"
#include "navigation.h"
#include "pid.h"
#include "can.h"

#define CHASSIS_REMOTE_TIMEOUT_MS 100U
#define CHASSIS_MOTOR_TIMEOUT_MS  100U
#define CHASSIS_RC_DEADBAND       10
#define CHASSIS_RC_RANGE          660.0f

/* 实车几何参数：四个 45 度全向轮。 */
#define CHASSIS_TRACK_WIDTH_M       0.320f
#define CHASSIS_WHEELBASE_M         0.370f
#define CHASSIS_WHEEL_DIAMETER_M    0.128f
#define CHASSIS_MOTOR_GEAR_RATIO    19.2f

/* 旋转项投影到 45 度轮轴方向后的等效力臂。 */
#define CHASSIS_ROTATION_ARM_M \
    (((CHASSIS_TRACK_WIDTH_M + CHASSIS_WHEELBASE_M) * 0.5f) * SQRT_HALF)

/* 轮缘线速度 m/s 转换为 M3508 电机轴转速 rpm。 */
#define CHASSIS_MPS_TO_MOTOR_RPM \
    (60.0f * CHASSIS_MOTOR_GEAR_RATIO / (PI_F * CHASSIS_WHEEL_DIAMETER_M))

/* 第一轮架空调试限速，确认方向和 PID 后再逐步提高。 */
#define CHASSIS_FORWARD_MAX_MPS     2.0f
#define CHASSIS_STRAFE_MAX_MPS      1.5f
#define CHASSIS_ROTATION_MAX_RAD_S  4.0f
/* 与原 Chassis 工程一致：单轮线速度限幅 4 m/s，按本车轮径和减速比换算为转子转速。 */
#define CHASSIS_WHEEL_MAX_MPS       4.0f
#define CHASSIS_WHEEL_MAX_RPM       (CHASSIS_WHEEL_MAX_MPS * CHASSIS_MPS_TO_MOTOR_RPM)
#define CHASSIS_CURRENT_MAX         14000.0f

#define CHASSIS_SPEED_KP 5.0f
#define CHASSIS_SPEED_KI 0.05f
#define CHASSIS_SPEED_KD 0.0f

/* 软件堵转判断：目标较高、实际速度很低且实际转矩电流较大，持续 500 ms。 */
#define CHASSIS_STALL_TARGET_RPM       500.0f
#define CHASSIS_STALL_SPEED_RPM        100
#define CHASSIS_STALL_CURRENT          3000
#define CHASSIS_STALL_TIME_MS          500U

/* 主控命令很大但电调实际电流接近 0，用于定位电调未出力。 */
#define CHASSIS_NO_OUTPUT_COMMAND       4000
#define CHASSIS_NO_OUTPUT_CURRENT        500
#define CHASSIS_NO_OUTPUT_TIME_MS        500U

/* 轮向测试：4个轮子按新车头方向低速滚动，电机转速符号由安装方向决定。 */
#define CHASSIS_DIRECTION_TEST       0U
#define CHASSIS_DIRECTION_TEST_RPM 300.0f

/*
 * 一次性对角方向测试：左前315°、右前45°、右后135°、左后225°。
 * 每个方向运行2 s后停车1 s，第一轮后再停车5 s，再运行一轮，之后永久停车。
 */
/* 暂时关闭对角方向测试；改为 1U 可重新启用。 */
#define CHASSIS_DIAGONAL_TEST            0U
#define CHASSIS_DIAGONAL_TEST_SPEED_MPS  0.5f
#define CHASSIS_DIAGONAL_MOVE_TIME_MS    2000U
#define CHASSIS_DIAGONAL_STOP_TIME_MS    1000U
#define CHASSIS_DIAGONAL_PAUSE_TIME_MS   5000U
#define CHASSIS_DIAGONAL_STEP_TIME_MS \
    (CHASSIS_DIAGONAL_MOVE_TIME_MS + CHASSIS_DIAGONAL_STOP_TIME_MS)
#define CHASSIS_DIAGONAL_ONE_ROUND_MS    (4U * CHASSIS_DIAGONAL_STEP_TIME_MS)
#define CHASSIS_DIAGONAL_SECOND_ROUND_MS \
    (CHASSIS_DIAGONAL_ONE_ROUND_MS + CHASSIS_DIAGONAL_PAUSE_TIME_MS)
#define CHASSIS_DIAGONAL_TOTAL_TIME_MS \
    (2U * CHASSIS_DIAGONAL_ONE_ROUND_MS + CHASSIS_DIAGONAL_PAUSE_TIME_MS)

#define PI_F      3.14159265f
#define SQRT_HALF 0.70710678f

volatile chassis_debug_t g_chassis_debug;
static pid_t g_speed_pid[4];
static uint16_t g_stall_time_ms[4];
static uint16_t g_no_output_time_ms[4];
static uint32_t g_diagonal_test_elapsed_ms;
static uint32_t g_navigation_entry_frame_count;

/* 新车头相对原定义转180度；数组下标仍按CAN ID1～ID4排列。 */
enum
{
    MOTOR_ID1_LEFT_REAR = 0,
    MOTOR_ID2_LEFT_FRONT,
    MOTOR_ID3_RIGHT_FRONT,
    MOTOR_ID4_RIGHT_REAR
};

/*
 * 逻辑正方向统一表示轮子朝新车头滚动，因此四轮方向相对旧定义全部取反。
 * ID1、ID2 保持电机编码器原方向，ID3、ID4 安装方向取反。
 */
static const float g_motor_direction[4] = {1.0f, 1.0f, -1.0f, -1.0f};

static float Chassis_AbsFloat(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static int32_t Chassis_AbsInt16(int16_t value)
{
    return (value >= 0) ? value : -(int32_t)value;
}

static float Chassis_DeadbandNormalize(int16_t input)
{
    if ((input <= CHASSIS_RC_DEADBAND) && (input >= -CHASSIS_RC_DEADBAND))
    {
        return 0.0f;
    }
    return (float)input / CHASSIS_RC_RANGE;
}

static uint8_t Chassis_GetDiagonalTestCommand(float *vx, float *vy, float *wz)
{
    uint32_t round_elapsed_ms;
    uint32_t step_elapsed_ms;
    uint8_t segment;
    const float component_mps = CHASSIS_DIAGONAL_TEST_SPEED_MPS * SQRT_HALF;

    *vx = 0.0f;
    *vy = 0.0f;
    *wz = 0.0f;

    if (g_diagonal_test_elapsed_ms >= CHASSIS_DIAGONAL_TOTAL_TIME_MS)
    {
        return 0U;
    }

    if ((g_diagonal_test_elapsed_ms >= CHASSIS_DIAGONAL_ONE_ROUND_MS) &&
        (g_diagonal_test_elapsed_ms < CHASSIS_DIAGONAL_SECOND_ROUND_MS))
    {
        g_diagonal_test_elapsed_ms += CHASSIS_CONTROL_PERIOD_MS;
        return 1U;
    }

    round_elapsed_ms = g_diagonal_test_elapsed_ms;
    if (round_elapsed_ms >= CHASSIS_DIAGONAL_SECOND_ROUND_MS)
    {
        round_elapsed_ms -= CHASSIS_DIAGONAL_SECOND_ROUND_MS;
    }
    segment = (uint8_t)(round_elapsed_ms / CHASSIS_DIAGONAL_STEP_TIME_MS);
    step_elapsed_ms = round_elapsed_ms % CHASSIS_DIAGONAL_STEP_TIME_MS;

    /* 每次移动2 s后，先发送零速度停车1 s，再进入下一个方向。 */
    if (step_elapsed_ms >= CHASSIS_DIAGONAL_MOVE_TIME_MS)
    {
        g_diagonal_test_elapsed_ms += CHASSIS_CONTROL_PERIOD_MS;
        return 1U;
    }

    switch (segment)
    {
        case 0U: /* 左前315° */
            *vx = component_mps;
            *vy = -component_mps;
            break;
        case 1U: /* 右前45° */
            *vx = component_mps;
            *vy = component_mps;
            break;
        case 2U: /* 右后135° */
            *vx = -component_mps;
            *vy = component_mps;
            break;
        default: /* 左后225° */
            *vx = -component_mps;
            *vy = -component_mps;
            break;
    }

    g_diagonal_test_elapsed_ms += CHASSIS_CONTROL_PERIOD_MS;
    return 1U;
}

static void Chassis_LimitWheelSpeed(float target[4])
{
    float maximum = 0.0f;
    float absolute;
    float scale;
    uint8_t i;

    for (i = 0; i < 4; i++)
    {
        absolute = (target[i] >= 0.0f) ? target[i] : -target[i];
        if (absolute > maximum)
        {
            maximum = absolute;
        }
    }

    if (maximum > CHASSIS_WHEEL_MAX_RPM)
    {
        scale = CHASSIS_WHEEL_MAX_RPM / maximum;
        for (i = 0; i < 4; i++)
        {
            target[i] *= scale;
        }
    }
}

static uint8_t Chassis_AllMotorsOnline(uint32_t now_ms)
{
    uint8_t mask = 0U;
    uint8_t i;
    dji_motor_feedback_t motor;

    for (i = 0; i < 4; i++)
    {
        DJI_Motor_GetFeedback(i, &motor);
        g_chassis_debug.motor_feedback_rpm[i] = motor.speed_rpm;
        g_chassis_debug.motor_feedback_current[i] = motor.given_current;
        g_chassis_debug.motor_temperature[i] = motor.temperature;
        g_chassis_debug.motor_error_code[i] = motor.error_code;
        g_chassis_debug.motor_feedback_age_ms[i] =
            (motor.update_tick_ms == 0U) ? 0xFFFFFFFFU :
            (uint32_t)(now_ms - motor.update_tick_ms);

        if ((motor.update_tick_ms != 0U) &&
            (g_chassis_debug.motor_feedback_age_ms[i] <= CHASSIS_MOTOR_TIMEOUT_MS))
        {
            mask |= (uint8_t)(1U << i);
        }
    }
    g_chassis_debug.motor_online_mask = mask;
    return (uint8_t)(mask == 0x0FU);
}

void Chassis_Init(void)
{
    uint8_t i;
    /* 上电默认不允许导航，必须先收到左开关中档的有效遥控帧。 */
    g_chassis_debug.control_source = CHASSIS_SOURCE_STOP;
    g_chassis_debug.navigation_armed = 0U;
    g_chassis_debug.navigation_waiting_frame = 0U;
    for (i = 0; i < 4; i++)
    {
        PID_Init(&g_speed_pid[i],
                 CHASSIS_SPEED_KP, CHASSIS_SPEED_KI, CHASSIS_SPEED_KD,
                 2000.0f, CHASSIS_CURRENT_MAX);
    }
    Chassis_Stop();
}

void Chassis_Stop(void)
{
    uint8_t i;
    HAL_StatusTypeDef status;

    for (i = 0; i < 4; i++)
    {
        PID_Clear(&g_speed_pid[i]);
        g_chassis_debug.motor_target_rpm[i] = 0.0f;
        g_chassis_debug.motor_current[i] = 0;
        g_stall_time_ms[i] = 0U;
        g_no_output_time_ms[i] = 0U;
    }
    g_chassis_debug.motor_stall_mask = 0U;
    g_chassis_debug.motor_no_output_mask = 0U;
    g_chassis_debug.vx_command = 0.0f;
    g_chassis_debug.vy_command = 0.0f;
    g_chassis_debug.wz_command = 0.0f;
    g_chassis_debug.enabled = 0U;
    status = DJI_Motor_SendChassisCurrent(&hcan1, 0, 0, 0, 0);
    if (status != HAL_OK)
    {
        g_chassis_debug.can_tx_fail_count++;
    }
    g_chassis_debug.can_tx_mailboxes_free =
        (uint8_t)HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
}

void Chassis_Update(void)
{
    remote_control_t remote = {0};
    navigation_command_t navigation;
    chassis_control_source_t source = CHASSIS_SOURCE_STOP;
    dji_motor_feedback_t motor;
    float wheel_linear_mps[4];
    float target[4];
    float vx = 0.0f;
    float vy = 0.0f;
    float wz = 0.0f;
    uint32_t now_ms;
    uint8_t motors_online;
    uint8_t i;

    Remote_Control_Get(&remote);
    Navigation_Get(&navigation);
    now_ms = HAL_GetTick();
    /* 在线状态与本次遥控快照使用同一时间戳，避免混用两帧数据。 */
    g_chassis_debug.remote_online = (uint8_t)(
        (remote.update_tick_ms != 0U) &&
        ((uint32_t)(now_ms - remote.update_tick_ms) <= CHASSIS_REMOTE_TIMEOUT_MS));
    g_chassis_debug.navigation_age_ms = (navigation.valid != 0U) ?
        (uint32_t)(now_ms - navigation.update_tick_ms) : 0xFFFFFFFFU;
    g_chassis_debug.navigation_online = (uint8_t)(
        (navigation.valid != 0U) &&
        (g_chassis_debug.navigation_age_ms <= NAVIGATION_TIMEOUT_MS));
    g_chassis_debug.navigation_frame_count = navigation.frame_count;
    g_chassis_debug.navigation_checksum_errors = navigation.checksum_error_count;
    /* 无论当前来源如何，都刷新四轮反馈并检查电机在线。 */
    motors_online = Chassis_AllMotorsOnline(now_ms);

    if (g_chassis_debug.remote_online != 0U)
    {
        /* 在线但数据无效时停车，不把坏帧当成遥控断线。 */
        if (Remote_Control_DataValid(&remote) != 0U)
        {
            g_chassis_debug.navigation_armed = (uint8_t)(
                remote.switch_position[RC_S_LEFT] == RC_SW_MID);
            if (g_chassis_debug.navigation_armed != 0U)
            {
                source = CHASSIS_SOURCE_REMOTE;
            }
        }
    }
    else if (g_chassis_debug.navigation_armed != 0U)
    {
        /* 遥控超过 100 ms 无新帧，且最后有效左开关为中档，才切导航。 */
        source = CHASSIS_SOURCE_NAVIGATION;
    }

    if (source != g_chassis_debug.control_source)
    {
        g_chassis_debug.control_source = source;
        g_chassis_debug.navigation_waiting_frame =
            (uint8_t)(source == CHASSIS_SOURCE_NAVIGATION);
        g_navigation_entry_frame_count = navigation.frame_count;
        /* 切换来源先清 PID 并输出零电流；下一控制周期使用新来源。 */
        Chassis_Stop();
        return;
    }

    if ((source == CHASSIS_SOURCE_STOP) || (motors_online == 0U))
    {
        Chassis_Stop();
        return;
    }

    if (source == CHASSIS_SOURCE_NAVIGATION)
    {
        if (navigation.frame_count != g_navigation_entry_frame_count)
        {
            g_chassis_debug.navigation_waiting_frame = 0U;
        }
        if ((g_chassis_debug.navigation_waiting_frame != 0U) ||
            (g_chassis_debug.navigation_online == 0U))
        {
            /* 等待新帧/超过 300 ms 无有效帧：清目标、清 PID、撤销驱动力。 */
            Chassis_Stop();
            return;
        }
        for (i = 0U; i < 4U; i++)
        {
            /* 已是 CAN ID1～ID4 的有符号转子 rpm，不再乘安装方向或减速比。 */
            target[i] = (float)navigation.motor_rpm[i];
        }
    }
    else if (CHASSIS_DIRECTION_TEST != 0U)
    {
        /* ID1～ID4全部朝新车头滚动，只用于架空确认电机安装方向。 */
        vx = 0.0f;
        vy = 0.0f;
        wz = 0.0f;
        for (i = 0; i < 4; i++)
        {
            target[i] = CHASSIS_DIRECTION_TEST_RPM * g_motor_direction[i];
        }
    }
    else
    {
        /* 坐标约定：+vx=0°正前，+vy=90°右方；方向角从正前开始顺时针递增。 */
        if (CHASSIS_DIAGONAL_TEST != 0U)
        {
            if (Chassis_GetDiagonalTestCommand(&vx, &vy, &wz) == 0U)
            {
                Chassis_Stop();
                return;
            }
        }
        else
        {
            vx = Chassis_DeadbandNormalize(remote.channel[3]) * CHASSIS_FORWARD_MAX_MPS;
            vy = -Chassis_DeadbandNormalize(remote.channel[2]) * CHASSIS_STRAFE_MAX_MPS;
            wz = Chassis_DeadbandNormalize(remote.channel[4]) * CHASSIS_ROTATION_MAX_RAD_S;
        }
        /* 底盘旋转使用遥控器左侧拨轮（DBUS 第 5 通道）。 */

        /*
         * 单位：vx、vy 为 m/s，wz 为 rad/s，wheel_linear_mps 为 m/s。
         * 新车头下的电机位置：0x201 左后、0x202 左前、0x203 右前、0x204 右后。
         */
        /*
         * 前进时4个轮子都朝新车头滚动；平移相对原车头反向，旋转方向保持原样。
         * 右侧旋转项同号，左侧旋转项反号。
         */
        wheel_linear_mps[MOTOR_ID3_RIGHT_FRONT] =
            vx * SQRT_HALF + vy * SQRT_HALF + wz * CHASSIS_ROTATION_ARM_M;
        wheel_linear_mps[MOTOR_ID4_RIGHT_REAR] =
            vx * SQRT_HALF - vy * SQRT_HALF + wz * CHASSIS_ROTATION_ARM_M;
        wheel_linear_mps[MOTOR_ID1_LEFT_REAR] =
            vx * SQRT_HALF + vy * SQRT_HALF - wz * CHASSIS_ROTATION_ARM_M;
        wheel_linear_mps[MOTOR_ID2_LEFT_FRONT] =
            vx * SQRT_HALF - vy * SQRT_HALF - wz * CHASSIS_ROTATION_ARM_M;

        for (i = 0; i < 4; i++)
        {
            target[i] = wheel_linear_mps[i] * CHASSIS_MPS_TO_MOTOR_RPM *
                        g_motor_direction[i];
        }
    }

    /* 手动与导航统一按现有单轮 4 m/s 上限等比例缩放，保留四轮速度比例。 */
    Chassis_LimitWheelSpeed(target);
    g_chassis_debug.vx_command = vx;
    g_chassis_debug.vy_command = vy;
    g_chassis_debug.wz_command = wz;

    for (i = 0; i < 4; i++)
    {
        DJI_Motor_GetFeedback(i, &motor);
        g_chassis_debug.motor_target_rpm[i] = target[i];
        g_chassis_debug.motor_feedback_rpm[i] = motor.speed_rpm;
        /* 手动回中或导航目标为 0 时清除积分，同时输出零电流。 */
        if (Chassis_AbsFloat(target[i]) < 1.0f)
        {
            PID_Clear(&g_speed_pid[i]);
            g_chassis_debug.motor_current[i] = 0;
        }
        else
        {
            g_chassis_debug.motor_current[i] =
                (int16_t)PID_Update(&g_speed_pid[i], target[i], motor.speed_rpm);
        }

        if ((Chassis_AbsFloat(target[i]) >= CHASSIS_STALL_TARGET_RPM) &&
            (Chassis_AbsInt16(motor.speed_rpm) <= CHASSIS_STALL_SPEED_RPM) &&
            (Chassis_AbsInt16(motor.given_current) >= CHASSIS_STALL_CURRENT))
        {
            if (g_stall_time_ms[i] < CHASSIS_STALL_TIME_MS)
            {
                g_stall_time_ms[i] += CHASSIS_CONTROL_PERIOD_MS;
            }
        }
        else
        {
            g_stall_time_ms[i] = 0U;
            g_chassis_debug.motor_stall_mask &= (uint8_t)~(1U << i);
        }

        if (g_stall_time_ms[i] >= CHASSIS_STALL_TIME_MS)
        {
            g_chassis_debug.motor_stall_mask |= (uint8_t)(1U << i);
        }

        if ((Chassis_AbsFloat(target[i]) >= CHASSIS_STALL_TARGET_RPM) &&
            (Chassis_AbsInt16(g_chassis_debug.motor_current[i]) >=
             CHASSIS_NO_OUTPUT_COMMAND) &&
            (Chassis_AbsInt16(motor.given_current) <= CHASSIS_NO_OUTPUT_CURRENT))
        {
            if (g_no_output_time_ms[i] < CHASSIS_NO_OUTPUT_TIME_MS)
            {
                g_no_output_time_ms[i] += CHASSIS_CONTROL_PERIOD_MS;
            }
        }
        else
        {
            g_no_output_time_ms[i] = 0U;
            g_chassis_debug.motor_no_output_mask &= (uint8_t)~(1U << i);
        }

        if (g_no_output_time_ms[i] >= CHASSIS_NO_OUTPUT_TIME_MS)
        {
            g_chassis_debug.motor_no_output_mask |= (uint8_t)(1U << i);
        }
    }

    g_chassis_debug.enabled = 1U;
    if (DJI_Motor_SendChassisCurrent(&hcan1,
                                     g_chassis_debug.motor_current[0],
                                     g_chassis_debug.motor_current[1],
                                     g_chassis_debug.motor_current[2],
                                     g_chassis_debug.motor_current[3]) != HAL_OK)
    {
        g_chassis_debug.can_tx_fail_count++;
    }
    g_chassis_debug.can_tx_mailboxes_free =
        (uint8_t)HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
}
