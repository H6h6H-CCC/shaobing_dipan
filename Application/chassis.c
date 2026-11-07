#include "chassis.h"
#include "dji_motor.h"
#include "remote_control.h"
#include "navigation.h"
#include "gimbal_heading_link.h"
#include "pid.h"
#include "can.h"

#include <math.h>

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

/* 修改：主人指定前后、左右速度上限统一2m/s，普通/小陀螺行进共用。 */
#define CHASSIS_FORWARD_MAX_MPS     2.0f
#define CHASSIS_STRAFE_MAX_MPS      2.0f
#define CHASSIS_ROTATION_MAX_RAD_S  4.0f
/* 修改：匹配老工程低档的底盘转速约19.74 rpm，右上为正、右下为负。
 * 老工程5000控制量 -> 每轮1250 rpm -> /19 * 0.075/0.25 = 19.74底盘rpm。 */
#define CHASSIS_SMALL_GYRO_MAX_RAD_S 2.067f
#define CHASSIS_SMALL_GYRO_RAMP_RAD_S_PER_S 3.0f
/* 与原 Chassis 工程一致：单轮线速度限幅 4 m/s，按本车轮径和减速比换算为转子转速。 */
#define CHASSIS_WHEEL_MAX_MPS       4.0f
#define CHASSIS_WHEEL_MAX_RPM       (CHASSIS_WHEEL_MAX_MPS * CHASSIS_MPS_TO_MOTOR_RPM)
#define CHASSIS_CURRENT_MAX         14000.0f

/* 实车车头与当前逻辑坐标相差180°；仅翻转平移，不改变原地旋转方向。 */
#define CHASSIS_HEADING_REVERSED    1U

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

/* 方向修正：运动学+wz与两套IMU同号，均为俯视逆时针Yaw增大。 */
#define CHASSIS_YAW_RATE_SIGN_FROM_WZ (1.0f)

volatile chassis_debug_t g_chassis_debug;
static pid_t g_speed_pid[4];
static uint16_t g_stall_time_ms[4];
static uint16_t g_no_output_time_ms[4];
static uint32_t g_diagonal_test_elapsed_ms;
static uint32_t g_navigation_entry_frame_count;
static float g_small_gyro_wz_command;

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

void Chassis_MotorRpmToVelocity(const int16_t motor_rpm[4],
                              float *vx, float *vy, float *wz)
{
    float wheel[4];
    uint8_t i;

    /* 撤回安装方向和减速比换算，只用于串口显示，不影响导航控制目标。 */
    for (i = 0U; i < 4U; i++)
    {
        wheel[i] = (float)motor_rpm[i] /
                   (CHASSIS_MPS_TO_MOTOR_RPM * g_motor_direction[i]);
    }
    *vx = (wheel[0] + wheel[1] + wheel[2] + wheel[3]) / (4.0f * SQRT_HALF);
    *vy = (wheel[0] - wheel[1] + wheel[2] - wheel[3]) / (4.0f * SQRT_HALF);
    *wz = (-wheel[0] - wheel[1] + wheel[2] + wheel[3]) /
          (4.0f * CHASSIS_ROTATION_ARM_M);
}

float Chassis_GetActualZSpeedRpm(void)
{
    int16_t motor_rpm[4];
    float vx;
    float vy;
    float wz;
    uint8_t i;

    for (i = 0U; i < 4U; i++)
    {
        motor_rpm[i] = g_chassis_debug.motor_feedback_rpm[i];
    }
    Chassis_MotorRpmToVelocity(motor_rpm, &vx, &vy, &wz);
    /* A5统一使用与BMI088相同的符号：逆时针Yaw增大。 */
    return wz * CHASSIS_YAW_RATE_SIGN_FROM_WZ * 60.0f /
           (2.0f * PI_F);
}

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
    g_chassis_debug.small_gyro_active = 0U;
    g_small_gyro_wz_command = 0.0f;
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
    g_chassis_debug.small_gyro_active = 0U;
    g_small_gyro_wz_command = 0.0f;
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
    gimbal_heading_command_t heading = {0};
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
    Gimbal_Heading_Get(&heading);
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
    /* 修改：heading为最终枪口相对底盘的角度，逆时针正；不直接相减两套IMU。 */
    g_chassis_debug.gimbal_heading_deg = (float)heading.angle_cdeg * 0.01f;
    g_chassis_debug.gimbal_heading_age_ms = (heading.frame_count != 0U) ?
        (uint32_t)(now_ms - heading.update_tick_ms) : UINT32_MAX;
    g_chassis_debug.gimbal_heading_valid = (uint8_t)((heading.valid != 0U) &&
        (g_chassis_debug.gimbal_heading_age_ms <= GIMBAL_HEADING_TIMEOUT_MS));
    /* 无论当前来源如何，都刷新四轮反馈并检查电机在线。 */
    motors_online = Chassis_AllMotorsOnline(now_ms);
    g_chassis_debug.actual_wz_rad_s = Chassis_GetActualZSpeedRpm() *
                                      (2.0f * PI_F) / 60.0f;

    if (g_chassis_debug.remote_online != 0U)
    {
        /* 在线但数据无效时停车，不把坏帧当成遥控断线。 */
        if (Remote_Control_DataValid(&remote) != 0U)
        {
            g_chassis_debug.navigation_armed = (uint8_t)(
                remote.switch_position[RC_S_LEFT] == RC_SW_MID);
            /* 左上进入小陀螺控制，左中保留普通遥控，左下停车。 */
            if ((remote.switch_position[RC_S_LEFT] == RC_SW_MID) ||
                (remote.switch_position[RC_S_LEFT] == RC_SW_UP))
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

    /* 修改：遥控行进必须先标定枪口正前零位并保持回传在线，失效时整车停车。
     * 导航已给出四轮rpm，保持原协议和控制方式，不要求云台朝向有效。 */
    if ((source == CHASSIS_SOURCE_REMOTE) &&
        (CHASSIS_DIRECTION_TEST == 0U) && (CHASSIS_DIAGONAL_TEST == 0U) &&
        (g_chassis_debug.gimbal_heading_valid == 0U))
    {
        Chassis_Stop();
        /* 修改：停车后仍有实际余转时，让ID1继续反向补偿到轮速接近零。 */
        g_chassis_debug.small_gyro_active = (uint8_t)(
            (remote.switch_position[RC_S_LEFT] == RC_SW_UP) &&
            (Chassis_AbsFloat(g_chassis_debug.actual_wz_rad_s) > 0.03f));
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
            float forward = Chassis_DeadbandNormalize(remote.channel[3]) *
                            CHASSIS_FORWARD_MAX_MPS;
            float right = -Chassis_DeadbandNormalize(remote.channel[2]) *
                           CHASSIS_STRAFE_MAX_MPS;
            float relative_yaw_rad = g_chassis_debug.gimbal_heading_deg *
                                     PI_F / 180.0f;
            float heading_cos = cosf(relative_yaw_rad);
            float heading_sin = sinf(relative_yaw_rad);

            /* 修改：普通行进/小陀螺共用枪口方向平移；+X前、+Y右、角度逆时针正。
             * 此处只作坐标变换，下方保留唯一一次原车头180度安装修正。 */
            vx = forward * heading_cos + right * heading_sin;
            vy = -forward * heading_sin + right * heading_cos;
            if (remote.switch_position[RC_S_LEFT] == RC_SW_UP)
            {
                float target_small_gyro_wz = 0.0f;
                float step = CHASSIS_SMALL_GYRO_RAMP_RAD_S_PER_S *
                             ((float)CHASSIS_CONTROL_PERIOD_MS * 0.001f);

                /* 修改：右上正转、右下反转、右中只停自转，左摇杆仍按枪口方向平移。 */
                if (remote.switch_position[RC_S_RIGHT] == RC_SW_UP)
                {
                    target_small_gyro_wz = CHASSIS_SMALL_GYRO_MAX_RAD_S;
                }
                else if (remote.switch_position[RC_S_RIGHT] == RC_SW_DOWN)
                {
                    target_small_gyro_wz = -CHASSIS_SMALL_GYRO_MAX_RAD_S;
                }
                if (g_small_gyro_wz_command < target_small_gyro_wz - step)
                {
                    g_small_gyro_wz_command += step;
                }
                else if (g_small_gyro_wz_command > target_small_gyro_wz + step)
                {
                    g_small_gyro_wz_command -= step;
                }
                else
                {
                    g_small_gyro_wz_command = target_small_gyro_wz;
                }
                wz = g_small_gyro_wz_command;
                /* 修改：右中减速期间继续补偿，轮速接近零后再撤销ID1补偿。 */
                g_chassis_debug.small_gyro_active = (uint8_t)(
                    (target_small_gyro_wz != 0.0f) ||
                    (g_small_gyro_wz_command != 0.0f) ||
                    (Chassis_AbsFloat(g_chassis_debug.actual_wz_rad_s) > 0.03f));
            }
            else
            {
                g_small_gyro_wz_command = 0.0f;
                wz = Chassis_DeadbandNormalize(remote.channel[4]) * CHASSIS_ROTATION_MAX_RAD_S;
                g_chassis_debug.small_gyro_active = 0U;
            }
        }

#if CHASSIS_HEADING_REVERSED
        /* 当前逻辑车头与实车车头相反：前后、左右平移一起取反，旋转保持原方向。 */
        vx = -vx;
        vy = -vy;
#endif
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
