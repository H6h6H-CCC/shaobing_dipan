#include "imu_system.h"

#include <math.h>
#include <string.h>

#include "BMI088driver.h"
#include "imu_temperature_control.h"

#define IMU_TEMPERATURE_TARGET_C 39.5f
#define IMU_TEMPERATURE_PI_START_PERCENT 90.0f /* 达到目标温度的此百分比时切入 PI。 */
#define IMU_TEMPERATURE_STABLE_ERROR 0.25f
#define IMU_TEMPERATURE_STABLE_MS 2000U
#define IMU_TEMPERATURE_COLD_START_THRESHOLD_C 25.0f
#define IMU_TEMPERATURE_COLD_START_FEED_FORWARD_PWM 1100.0f
#define IMU_TEMPERATURE_NORMAL_START_FEED_FORWARD_PWM 900.0f
#define IMU_GYRO_CALIBRATION_SAMPLES 6000U
#define IMU_STATIC_GYRO_MAX_RAD_S 0.05f
#define IMU_STATIC_ACCEL_TOLERANCE 0.5f
#define IMU_SAMPLE_PERIOD_MS 2U
#define IMU_TWO_PI 6.28318530718f

static imu_system_t g_imu;
static imu_system_config_t g_config;
static imu_temperature_controller_t g_temperature_controller;
static imu_system_stage_t g_stage;
static uint8_t g_stage_initialized;

static void IMU_System_SetStage(imu_system_stage_t stage)
{
  if (g_stage_initialized && g_stage == stage) return;
  g_stage = stage;
  g_stage_initialized = 1U;
  if (g_config.stage_changed != NULL) g_config.stage_changed(stage);
}

static void IMU_System_Progress(void)
{
  if (g_config.progress != NULL) g_config.progress();
}

static void IMU_System_ReadSensor(void)
{
  /* 修改：加速度仅用于启动静止判断，移除姿态解算专用低通滤波。 */
  BMI088_read(g_imu.gyro, g_imu.accel, &g_imu.temperature);
}

static void IMU_System_UpdateTemperature(float dt)
{
  IMU_TemperatureController_Update(&g_temperature_controller,
                                  g_imu.temperature, dt);
}

static uint8_t IMU_System_TemperatureInRange(void)
{
  return (uint8_t)(
      IMU_TemperatureController_PIStarted(&g_temperature_controller) &&
      fabsf(g_imu.temperature - g_temperature_controller.target_temp) <=
          IMU_TEMPERATURE_STABLE_ERROR);
}

static void IMU_System_WaitForTemperature(void)
{
  uint32_t last_tick = g_config.get_tick_ms();
  uint32_t stable_start_tick = 0U;

  for (;;)
  {
    uint32_t now = g_config.get_tick_ms();
    uint32_t elapsed_ms = now - last_tick;
    if (elapsed_ms < IMU_SAMPLE_PERIOD_MS) continue;

    last_tick = now;
    g_imu.dt_us = elapsed_ms * 1000U;
    IMU_System_ReadSensor();
    IMU_System_UpdateTemperature((float)elapsed_ms * 0.001f);
    IMU_System_Progress();

    if (IMU_System_TemperatureInRange())
    {
      IMU_System_SetStage(IMU_SYSTEM_TEMPERATURE_STABLE);
      if (stable_start_tick == 0U) stable_start_tick = now;
      if ((uint32_t)(now - stable_start_tick) >= IMU_TEMPERATURE_STABLE_MS) return;
    }
    else
    {
      IMU_System_SetStage(IMU_SYSTEM_WARMING);
      stable_start_tick = 0U;
    }
  }
}

static void IMU_System_CalibrateGyro(void)
{
  const float zero_offset[3] = {0.0f, 0.0f, 0.0f};
  float sum_z = 0.0f;
  uint16_t valid_samples = 0U;
  uint32_t last_tick = g_config.get_tick_ms();

  BMI088_set_gyro_offset(zero_offset);
  IMU_System_SetStage(IMU_SYSTEM_CALIBRATING);
  while (valid_samples < IMU_GYRO_CALIBRATION_SAMPLES)
  {
    uint32_t now = g_config.get_tick_ms();
    uint32_t elapsed_ms = now - last_tick;
    if (elapsed_ms < IMU_SAMPLE_PERIOD_MS) continue;

    last_tick = now;
    g_imu.dt_us = elapsed_ms * 1000U;
    IMU_System_ReadSensor();
    IMU_System_UpdateTemperature((float)elapsed_ms * 0.001f);
    IMU_System_Progress();

    if (!IMU_System_TemperatureInRange())
    {
      valid_samples = 0U;
      sum_z = 0.0f;
      IMU_System_WaitForTemperature();
      IMU_System_SetStage(IMU_SYSTEM_CALIBRATING);
      last_tick = g_config.get_tick_ms();
      continue;
    }

    float gyro_norm = sqrtf(g_imu.gyro[0] * g_imu.gyro[0] +
                            g_imu.gyro[1] * g_imu.gyro[1] +
                            g_imu.gyro[2] * g_imu.gyro[2]);
    float accel_norm = sqrtf(g_imu.accel[0] * g_imu.accel[0] +
                             g_imu.accel[1] * g_imu.accel[1] +
                             g_imu.accel[2] * g_imu.accel[2]);
    if (gyro_norm < IMU_STATIC_GYRO_MAX_RAD_S &&
        fabsf(accel_norm - 9.80665f) < IMU_STATIC_ACCEL_TOLERANCE)
    {
      /* 修改：三轴共同判断静止，只累计Z轴零偏。 */
      sum_z += g_imu.gyro[2];
      valid_samples++;
    }
    else
    {
      valid_samples = 0U;
      sum_z = 0.0f;
    }
  }

  g_imu.gyro_bias_z = sum_z / (float)IMU_GYRO_CALIBRATION_SAMPLES;
  {
    /* 修改：只在驱动层减一次Z零偏；X/Y不标定，运行期间不更新偏置。 */
    const float offset[3] = {0.0f, 0.0f, g_imu.gyro_bias_z};
    BMI088_set_gyro_offset(offset);
  }
}

uint8_t IMU_System_Init(const imu_system_config_t *config)
{
  if (config == NULL || config->get_tick_ms == NULL ||
      config->heater_start == NULL || config->heater_set_pwm == NULL)
    return 0U;

  memset(&g_imu, 0, sizeof(g_imu));
  g_config = *config;
  g_stage_initialized = 0U;

  IMU_TemperatureController_Init(&g_temperature_controller,
                                g_config.heater_set_pwm);
  g_temperature_controller.target_temp = IMU_TEMPERATURE_TARGET_C;
  g_temperature_controller.pi_start_temp =
      g_temperature_controller.target_temp *
      (IMU_TEMPERATURE_PI_START_PERCENT / 100.0f);
  IMU_System_SetStage(IMU_SYSTEM_WARMING);
  if (BMI088_init() != BMI088_NO_ERROR) return 0U;

  /* 修改：加热前读取启动温度，冷启动和常温启动使用不同前馈。 */
  IMU_System_ReadSensor();
  g_temperature_controller.feed_forward_pwm =
      (g_imu.temperature < IMU_TEMPERATURE_COLD_START_THRESHOLD_C) ?
          IMU_TEMPERATURE_COLD_START_FEED_FORWARD_PWM :
          IMU_TEMPERATURE_NORMAL_START_FEED_FORWARD_PWM;

  if (!g_config.heater_start()) return 0U;

  IMU_System_WaitForTemperature();
  IMU_System_CalibrateGyro();
  /* 修改：无需四元数初始化；校准结束时Yaw从0开始，温控继续闭环。 */
  g_imu.yaw_rad = 0.0f;
  g_imu.yaw_total_rad = 0.0f;
  IMU_System_ReadSensor();
  g_imu.yaw_rate_rad_s = g_imu.gyro[2];

  g_imu.initialized = 1U;
  IMU_System_SetStage(IMU_SYSTEM_READY);
  IMU_System_Progress();
  return 1U;
}

void IMU_System_Update(float dt)
{
  float delta_yaw;
  if (!g_imu.initialized) return;
  g_imu.dt_us = (uint32_t)lroundf(dt * 1000000.0f);
  IMU_System_ReadSensor();
  IMU_System_UpdateTemperature(dt);
  /* 修改：gyro[2]已经扣除启动Z零偏，不再重复扣除；慢速旋转也正常积分。 */
  g_imu.yaw_rate_rad_s = g_imu.gyro[2];
  delta_yaw = g_imu.yaw_rate_rad_s * dt;
  g_imu.yaw_total_rad += delta_yaw;
  g_imu.yaw_rad = remainderf(g_imu.yaw_rad + delta_yaw, IMU_TWO_PI);
}

const imu_system_t *IMU_System_GetData(void)
{
  return &g_imu;
}
