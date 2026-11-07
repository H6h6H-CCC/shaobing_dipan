#ifndef IMU_SYSTEM_H
#define IMU_SYSTEM_H

#include <stdint.h>

typedef enum
{
  IMU_SYSTEM_WARMING = 0U,
  IMU_SYSTEM_TEMPERATURE_STABLE,
  IMU_SYSTEM_CALIBRATING,
  IMU_SYSTEM_READY
} imu_system_stage_t;

typedef struct
{
  uint32_t (*get_tick_ms)(void);
  uint8_t (*heater_start)(void);
  void (*heater_set_pwm)(uint16_t pwm);
  void (*stage_changed)(imu_system_stage_t stage);
  void (*progress)(void);
} imu_system_config_t;

typedef struct
{
  /* 修改：三轴原始量保留用于启动静止判断，不再进行Pitch/Roll或EKF解算。 */
  float accel[3];
  float gyro[3];              /* rad/s，校准后Z轴已由驱动扣除静态零偏。 */
  float gyro_bias_z;          /* rad/s，仅启动静态校准时写入，运行期间冻结。 */
  float yaw_rate_rad_s;      /* rad/s，逆时针为正。 */
  float yaw_rad;             /* rad，归一化到[-pi, pi]。 */
  float yaw_total_rad;       /* rad，上电校准完成后从0累计，不限制圈数。 */
  float temperature;
  uint32_t dt_us;
  uint8_t initialized;
} imu_system_t;

uint8_t IMU_System_Init(const imu_system_config_t *config);
void IMU_System_Update(float dt);
const imu_system_t *IMU_System_GetData(void);

#endif
