/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"
#include "imu_system.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "dji_motor.h"
#include "remote_control.h"
#include "chassis.h"
#include "navigation.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IMU_STATUS_LED_PWM 50000U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint32_t chassis_update_tick_ms;
/* IMU迁移：独立保存实际采样时间，正常运行约每2ms更新一次。 */
static uint32_t imu_update_tick_ms;
static uint32_t debug_uart6_tick_ms;
static uint32_t reset_flags;
static uint32_t main_loop_count;
static uint8_t watchdog_enabled;
typedef enum
{
  IMU_LED_WARMING = 0U,
  IMU_LED_STABLE,
  IMU_LED_CALIBRATING,
  IMU_LED_READY
} imu_led_state_t;
static imu_led_state_t imu_led_state;
static uint8_t imu_led_phase;
static uint32_t imu_led_tick;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void Watchdog_Init(void)
{
  uint32_t start_tick;

  /* LSI 约 32 kHz，64 分频、重装值 999，对应约 2 秒超时。 */
  SET_BIT(RCC->CSR, RCC_CSR_LSION);
  start_tick = HAL_GetTick();
  while ((RCC->CSR & RCC_CSR_LSIRDY) == 0U)
  {
    if ((uint32_t)(HAL_GetTick() - start_tick) > 100U)
    {
      return;
    }
  }

  __HAL_DBGMCU_FREEZE_IWDG();
  IWDG->KR = 0x5555U;
  IWDG->PR = 4U;
  IWDG->RLR = 999U;
  start_tick = HAL_GetTick();
  while (IWDG->SR != 0U)
  {
    if ((uint32_t)(HAL_GetTick() - start_tick) > 100U)
    {
      return;
    }
  }
  IWDG->KR = 0xAAAAU;
  IWDG->KR = 0xCCCCU;
  watchdog_enabled = 1U;
}

static void Watchdog_Refresh(void)
{
  if (watchdog_enabled != 0U)
  {
    IWDG->KR = 0xAAAAU;
  }
}

static void Debug_UART6_Print(void)
{
  navigation_command_t navigation;
  const imu_system_t *imu;
  float vx;
  float vy;
  float wz;
  uint32_t now_ms;
  uint8_t received;
  uint8_t valid;
  static char message[320];
  int length;

  /* DMA 未结束时不改写发送缓冲区。 */
  if (huart6.gState != HAL_UART_STATE_READY)
  {
    return;
  }
  Navigation_Get(&navigation);
  imu = IMU_System_GetData();
  now_ms = HAL_GetTick();
  /* 只输出导航接收诊断：rx/valid 分别表示最近 300 ms 有原始数据/有效帧。 */
  received = (uint8_t)((navigation.rx_byte_count != 0U) &&
      ((uint32_t)(now_ms - navigation.last_rx_tick_ms) <= NAVIGATION_TIMEOUT_MS));
  valid = (uint8_t)((navigation.valid != 0U) &&
      ((uint32_t)(now_ms - navigation.update_tick_ms) <= NAVIGATION_TIMEOUT_MS));
  /* AA 56 只有四路轮速：以下速度由最近一帧目标反算，是否过期看 valid。 */
  Chassis_MotorRpmToVelocity(navigation.motor_rpm, &vx, &vy, &wz);

  /* active 表示已切入导航模式；等待新帧或超时停车时仍为 1。 */
  length = snprintf(message, sizeof(message),
                    "NAV:active=%u rx=%u valid=%u bytes=%lu frames=%lu bad=%lu "
                    "vx=%.3f vy=%.3f wz=%.3f "
                    "IMU:ready=%u temp=%.2f G=%.3f,%.3f,%.3f "
                    "YPR=%.2f,%.2f,%.2f DB=%.4f,%.4f,%.4f\r\n",
                    (unsigned int)(g_chassis_debug.control_source == CHASSIS_SOURCE_NAVIGATION),
                    (unsigned int)received,
                    (unsigned int)valid,
                    (unsigned long)navigation.rx_byte_count,
                    (unsigned long)navigation.frame_count,
                    (unsigned long)navigation.checksum_error_count,
                    (double)vx, (double)vy, (double)wz,
                    (unsigned int)imu->initialized,
                    (double)imu->temperature,
                    (double)imu->gyro[0], (double)imu->gyro[1], (double)imu->gyro[2],
                    (double)(imu->ekf.yaw * 57.29578f),
                    (double)(imu->ekf.pitch * 57.29578f),
                    (double)(imu->ekf.roll * 57.29578f),
                    (double)imu->gyro_bias[0],
                    (double)imu->gyro_bias[1],
                    (double)imu->gyro_bias[2]);

  if (length > 0)
  {
    if (length >= (int)sizeof(message))
    {
      length = (int)sizeof(message) - 1;
    }
    /* DMA 非阻塞发送，避免每秒打印时暂停底盘控制。 */
    (void)HAL_UART_Transmit_DMA(&huart6, (uint8_t *)message, (uint16_t)length);
  }
}


/* IMU迁移：用底盘已有 TIM5 RGB 灯显示升温、稳定、校准和就绪状态。 */
static void IMU_StatusLed_Output(uint16_t red, uint16_t green, uint16_t blue)
{
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, blue);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, green);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, red);
}

static void IMU_StatusLed_Set(imu_led_state_t state)
{
  imu_led_state = state;
  imu_led_phase = 1U;
  imu_led_tick = HAL_GetTick();
  if (state == IMU_LED_WARMING)
    IMU_StatusLed_Output(IMU_STATUS_LED_PWM, 0U, 0U);
  else if (state == IMU_LED_STABLE)
    IMU_StatusLed_Output(IMU_STATUS_LED_PWM, IMU_STATUS_LED_PWM, 0U);
  else if (state == IMU_LED_CALIBRATING)
    IMU_StatusLed_Output(0U, 0U, IMU_STATUS_LED_PWM);
  else
    IMU_StatusLed_Output(0U, IMU_STATUS_LED_PWM, 0U);
}

static void IMU_StatusLed_Update(void)
{
  uint32_t now = HAL_GetTick();
  uint32_t period = (imu_led_state == IMU_LED_CALIBRATING) ? 250U : 500U;

  if (imu_led_state == IMU_LED_READY)
  {
    IMU_StatusLed_Output(0U, IMU_STATUS_LED_PWM, 0U);
    return;
  }
  if ((uint32_t)(now - imu_led_tick) >= period)
  {
    imu_led_tick = now;
    imu_led_phase ^= 1U;
    if (imu_led_phase == 0U)
      IMU_StatusLed_Output(0U, 0U, 0U);
    else if (imu_led_state == IMU_LED_WARMING)
      IMU_StatusLed_Output(IMU_STATUS_LED_PWM, 0U, 0U);
    else if (imu_led_state == IMU_LED_STABLE)
      IMU_StatusLed_Output(IMU_STATUS_LED_PWM, IMU_STATUS_LED_PWM, 0U);
    else
      IMU_StatusLed_Output(0U, 0U, IMU_STATUS_LED_PWM);
  }
}

static void IMU_Temperature_SetPWM(uint16_t pwm)
{
  __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, pwm);
}

static uint8_t IMU_HeaterStart(void)
{
  IMU_Temperature_SetPWM(0U);
  return (uint8_t)(HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1) == HAL_OK);
}

static void IMU_StageChanged(imu_system_stage_t stage)
{
  if (stage == IMU_SYSTEM_WARMING)
    IMU_StatusLed_Set(IMU_LED_WARMING);
  else if (stage == IMU_SYSTEM_TEMPERATURE_STABLE)
    IMU_StatusLed_Set(IMU_LED_STABLE);
  else if (stage == IMU_SYSTEM_CALIBRATING)
    IMU_StatusLed_Set(IMU_LED_CALIBRATING);
  else
    IMU_StatusLed_Set(IMU_LED_READY);
}

static void IMU_StartupProgress(void)
{
  IMU_StatusLed_Update();
}

static uint32_t IMU_GetTickMs(void)
{
  return HAL_GetTick();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* 保存上次复位原因；高 7 位分别表示 BOR、引脚、上电、软件和看门狗复位。 */
  reset_flags = RCC->CSR & 0xFE000000U;
  SET_BIT(RCC->CSR, RCC_CSR_RMVF);
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* 导航缓冲必须先于 USB 启动初始化，避免覆盖刚收到的上位机指令。 */
  Navigation_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM5_Init();
  MX_TIM4_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_ADC1_Init();
  MX_ADC3_Init();
  MX_RTC_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_TIM10_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  {
    imu_system_config_t imu_config = {
        .get_tick_ms = IMU_GetTickMs,
        .heater_start = IMU_HeaterStart,
        .heater_set_pwm = IMU_Temperature_SetPWM,
        .stage_changed = IMU_StageChanged,
        .progress = IMU_StartupProgress};
    /* IMU迁移：BMI088自检、恒温、静态校准和初始姿态完成后再启动底盘控制。 */
    if (!IMU_System_Init(&imu_config))
    {
      IMU_Temperature_SetPWM(0U);
      Error_Handler();
    }
  }
  imu_update_tick_ms = HAL_GetTick();
  MX_TIM14_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE: start only the drivers needed by the first chassis test. */
  if (BSP_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  Remote_Control_Init();
  Chassis_Init();
  chassis_update_tick_ms = HAL_GetTick();
  debug_uart6_tick_ms = HAL_GetTick();
  Watchdog_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    main_loop_count++;
    Watchdog_Refresh();
    /* 底盘接收DBUS后，通过USART1 DMA转发给云台板。 */
    Remote_Control_Forward_Update();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* USER CODE: simple 2 ms scheduler; FreeRTOS is intentionally not used yet. */
    if ((uint32_t)(HAL_GetTick() - chassis_update_tick_ms) >= CHASSIS_CONTROL_PERIOD_MS)
    {
      uint32_t now_ms = HAL_GetTick();
      uint32_t imu_elapsed_ms = now_ms - imu_update_tick_ms;
      /* 超时后从当前时刻重新计时，不连续追帧挤满 CAN 邮箱。 */
      chassis_update_tick_ms = now_ms;
      imu_update_tick_ms = now_ms;
      IMU_System_Update((float)imu_elapsed_ms * 0.001f);
      Chassis_Update();
    }

    /* USART6 每秒只发送一行导航接收诊断。 */
    if ((uint32_t)(HAL_GetTick() - debug_uart6_tick_ms) >= 1000U)
    {
      debug_uart6_tick_ms += 1000U;
      Debug_UART6_Print();
    }

    /* IMU状态灯由温控/校准阶段控制；就绪后保持绿灯。 */
    IMU_StatusLed_Update();
    
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
