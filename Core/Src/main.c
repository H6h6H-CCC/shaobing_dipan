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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint32_t chassis_update_tick_ms;
static uint32_t debug_uart6_tick_ms;
static uint32_t heartbeat_tick_ms;
static uint32_t reset_flags;
static uint32_t main_loop_count;
static uint8_t watchdog_enabled;
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
  uint32_t now_ms;
  uint8_t received;
  uint8_t valid;
  static char message[128];
  int length;

  /* DMA 未结束时不改写发送缓冲区。 */
  if (huart6.gState != HAL_UART_STATE_READY)
  {
    return;
  }
  Navigation_Get(&navigation);
  now_ms = HAL_GetTick();
  /* 只输出导航接收诊断：rx/valid 分别表示最近 300 ms 有原始数据/有效帧。 */
  received = (uint8_t)((navigation.rx_byte_count != 0U) &&
      ((uint32_t)(now_ms - navigation.last_rx_tick_ms) <= NAVIGATION_TIMEOUT_MS));
  valid = (uint8_t)((navigation.valid != 0U) &&
      ((uint32_t)(now_ms - navigation.update_tick_ms) <= NAVIGATION_TIMEOUT_MS));

  /* active 表示已切入导航模式；等待新帧或超时停车时仍为 1。 */
  length = snprintf(message, sizeof(message),
                    "NAV:active=%u rx=%u valid=%u bytes=%lu frames=%lu bad=%lu\r\n",
                    (unsigned int)(g_chassis_debug.control_source == CHASSIS_SOURCE_NAVIGATION),
                    (unsigned int)received,
                    (unsigned int)valid,
                    (unsigned long)navigation.rx_byte_count,
                    (unsigned long)navigation.frame_count,
                    (unsigned long)navigation.checksum_error_count);

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
  heartbeat_tick_ms = HAL_GetTick();
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
      /* 超时后从当前时刻重新计时，不连续追帧挤满 CAN 邮箱。 */
      chassis_update_tick_ms = HAL_GetTick();
      Chassis_Update();
    }

    /* USART6 每秒只发送一行导航接收诊断。 */
    if ((uint32_t)(HAL_GetTick() - debug_uart6_tick_ms) >= 1000U)
    {
      debug_uart6_tick_ms += 1000U;
      Debug_UART6_Print();
    }

    /* 绿色 LED 每 500 ms 翻转一次，串口停止时仍可观察主循环是否存活。 */
    if ((uint32_t)(HAL_GetTick() - heartbeat_tick_ms) >= 500U)
    {
      heartbeat_tick_ms = HAL_GetTick();
      HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
    }
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
