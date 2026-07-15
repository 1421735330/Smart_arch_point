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
#include "usart.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum
{
  PWR_WAKE_STATE_IDLE = 0,
  PWR_WAKE_STATE_LOW
} PWR_WAKE_State_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define PWR_WAKE_PERIOD_MS   10000U
#define PWR_WAKE_LOW_TIME_MS 1000U
#define ICM_DRDY_POLL_INTERVAL_MS 10U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* ICM 数据就绪标志, 由 EXTI4 回调置位, 主循环消费 */
static volatile uint8_t g_icm_data_ready = 0;
static volatile uint32_t g_icm_exti_count = 0U;
static volatile uint32_t g_icm_poll_ready_count = 0U;

static PWR_WAKE_State_t g_pwr_wake_state = PWR_WAKE_STATE_IDLE;
static uint32_t g_pwr_wake_last_pulse_ms = 0U;
static uint32_t g_icm_last_drdy_poll_ms = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

static void PWR_WAKE_PulseTimerInit(void);
static void PWR_WAKE_PulseTimerTask(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  GPIO EXTI 回调 (HAL 弱定义的覆盖)
  * @note   PA4 = ICM INT1 数据就绪
  *         PB1 = 你保留的另一路外部中断, 这里仅留接口
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_4)
  {
    g_icm_data_ready = 1;
    g_icm_exti_count++;
  }
  else if (GPIO_Pin == GPIO_PIN_1)
  {
    /* 预留: PB1 EXTI1 用户中断 */
  }
}

static void PWR_WAKE_PulseTimerInit(void)
{
  HAL_GPIO_WritePin(PWR_WAKE_GPIO_Port, PWR_WAKE_Pin, GPIO_PIN_SET);
  g_pwr_wake_state = PWR_WAKE_STATE_IDLE;
  g_pwr_wake_last_pulse_ms = HAL_GetTick();
}

static void PWR_WAKE_PulseTimerTask(void)
{
  uint32_t now_ms = HAL_GetTick();

  if (g_pwr_wake_state == PWR_WAKE_STATE_IDLE)
  {
    if ((uint32_t)(now_ms - g_pwr_wake_last_pulse_ms) >= PWR_WAKE_PERIOD_MS)
    {
      HAL_GPIO_WritePin(PWR_WAKE_GPIO_Port, PWR_WAKE_Pin, GPIO_PIN_RESET);
      g_pwr_wake_state = PWR_WAKE_STATE_LOW;
      g_pwr_wake_last_pulse_ms = now_ms;
    }
  }
  else
  {
    if ((uint32_t)(now_ms - g_pwr_wake_last_pulse_ms) >= PWR_WAKE_LOW_TIME_MS)
    {
      HAL_GPIO_WritePin(PWR_WAKE_GPIO_Port, PWR_WAKE_Pin, GPIO_PIN_SET);
      g_pwr_wake_state = PWR_WAKE_STATE_IDLE;
    }
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

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LPUART1_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  /* ICM-42688-P CS 空闲应为高电平，避免后续 BLE 初始化期间片选有效 */
  HAL_GPIO_WritePin(ICM_CS_PORT, ICM_CS_PIN, GPIO_PIN_SET);

  PWR_WAKE_PulseTimerInit();
  g_icm_last_drdy_poll_ms = HAL_GetTick();

  /* 初始化 VG6328A-MS BLE 从机透传配置 */
  (void)VG6328A_BLE_Init();

  /* 初始化 ICM-42688-P; 失败则进入错误处理 */
  if (ICM42688P_Init() != HAL_OK)
  {
    Error_Handler();
  }

  /* 使能 PA4 EXTI4 中断 (ICM INT1 数据就绪) */
  MX_ICM_INT_NVIC_Enable();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    PWR_WAKE_PulseTimerTask();

    if ((g_icm_data_ready == 0U) &&
        ((uint32_t)(HAL_GetTick() - g_icm_last_drdy_poll_ms) >= ICM_DRDY_POLL_INTERVAL_MS))
    {
      g_icm_last_drdy_poll_ms = HAL_GetTick();
      if (ICM42688P_IsDataReady() != 0U)
      {
        g_icm_data_ready = 1U;
        g_icm_poll_ready_count++;
      }
    }

    if (g_icm_data_ready)
    {
      ICM42688P_RawData_t raw;

      g_icm_data_ready = 0;

      if (ICM42688P_ReadRaw(&raw) == HAL_OK)
      {
        LPUART1_SendFrame(&raw);
      }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPUART1;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
