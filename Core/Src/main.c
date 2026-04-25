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
#include "fdcan.h"
#include "gpdma.h"
#include "icache.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ws2812b.h"
#include "SEGGER_RTT.h"
#include "as5047p.h"
#include "as5047p_port.h"
#include "drv8323rs.h"
#include "motor_open_loop.h"
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
as5047p_handle_t as5047p;
uint16_t current_angle = 0;
volatile uint16_t current_angle_deg = 0;
drv8323rs_t drv8323rs;
motor_pwm_t motor_pwm;
motor_open_loop_t motor_open_loop;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_GPDMA1_Init();
  MX_ICACHE_Init();
  MX_FDCAN1_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
  SEGGER_RTT_Init();
  WS2812B_Init(&htim3, TIM_CHANNEL_2);
  as5047p_port_init(&as5047p);

  if (drv8323rs_init(&drv8323rs,
                     &(drv8323rs_config_t){
                         .hspi = &hspi3,
                         .cs_port = SPI3_CS_GPIO_Port,
                         .cs_pin = SPI3_CS_Pin,
                         .enable_port = ENABLE_GPIO_Port,
                         .enable_pin = ENABLE_Pin,
                     }) != HAL_OK) {
    uint16_t drv_regs[7] = {0};
    RTT_Log("[DRV8323RS] pins: CS=%u MISO=%u ENABLE=%u CAL=%u\n",
            (unsigned int)HAL_GPIO_ReadPin(SPI3_CS_GPIO_Port, SPI3_CS_Pin),
            (unsigned int)HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2),
            (unsigned int)HAL_GPIO_ReadPin(ENABLE_GPIO_Port, ENABLE_Pin),
            (unsigned int)HAL_GPIO_ReadPin(CAL_GPIO_Port, CAL_Pin));
    if (drv8323rs_dump_registers(&drv8323rs, drv_regs, 7U) == HAL_OK) {
      RTT_Log("[DRV8323RS] init failed step=%u hal=%d readback=0x%03X regs:"
              " 0=0x%03X 1=0x%03X 2=0x%03X 3=0x%03X 4=0x%03X 5=0x%03X 6=0x%03X\n",
              (unsigned int)drv8323rs.last_error_step,
              (int)drv8323rs.last_hal_status,
              (unsigned int)drv8323rs.last_readback,
              (unsigned int)drv_regs[0],
              (unsigned int)drv_regs[1],
              (unsigned int)drv_regs[2],
              (unsigned int)drv_regs[3],
              (unsigned int)drv_regs[4],
              (unsigned int)drv_regs[5],
              (unsigned int)drv_regs[6]);
    } else {
      RTT_Log("[DRV8323RS] init failed step=%u hal=%d readback=0x%03X dump failed\n",
              (unsigned int)drv8323rs.last_error_step,
              (int)drv8323rs.last_hal_status,
              (unsigned int)drv8323rs.last_readback);
    }
    while (1) {
      HAL_Delay(1000);
    }
  }

  WS2812B_PowerOnSelfTest(50, 120);

  if (motor_pwm_init(&motor_pwm,
                     &(motor_pwm_config_t){
                         .htim = &htim1,
                         .enable_port = ENABLE_GPIO_Port,
                         .enable_pin = ENABLE_Pin,
                         .bus_voltage = 12.0f,
                     }) != HAL_OK) {
    Error_Handler();
  }

  if (motor_open_loop_init(&motor_open_loop,
                           &(motor_open_loop_config_t){
                               .motor_pwm = &motor_pwm,
                               .bus_voltage = 12.0f,
                               .align_voltage = 3.0f,
                               .align_time_ms = 800U,
                               .voltage_q = 4.0f,
                               .startup_speed_hz = 0.3f,
                               .ramp_time_ms = 1500U,
                               .electrical_speed_hz = 1.0f,
                               .modulation_limit = 0.80f,
                           }) != HAL_OK) {
    Error_Handler();
  }

  if (motor_open_loop_start(&motor_open_loop) != HAL_OK) {
    Error_Handler();
  }

  RTT_Log("[SYSTEM] DRV8323RS ready, open-loop SVPWM started.\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    static uint32_t last_log_ms = 0;
    int8_t status;

    motor_open_loop_update(&motor_open_loop);

    status = as5047p_get_position(&as5047p, without_daec, &current_angle);
    if (status == 0) {
      current_angle_deg = (uint16_t)(((uint32_t)current_angle * 360U) / 16384U);
    }

    if ((HAL_GetTick() - last_log_ms) >= 100U) {
      last_log_ms = HAL_GetTick();
      RTT_Log("cmd_ele=%.1f deg, enc_mech=%u deg, raw=%u\n",
              motor_open_loop_get_electrical_angle_deg(&motor_open_loop),
              (unsigned int)current_angle_deg,
              (unsigned int)current_angle);
    }

    HAL_Delay(1);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the programming delay
  */
  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_2);
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
  WS2812B_TransferCompleteCallback(htim);
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};
  MPU_Attributes_InitTypeDef MPU_AttributesInit = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region 0 and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x08FFF000;
  MPU_InitStruct.LimitAddress = 0x08FFFFFF;
  MPU_InitStruct.AttributesIndex = MPU_ATTRIBUTES_NUMBER0;
  MPU_InitStruct.AccessPermission = MPU_REGION_ALL_RO;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Attribute 0 and the memory to be protected
  */
  MPU_AttributesInit.Number = MPU_ATTRIBUTES_NUMBER0;
  MPU_AttributesInit.Attributes = INNER_OUTER(MPU_NOT_CACHEABLE);

  HAL_MPU_ConfigMemoryAttributes(&MPU_AttributesInit);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
