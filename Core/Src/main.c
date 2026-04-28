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
#include "drv8323rs_current.h"
#include "motor_encoder_align.h"
#include "motor_encoder_drive.h"
#include "motor_open_loop.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR_POLE_PAIRS 14U
#define MOTOR_BUS_VOLTAGE 12.0f
#define MOTOR_ALIGN_VOLTAGE 1.5f
#define MOTOR_MODULATION_LIMIT 0.80f
#define MOTOR_ENCODER_ALIGN_TIME_MS 1000U
#define MOTOR_ENCODER_ALIGN_SAMPLE_COUNT 32U
#define MOTOR_RUN_VOLTAGE_Q 2.0f
#define MOTOR_ELECTRICAL_SPEED_HZ 5.0f
#define MOTOR_ENCODER_DIRECTION 1
#define DRV8323RS_CURRENT_VREF_MV 3300U
#define DRV8323RS_CURRENT_SHUNT_OHM 0.010f
#define DRV8323RS_CURRENT_CSA_GAIN_VV 10.0f
#define DRV8323RS_CURRENT_CALIBRATION_SAMPLES 256U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
as5047p_handle_t as5047p;
uint16_t current_angle = 0;
uint32_t current_angle_mdeg = 0;
uint32_t encoder_electrical_mdeg = 0;
drv8323rs_current_amps_t phase_current_amps = {0.0f, 0.0f, 0.0f};
drv8323rs_t drv8323rs;
drv8323rs_current_t drv8323rs_current;
volatile uint16_t drv8323rs_current_dma_buffer[3] = {2048U, 2048U, 2048U};
motor_pwm_t motor_pwm;
motor_open_loop_t motor_open_loop;
motor_encoder_align_t motor_encoder_align;
motor_encoder_drive_t motor_encoder_drive;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint32_t AS5047P_RawToMilliDegrees(uint16_t raw)
{
  return motor_encoder_align_raw_to_mdeg(raw);
}

static int32_t AmpsToMilliamps(float amps)
{
  return (int32_t)((amps >= 0.0f) ? ((amps * 1000.0f) + 0.5f)
                                  : ((amps * 1000.0f) - 0.5f));
}

static void PrintMilliamps(int32_t milliamps)
{
  if (milliamps < 0) {
    RTT_Log("-%ld.%03ld", (long)((-milliamps) / 1000),
            (long)((-milliamps) % 1000));
  } else {
    RTT_Log("%ld.%03ld", (long)(milliamps / 1000),
            (long)(milliamps % 1000));
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
                         .csa_gain = DRV8323RS_CSA_GAIN_10VV,
                         .sense_ocp_enable = 0U,
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
                         .bus_voltage = MOTOR_BUS_VOLTAGE,
                     }) != HAL_OK) {
    Error_Handler();
  }

  if (motor_open_loop_init(&motor_open_loop,
                           &(motor_open_loop_config_t){
                               .motor_pwm = &motor_pwm,
                               .bus_voltage = MOTOR_BUS_VOLTAGE,
                               .align_voltage = MOTOR_ALIGN_VOLTAGE,
                               .align_time_ms = 800U,
                               .voltage_q = 4.0f,
                               .startup_speed_hz = 0.3f,
                               .ramp_time_ms = 1500U,
                               .electrical_speed_hz = 1.0f,
                               .modulation_limit = MOTOR_MODULATION_LIMIT,
                           }) != HAL_OK) {
    Error_Handler();
  }

  if (drv8323rs_current_init(
          &drv8323rs_current,
          &(drv8323rs_current_config_t){
              .hadc = &hadc1,
              .vref_mv = DRV8323RS_CURRENT_VREF_MV,
              .shunt_ohm = DRV8323RS_CURRENT_SHUNT_OHM,
              .csa_gain_vv = DRV8323RS_CURRENT_CSA_GAIN_VV,
              .calibration_samples = DRV8323RS_CURRENT_CALIBRATION_SAMPLES,
              .dma_buffer = drv8323rs_current_dma_buffer,
              .dma_buffer_length = 3U,
          }) != HAL_OK) {
    Error_Handler();
  }

  if (drv8323rs_current_start_dma(&drv8323rs_current) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK) {
    Error_Handler();
  }

  HAL_Delay(5U);

  if (drv8323rs_current_calibrate_offsets(&drv8323rs_current) != HAL_OK) {
    Error_Handler();
  }

  if (motor_encoder_align_init(
          &motor_encoder_align,
          &(motor_encoder_align_config_t){
              .encoder = &as5047p,
              .motor_pwm = &motor_pwm,
              .pole_pairs = MOTOR_POLE_PAIRS,
              .bus_voltage = MOTOR_BUS_VOLTAGE,
              .align_voltage = MOTOR_ALIGN_VOLTAGE,
              .modulation_limit = MOTOR_MODULATION_LIMIT,
              .align_time_ms = MOTOR_ENCODER_ALIGN_TIME_MS,
              .sample_count = MOTOR_ENCODER_ALIGN_SAMPLE_COUNT,
          }) != HAL_OK) {
    Error_Handler();
  }

  if (motor_encoder_drive_init(
          &motor_encoder_drive,
          &(motor_encoder_drive_config_t){
              .align = &motor_encoder_align,
              .voltage_q = MOTOR_RUN_VOLTAGE_Q,
              .electrical_speed_hz = MOTOR_ELECTRICAL_SPEED_HZ,
              .bus_voltage = MOTOR_BUS_VOLTAGE,
              .modulation_limit = MOTOR_MODULATION_LIMIT,
              .sensor_direction = MOTOR_ENCODER_DIRECTION,
          }) != HAL_OK) {
    Error_Handler();
  }

  {
    uint16_t aligned_mech_raw = 0;

    if (motor_encoder_align_run(&motor_encoder_align, &aligned_mech_raw) !=
        HAL_OK) {
      RTT_Log("[ALIGN] failed, PWM stopped.\n");
      Error_Handler();
    }
  }

  if (motor_encoder_drive_start(&motor_encoder_drive) != HAL_OK) {
    Error_Handler();
  }

  HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_SET);
  RTT_Log("[ANGLE] offset=%u\n",
          (unsigned int)motor_encoder_align_get_offset_raw(
              &motor_encoder_align));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    static uint32_t last_log_ms = 0;
    int8_t status;

    (void)motor_encoder_drive_update(&motor_encoder_drive);

    status = as5047p_get_position(&as5047p, with_daec, &current_angle);
    if (status == 0) {
      current_angle_mdeg = AS5047P_RawToMilliDegrees(current_angle);
      encoder_electrical_mdeg =
          AS5047P_RawToMilliDegrees(motor_encoder_align_get_elec_raw(
              &motor_encoder_align, current_angle));
    }

    if ((HAL_GetTick() - last_log_ms) >= 250U) {
      int32_t ia_ma = 0;
      int32_t ib_ma = 0;
      int32_t ic_ma = 0;

      last_log_ms = HAL_GetTick();
      if (drv8323rs_current_read_amps(&drv8323rs_current,
                                      &phase_current_amps) == HAL_OK) {
        ia_ma = AmpsToMilliamps(phase_current_amps.a);
        ib_ma = AmpsToMilliamps(phase_current_amps.b);
        ic_ma = AmpsToMilliamps(phase_current_amps.c);
      }

      RTT_Log("[ANGLE2] cmd=%u.%03u ele=%u.%03u mech=%u.%03u raw=%u off=%u Iabc=",
              (unsigned int)(motor_encoder_drive_get_electrical_mdeg(
                                 &motor_encoder_drive) /
                             1000U),
              (unsigned int)(motor_encoder_drive_get_electrical_mdeg(
                                 &motor_encoder_drive) %
                             1000U),
              (unsigned int)(encoder_electrical_mdeg / 1000U),
              (unsigned int)(encoder_electrical_mdeg % 1000U),
              (unsigned int)(current_angle_mdeg / 1000U),
              (unsigned int)(current_angle_mdeg % 1000U),
              (unsigned int)current_angle,
              (unsigned int)motor_encoder_align_get_offset_raw(
                  &motor_encoder_align));
      PrintMilliamps(ia_ma);
      RTT_Log(",");
      PrintMilliamps(ib_ma);
      RTT_Log(",");
      PrintMilliamps(ic_ma);
      RTT_Log("A\n");
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
