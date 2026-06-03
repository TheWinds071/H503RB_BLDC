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
#include "motor_can.h"
#include "motor_foc.h"
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
#define MOTOR_ENCODER_ALIGN_ON_STARTUP 0U
#define MOTOR_ENCODER_FIXED_OFFSET_RAW 7052U
#define MOTOR_OPEN_LOOP_RUN_VOLTAGE_Q 4.0f
#define MOTOR_RUN_VOLTAGE_Q 2.0f
#define MOTOR_ELECTRICAL_SPEED_HZ 5.0f
#define MOTOR_ENCODER_DIRECTION 1
#define MOTOR_POSITION_STEP_MODE 0U
#define MOTOR_POSITION_START_DEG 0
#define MOTOR_POSITION_STEP_DEG 10
#define MOTOR_POSITION_REV_MDEG 360000
#define MOTOR_POSITION_STEP_SETTLE_MDEG 500
#define MOTOR_POSITION_STEP_RESET_MDEG 2000
#define MOTOR_POSITION_STEP_HOLD_MS 300U
#define MOTOR_CONTROL_ADC_TICK_S 0.00005f
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
uint32_t encoder_electrical_mdeg = 0;
drv8323rs_current_amps_t phase_current_amps = {0.0f, 0.0f, 0.0f};
drv8323rs_t drv8323rs;
drv8323rs_current_t drv8323rs_current;
volatile uint16_t drv8323rs_current_dma_buffer[3] = {2048U, 2048U, 2048U};
motor_pwm_t motor_pwm;
motor_open_loop_t motor_open_loop;
motor_encoder_align_t motor_encoder_align;
motor_encoder_drive_t motor_encoder_drive;
volatile uint8_t motor_control_adc_sync_enabled = 0U;
volatile uint8_t motor_control_adc_tick_pending = 0U;
volatile uint32_t motor_control_adc_tick_count = 0U;
volatile uint32_t motor_control_adc_tick_overrun = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static int32_t AmpsToMilliamps(float amps)
{
  return (int32_t)((amps >= 0.0f) ? ((amps * 1000.0f) + 0.5f)
                                  : ((amps * 1000.0f) - 0.5f));
}

static int32_t RoundToNearestRevolutionMdeg(int32_t mdeg)
{
  if (mdeg >= 0) {
    return ((mdeg + (MOTOR_POSITION_REV_MDEG / 2)) /
            MOTOR_POSITION_REV_MDEG) *
           MOTOR_POSITION_REV_MDEG;
  }

  return ((mdeg - (MOTOR_POSITION_REV_MDEG / 2)) /
          MOTOR_POSITION_REV_MDEG) *
         MOTOR_POSITION_REV_MDEG;
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
                               .voltage_q = MOTOR_OPEN_LOOP_RUN_VOLTAGE_Q,
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
  {
    drv8323rs_current_raw_t offset_raw = {0U, 0U, 0U};

    if (drv8323rs_current_get_offsets(&drv8323rs_current, &offset_raw) ==
        HAL_OK) {
      RTT_Log("[CURRENT] offset raw=%u,%u,%u\n",
              (unsigned int)offset_raw.a,
              (unsigned int)offset_raw.b,
              (unsigned int)offset_raw.c);
    }
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

  if (MOTOR_ENCODER_ALIGN_ON_STARTUP != 0U) {
    uint16_t aligned_mech_raw = 0;

    if (motor_encoder_align_run(&motor_encoder_align, &aligned_mech_raw) !=
        HAL_OK) {
      RTT_Log("[ALIGN] failed, PWM stopped.\n");
      Error_Handler();
    }
    RTT_Log("[ALIGN] mech_raw=%u offset=%u\n",
            (unsigned int)aligned_mech_raw,
            (unsigned int)motor_encoder_align_get_offset_raw(
                &motor_encoder_align));
  } else {
    if (motor_encoder_align_set_offset_raw(&motor_encoder_align,
                                           MOTOR_ENCODER_FIXED_OFFSET_RAW) !=
        HAL_OK) {
      Error_Handler();
    }
    RTT_Log("[ALIGN] skipped fixed_offset=%u\n",
            (unsigned int)motor_encoder_align_get_offset_raw(
                &motor_encoder_align));
  }

  if (MOTOR_POSITION_STEP_MODE != 0U) {
    if (motor_encoder_drive_start(&motor_encoder_drive) != HAL_OK) {
      Error_Handler();
    }
    motor_encoder_drive_set_mode(&motor_encoder_drive,
                                 MOTOR_ENCODER_DRIVE_MODE_POSITION);
    if (motor_encoder_drive_set_position_target_mdeg(&motor_encoder_drive,
                                                     MOTOR_POSITION_START_DEG *
                                                         1000) !=
        HAL_OK) {
      Error_Handler();
    }
    RTT_Log("[POS] absolute step mode start=%ddeg step=%ddeg settle=%dmdeg\n",
            MOTOR_POSITION_START_DEG,
            MOTOR_POSITION_STEP_DEG,
            MOTOR_POSITION_STEP_SETTLE_MDEG);
  }
  if (motor_can_init(&hfdcan1) != HAL_OK) {
    RTT_Log("[CAN] init failed\n");
    Error_Handler();
  }
  RTT_Log("[CAN] FD ready cmd=0x%03X status=0x%03X\n",
          MOTOR_CAN_CMD_ID,
          MOTOR_CAN_STATUS_ID);

  motor_control_adc_tick_pending = 0U;
  motor_control_adc_tick_overrun = 0U;
  motor_control_adc_sync_enabled = 1U;

  HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_RESET);
  RTT_Log("[MOTOR] waiting for CAN command, PWM disabled\n");
  RTT_Log("[ANGLE] offset=%u\n",
          (unsigned int)motor_encoder_align_get_offset_raw(
              &motor_encoder_align));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    static uint32_t last_log_ms = 0;
    static uint32_t last_control_adc_tick_count = 0U;
    static uint32_t position_step_settle_ms = 0U;
    static int32_t position_target_mdeg = MOTOR_POSITION_START_DEG * 1000;
    static uint8_t position_step_initialized = 0U;
    motor_foc_abc_t phase_current_foc = {0.0f, 0.0f, 0.0f};
    uint8_t phase_current_valid = 0U;
    uint32_t adc_tick_count = 0U;
    uint32_t adc_tick_overrun = 0U;
    uint32_t adc_tick_delta = 1U;
    float control_dt_s;

    if (motor_control_adc_tick_pending == 0U) {
      continue;
    }

    __disable_irq();
    motor_control_adc_tick_pending = 0U;
    adc_tick_count = motor_control_adc_tick_count;
    adc_tick_overrun = motor_control_adc_tick_overrun;
    __enable_irq();

    if (last_control_adc_tick_count != 0U) {
      adc_tick_delta = adc_tick_count - last_control_adc_tick_count;
      if (adc_tick_delta == 0U) {
        adc_tick_delta = 1U;
      }
    }
    last_control_adc_tick_count = adc_tick_count;
    control_dt_s = (float)adc_tick_delta * MOTOR_CONTROL_ADC_TICK_S;

    if (drv8323rs_current_read_amps(&drv8323rs_current,
                                    &phase_current_amps) == HAL_OK) {
      phase_current_foc.a = phase_current_amps.a;
      phase_current_foc.b = phase_current_amps.b;
      phase_current_foc.c = phase_current_amps.c;
      phase_current_valid = 1U;
    }

    motor_can_process(&motor_encoder_drive);
    if (motor_encoder_drive.started != 0U) {
      (void)motor_encoder_drive_update_with_current_dt(
          &motor_encoder_drive,
          (phase_current_valid != 0U) ? &phase_current_foc : NULL,
          control_dt_s);
    }

    if ((MOTOR_POSITION_STEP_MODE != 0U) && (motor_can_is_active() == 0U)) {
      int32_t position_mdeg =
          motor_encoder_drive_get_position_mdeg(&motor_encoder_drive);
      int32_t position_error_mdeg;
      uint32_t now_ms = HAL_GetTick();

      if (position_step_initialized == 0U) {
        position_target_mdeg = RoundToNearestRevolutionMdeg(position_mdeg);
        (void)motor_encoder_drive_set_position_target_mdeg(
            &motor_encoder_drive, position_target_mdeg);
        position_step_initialized = 1U;
        RTT_Log("[POS] align absolute zero target=%ldmdeg current=%ldmdeg\n",
                (long)position_target_mdeg,
                (long)position_mdeg);
      }

      position_error_mdeg = position_target_mdeg - position_mdeg;
      if (position_error_mdeg < 0) {
        position_error_mdeg = -position_error_mdeg;
      }

      if (position_error_mdeg <= MOTOR_POSITION_STEP_SETTLE_MDEG) {
        if (position_step_settle_ms == 0U) {
          position_step_settle_ms = now_ms;
        } else if ((now_ms - position_step_settle_ms) >=
                   MOTOR_POSITION_STEP_HOLD_MS) {
          position_target_mdeg += MOTOR_POSITION_STEP_DEG * 1000;
          position_step_settle_ms = 0U;
          (void)motor_encoder_drive_set_position_target_mdeg(
              &motor_encoder_drive, position_target_mdeg);
          RTT_Log("[POS] step target=%ldmdeg current=%ldmdeg err=%ldmdeg\n",
                  (long)position_target_mdeg,
                  (long)position_mdeg,
                  (long)position_error_mdeg);
        }
      } else if (position_error_mdeg > MOTOR_POSITION_STEP_RESET_MDEG) {
        position_step_settle_ms = 0U;
      }
    }

    if ((HAL_GetTick() - last_log_ms) >= 2000U) {
      int32_t ia_ma = 0;
      int32_t ib_ma = 0;
      int32_t ic_ma = 0;
      drv8323rs_current_raw_t current_raw = {0U, 0U, 0U};
      motor_foc_dq_t current_dq =
          motor_encoder_drive_get_current_dq(&motor_encoder_drive);
      int32_t id_ma = AmpsToMilliamps(current_dq.d);
      int32_t iq_ma = AmpsToMilliamps(current_dq.q);
      int32_t iq_ref_ma = AmpsToMilliamps(
          motor_encoder_drive_get_current_q_ref(&motor_encoder_drive));
      int32_t position_mdeg =
          motor_encoder_drive_get_position_mdeg(&motor_encoder_drive);
      int32_t position_target_log_mdeg =
          motor_encoder_drive_get_position_target_mdeg(&motor_encoder_drive);
      encoder_electrical_mdeg =
          motor_encoder_drive_get_electrical_mdeg(&motor_encoder_drive);

      last_log_ms = HAL_GetTick();
      if (phase_current_valid != 0U) {
        ia_ma = AmpsToMilliamps(phase_current_amps.a);
        ib_ma = AmpsToMilliamps(phase_current_amps.b);
        ic_ma = AmpsToMilliamps(phase_current_amps.c);
      }
      (void)drv8323rs_current_read_raw(&drv8323rs_current, &current_raw);

      RTT_Log("[FOC] t=%lu ov=%lu pos=%ld/%ld ele=%u.%03u spd=%.2f vq=%.2f "
              "adc=%u,%u,%u iq=%ld/%ld id=%ld ia=%ld,%ld,%ld\n",
              (unsigned long)adc_tick_count,
              (unsigned long)adc_tick_overrun,
              (long)position_mdeg,
              (long)position_target_log_mdeg,
              (unsigned int)(encoder_electrical_mdeg / 1000U),
              (unsigned int)(encoder_electrical_mdeg % 1000U),
              (double)motor_encoder_drive_get_measured_speed_hz(
                  &motor_encoder_drive),
              (double)motor_encoder_drive_get_voltage_q(&motor_encoder_drive),
              (unsigned int)current_raw.a,
              (unsigned int)current_raw.b,
              (unsigned int)current_raw.c,
              (long)iq_ref_ma,
              (long)iq_ma,
              (long)id_ma,
              (long)ia_ma,
              (long)ib_ma,
              (long)ic_ma);
    }

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
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if ((hadc != NULL) && (hadc->Instance == ADC1) &&
      (motor_control_adc_sync_enabled != 0U)) {
    if (motor_control_adc_tick_pending != 0U) {
      motor_control_adc_tick_overrun++;
    }
    motor_control_adc_tick_pending = 1U;
    motor_control_adc_tick_count++;
  }
}

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
