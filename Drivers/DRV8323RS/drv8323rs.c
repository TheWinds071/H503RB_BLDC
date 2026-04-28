#include "drv8323rs.h"

#define DRV8323RS_SPI_WORD_READ_MASK 0x8000U
#define DRV8323RS_SPI_WORD_ADDR_SHIFT 11U
#define DRV8323RS_SPI_WORD_DATA_MASK 0x07FFU

#define DRV8323RS_REG_FAULT_STATUS_1 0x00U
#define DRV8323RS_REG_VGS_STATUS_2   0x01U
#define DRV8323RS_REG_DRIVER_CONTROL 0x02U
#define DRV8323RS_REG_GATE_DRIVE_HS  0x03U
#define DRV8323RS_REG_GATE_DRIVE_LS  0x04U
#define DRV8323RS_REG_OCP_CONTROL    0x05U
#define DRV8323RS_REG_CSA_CONTROL    0x06U

#define DRV8323RS_DRIVER_CTRL_CLR_FLT   (1U << 0)
#define DRV8323RS_DRIVER_CTRL_PWM_6X    (0U << 5)

#define DRV8323RS_GATE_DRIVE_LOCK_UNLOCK (3U << 8)
#define DRV8323RS_IDRIVEP_120MA          (4U << 4)
#define DRV8323RS_IDRIVEN_120MA          (2U << 0)

#define DRV8323RS_CBC_ENABLE      (1U << 10)
#define DRV8323RS_TDRIVE_1000NS   (1U << 8)

#define DRV8323RS_OCP_RETRY_4MS   (0U << 10)
#define DRV8323RS_OCP_DEADTIME_100NS (1U << 8)
#define DRV8323RS_OCP_MODE_RETRY  (1U << 6)
#define DRV8323RS_OCP_DEG_4US     (1U << 4)
#define DRV8323RS_VDS_LVL_1P13V   (11U << 0)

#define DRV8323RS_CSA_VREF_DIV    (1U << 9)
#define DRV8323RS_CSA_GAIN_SHIFT  6U
#define DRV8323RS_CSA_DIS_SEN     (1U << 5)

static void drv8323rs_spi_delay(void) {
    volatile uint32_t delay_cycles;

    for (delay_cycles = 0U; delay_cycles < 64U; delay_cycles++) {
        __NOP();
    }
}

static uint16_t drv8323rs_build_word(uint8_t is_read, uint8_t reg_addr,
                                     uint16_t reg_value) {
    uint16_t word = 0U;

    if (is_read != 0U) {
        word |= DRV8323RS_SPI_WORD_READ_MASK;
    }

    word |= ((uint16_t)(reg_addr & 0x0FU) << DRV8323RS_SPI_WORD_ADDR_SHIFT);
    word |= (reg_value & DRV8323RS_SPI_WORD_DATA_MASK);

    return word;
}

static HAL_StatusTypeDef drv8323rs_spi_reconfigure(SPI_HandleTypeDef *hspi) {
    if (hspi == NULL) {
        return HAL_ERROR;
    }

    if (HAL_SPI_DeInit(hspi) != HAL_OK) {
        return HAL_ERROR;
    }

    hspi->Init.Mode = SPI_MODE_MASTER;
    hspi->Init.Direction = SPI_DIRECTION_2LINES;
    hspi->Init.DataSize = SPI_DATASIZE_16BIT;
    hspi->Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi->Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi->Init.NSS = SPI_NSS_SOFT;
    hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
    hspi->Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi->Init.TIMode = SPI_TIMODE_DISABLE;
    hspi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi->Init.CRCPolynomial = 0x7;
    hspi->Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    hspi->Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
    hspi->Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
    hspi->Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
    hspi->Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
    hspi->Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    hspi->Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
    hspi->Init.IOSwap = SPI_IO_SWAP_DISABLE;
    hspi->Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
    hspi->Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;

    return HAL_SPI_Init(hspi);
}

static HAL_StatusTypeDef drv8323rs_transfer_word(drv8323rs_t *drv,
                                                 uint16_t tx_word,
                                                 uint16_t *rx_word) {
    HAL_StatusTypeDef status;
    uint16_t rx_tmp = 0U;

    if ((drv == NULL) || (drv->hspi == NULL)) {
        return HAL_ERROR;
    }

    HAL_GPIO_WritePin(drv->cs_port, drv->cs_pin, GPIO_PIN_RESET);
    drv8323rs_spi_delay();
    status = HAL_SPI_TransmitReceive(drv->hspi, (uint8_t *)&tx_word,
                                     (uint8_t *)&rx_tmp, 1U, 100U);
    drv8323rs_spi_delay();
    HAL_GPIO_WritePin(drv->cs_port, drv->cs_pin, GPIO_PIN_SET);
    drv8323rs_spi_delay();

    if ((status == HAL_OK) && (rx_word != NULL)) {
        *rx_word = rx_tmp;
    }

    return status;
}

HAL_StatusTypeDef drv8323rs_read_reg(drv8323rs_t *drv, uint8_t reg_addr,
                                     uint16_t *reg_value) {
    HAL_StatusTypeDef status;
    uint16_t rx_word = 0U;

    if (reg_value == NULL) {
        return HAL_ERROR;
    }

    status = drv8323rs_transfer_word(drv, drv8323rs_build_word(1U, reg_addr, 0U),
                                     &rx_word);
    if (status != HAL_OK) {
        return status;
    }

    *reg_value = (rx_word & DRV8323RS_SPI_WORD_DATA_MASK);
    return HAL_OK;
}

HAL_StatusTypeDef drv8323rs_write_reg(drv8323rs_t *drv, uint8_t reg_addr,
                                      uint16_t reg_value) {
    return drv8323rs_transfer_word(drv,
                                   drv8323rs_build_word(0U, reg_addr, reg_value),
                                   NULL);
}

HAL_StatusTypeDef drv8323rs_get_faults(drv8323rs_t *drv,
                                       drv8323rs_faults_t *faults) {
    HAL_StatusTypeDef status;

    if (faults == NULL) {
        return HAL_ERROR;
    }

    status = drv8323rs_read_reg(drv, DRV8323RS_REG_FAULT_STATUS_1,
                                &faults->fault_status_1);
    if (status != HAL_OK) {
        return status;
    }

    return drv8323rs_read_reg(drv, DRV8323RS_REG_VGS_STATUS_2,
                              &faults->vgs_status_2);
}

HAL_StatusTypeDef drv8323rs_dump_registers(drv8323rs_t *drv, uint16_t *regs,
                                           uint8_t reg_count) {
    uint8_t reg_index;

    if ((drv == NULL) || (regs == NULL) || (reg_count == 0U)) {
        return HAL_ERROR;
    }

    for (reg_index = 0U; reg_index < reg_count; reg_index++) {
        HAL_StatusTypeDef status =
            drv8323rs_read_reg(drv, reg_index, &regs[reg_index]);
        if (status != HAL_OK) {
            return status;
        }
    }

    return HAL_OK;
}

HAL_StatusTypeDef drv8323rs_init(drv8323rs_t *drv,
                                 const drv8323rs_config_t *config) {
    HAL_StatusTypeDef status;
    uint16_t readback = 0U;
    uint16_t ocp_readback = 0U;
    uint16_t csa_readback = 0U;
    const uint16_t driver_control_clear_fault =
        DRV8323RS_DRIVER_CTRL_CLR_FLT | DRV8323RS_DRIVER_CTRL_PWM_6X;
    const uint16_t driver_control_run = DRV8323RS_DRIVER_CTRL_PWM_6X;
    const uint16_t gate_drive_hs =
        DRV8323RS_GATE_DRIVE_LOCK_UNLOCK |
        DRV8323RS_IDRIVEP_120MA |
        DRV8323RS_IDRIVEN_120MA;
    const uint16_t gate_drive_ls =
        DRV8323RS_CBC_ENABLE |
        DRV8323RS_TDRIVE_1000NS |
        DRV8323RS_IDRIVEP_120MA |
        DRV8323RS_IDRIVEN_120MA;
    const uint16_t ocp_control =
        DRV8323RS_OCP_RETRY_4MS |
        DRV8323RS_OCP_DEADTIME_100NS |
        DRV8323RS_OCP_MODE_RETRY |
        DRV8323RS_OCP_DEG_4US |
        DRV8323RS_VDS_LVL_1P13V;
    uint16_t csa_control =
        DRV8323RS_CSA_VREF_DIV |
        ((uint16_t)(config->csa_gain & 0x03U) << DRV8323RS_CSA_GAIN_SHIFT);

    if (config->sense_ocp_enable == 0U) {
        csa_control |= DRV8323RS_CSA_DIS_SEN;
    }

    if ((drv == NULL) || (config == NULL) || (config->hspi == NULL)) {
        return HAL_ERROR;
    }

    drv->hspi = config->hspi;
    drv->cs_port = config->cs_port;
    drv->cs_pin = config->cs_pin;
    drv->enable_port = config->enable_port;
    drv->enable_pin = config->enable_pin;
    drv->csa_gain = config->csa_gain;
    drv->ready = 0U;
    drv->last_error_step = 0U;
    drv->last_hal_status = HAL_OK;
    drv->last_readback = 0U;

    HAL_GPIO_WritePin(drv->cs_port, drv->cs_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(drv->enable_port, drv->enable_pin, GPIO_PIN_RESET);
    HAL_Delay(1U);

    drv->last_error_step = 1U;
    status = drv8323rs_spi_reconfigure(drv->hspi);
    if (status != HAL_OK) {
        drv->last_hal_status = status;
        return status;
    }

    HAL_GPIO_WritePin(drv->enable_port, drv->enable_pin, GPIO_PIN_SET);
    HAL_Delay(5U);

    drv->last_error_step = 2U;
    status = drv8323rs_write_reg(drv, DRV8323RS_REG_DRIVER_CONTROL,
                                 driver_control_clear_fault);
    if (status != HAL_OK) {
        drv->last_hal_status = status;
        return status;
    }

    drv->last_error_step = 3U;
    status = drv8323rs_write_reg(drv, DRV8323RS_REG_DRIVER_CONTROL,
                                 driver_control_run);
    if (status != HAL_OK) {
        drv->last_hal_status = status;
        return status;
    }

    drv->last_error_step = 4U;
    status = drv8323rs_write_reg(drv, DRV8323RS_REG_GATE_DRIVE_HS,
                                 gate_drive_hs);
    if (status != HAL_OK) {
        drv->last_hal_status = status;
        return status;
    }

    drv->last_error_step = 5U;
    status = drv8323rs_write_reg(drv, DRV8323RS_REG_GATE_DRIVE_LS,
                                 gate_drive_ls);
    if (status != HAL_OK) {
        drv->last_hal_status = status;
        return status;
    }

    drv->last_error_step = 6U;
    status = drv8323rs_write_reg(drv, DRV8323RS_REG_OCP_CONTROL,
                                 ocp_control);
    if (status != HAL_OK) {
        drv->last_hal_status = status;
        return status;
    }

    drv->last_error_step = 7U;
    status = drv8323rs_write_reg(drv, DRV8323RS_REG_CSA_CONTROL,
                                 csa_control);
    if (status != HAL_OK) {
        drv->last_hal_status = status;
        return status;
    }

    drv->last_error_step = 8U;
    status = drv8323rs_read_reg(drv, DRV8323RS_REG_GATE_DRIVE_HS, &readback);
    if (status != HAL_OK) {
        drv->last_hal_status = status;
        return status;
    }

    drv->last_readback = readback;
    status = drv8323rs_read_reg(drv, DRV8323RS_REG_OCP_CONTROL, &ocp_readback);
    if (status != HAL_OK) {
        drv->last_hal_status = status;
        return status;
    }

    status = drv8323rs_read_reg(drv, DRV8323RS_REG_CSA_CONTROL, &csa_readback);
    if (status != HAL_OK) {
        drv->last_hal_status = status;
        return status;
    }

    if (((readback | ocp_readback | csa_readback) & DRV8323RS_SPI_WORD_DATA_MASK) == 0U) {
        drv->last_hal_status = HAL_ERROR;
        return HAL_ERROR;
    }

    drv->ready = 1U;
    return HAL_OK;
}
