#ifndef DRV8323RS_H
#define DRV8323RS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "spi.h"

typedef enum {
    DRV8323RS_CSA_GAIN_5VV = 0U,
    DRV8323RS_CSA_GAIN_10VV = 1U,
    DRV8323RS_CSA_GAIN_20VV = 2U,
    DRV8323RS_CSA_GAIN_40VV = 3U
} drv8323rs_csa_gain_t;

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    GPIO_TypeDef *enable_port;
    uint16_t enable_pin;
    drv8323rs_csa_gain_t csa_gain;
    uint8_t sense_ocp_enable;
} drv8323rs_config_t;

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    GPIO_TypeDef *enable_port;
    uint16_t enable_pin;
    drv8323rs_csa_gain_t csa_gain;
    uint8_t ready;
    uint8_t last_error_step;
    HAL_StatusTypeDef last_hal_status;
    uint16_t last_readback;
} drv8323rs_t;

typedef struct {
    uint16_t fault_status_1;
    uint16_t vgs_status_2;
} drv8323rs_faults_t;

HAL_StatusTypeDef drv8323rs_init(drv8323rs_t *drv,
                                 const drv8323rs_config_t *config);
HAL_StatusTypeDef drv8323rs_read_reg(drv8323rs_t *drv, uint8_t reg_addr,
                                     uint16_t *reg_value);
HAL_StatusTypeDef drv8323rs_write_reg(drv8323rs_t *drv, uint8_t reg_addr,
                                      uint16_t reg_value);
HAL_StatusTypeDef drv8323rs_get_faults(drv8323rs_t *drv,
                                       drv8323rs_faults_t *faults);
HAL_StatusTypeDef drv8323rs_dump_registers(drv8323rs_t *drv, uint16_t *regs,
                                           uint8_t reg_count);

#ifdef __cplusplus
}
#endif

#endif
