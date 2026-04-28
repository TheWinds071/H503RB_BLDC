#ifndef DRV8323RS_CURRENT_H
#define DRV8323RS_CURRENT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h5xx_hal.h"

typedef struct {
    ADC_HandleTypeDef *hadc;
    uint32_t vref_mv;
    float shunt_ohm;
    float csa_gain_vv;
    uint32_t calibration_samples;
    volatile uint16_t *dma_buffer;
    uint32_t dma_buffer_length;
} drv8323rs_current_config_t;

typedef struct {
    uint16_t a;
    uint16_t b;
    uint16_t c;
} drv8323rs_current_raw_t;

typedef struct {
    float a;
    float b;
    float c;
} drv8323rs_current_amps_t;

typedef struct {
    ADC_HandleTypeDef *hadc;
    uint32_t vref_mv;
    float shunt_ohm;
    float csa_gain_vv;
    uint32_t calibration_samples;
    volatile uint16_t *dma_buffer;
    uint32_t dma_buffer_length;
    drv8323rs_current_raw_t offset_raw;
} drv8323rs_current_t;

HAL_StatusTypeDef drv8323rs_current_init(
    drv8323rs_current_t *sense,
    const drv8323rs_current_config_t *config);
HAL_StatusTypeDef drv8323rs_current_calibrate_offsets(
    drv8323rs_current_t *sense);
HAL_StatusTypeDef drv8323rs_current_start_dma(drv8323rs_current_t *sense);
HAL_StatusTypeDef drv8323rs_current_read_raw(
    drv8323rs_current_t *sense,
    drv8323rs_current_raw_t *raw);
HAL_StatusTypeDef drv8323rs_current_read_amps(
    drv8323rs_current_t *sense,
    drv8323rs_current_amps_t *amps);

#ifdef __cplusplus
}
#endif

#endif
