#include "drv8323rs_current.h"

#define DRV8323RS_CURRENT_ADC_MAX_COUNTS 4095.0f
#define DRV8323RS_CURRENT_PHASE_COUNT 3U

HAL_StatusTypeDef drv8323rs_current_init(
    drv8323rs_current_t *sense,
    const drv8323rs_current_config_t *config) {
    if ((sense == NULL) || (config == NULL) || (config->hadc == NULL) ||
        (config->dma_buffer == NULL) ||
        (config->dma_buffer_length < DRV8323RS_CURRENT_PHASE_COUNT) ||
        (config->vref_mv == 0U) || (config->shunt_ohm <= 0.0f) ||
        (config->csa_gain_vv <= 0.0f)) {
        return HAL_ERROR;
    }

    sense->hadc = config->hadc;
    sense->vref_mv = config->vref_mv;
    sense->shunt_ohm = config->shunt_ohm;
    sense->csa_gain_vv = config->csa_gain_vv;
    sense->calibration_samples =
        (config->calibration_samples == 0U) ? 256U : config->calibration_samples;
    sense->dma_buffer = config->dma_buffer;
    sense->dma_buffer_length = config->dma_buffer_length;
    sense->offset_raw.a = 2048U;
    sense->offset_raw.b = 2048U;
    sense->offset_raw.c = 2048U;

    return HAL_ADCEx_Calibration_Start(sense->hadc, ADC_SINGLE_ENDED);
}

HAL_StatusTypeDef drv8323rs_current_start_dma(drv8323rs_current_t *sense) {
    if ((sense == NULL) || (sense->hadc == NULL) ||
        (sense->dma_buffer == NULL)) {
        return HAL_ERROR;
    }

    return HAL_ADC_Start_DMA(sense->hadc, (uint32_t *)sense->dma_buffer,
                             sense->dma_buffer_length);
}

HAL_StatusTypeDef drv8323rs_current_read_raw(
    drv8323rs_current_t *sense,
    drv8323rs_current_raw_t *raw) {
    if ((sense == NULL) || (raw == NULL) || (sense->dma_buffer == NULL) ||
        (sense->dma_buffer_length < DRV8323RS_CURRENT_PHASE_COUNT)) {
        return HAL_ERROR;
    }

    raw->a = sense->dma_buffer[0];
    raw->b = sense->dma_buffer[1];
    raw->c = sense->dma_buffer[2];

    return HAL_OK;
}

HAL_StatusTypeDef drv8323rs_current_get_offsets(
    drv8323rs_current_t *sense,
    drv8323rs_current_raw_t *offsets) {
    if ((sense == NULL) || (offsets == NULL)) {
        return HAL_ERROR;
    }

    *offsets = sense->offset_raw;
    return HAL_OK;
}

HAL_StatusTypeDef drv8323rs_current_calibrate_offsets(
    drv8323rs_current_t *sense) {
    uint32_t sum_a = 0U;
    uint32_t sum_b = 0U;
    uint32_t sum_c = 0U;

    if (sense == NULL) {
        return HAL_ERROR;
    }

    for (uint32_t i = 0U; i < sense->calibration_samples; i++) {
        drv8323rs_current_raw_t raw;

        HAL_Delay(1U);
        if (drv8323rs_current_read_raw(sense, &raw) != HAL_OK) {
            return HAL_ERROR;
        }

        sum_a += raw.a;
        sum_b += raw.b;
        sum_c += raw.c;
    }

    sense->offset_raw.a = (uint16_t)(sum_a / sense->calibration_samples);
    sense->offset_raw.b = (uint16_t)(sum_b / sense->calibration_samples);
    sense->offset_raw.c = (uint16_t)(sum_c / sense->calibration_samples);

    return HAL_OK;
}

static float drv8323rs_current_raw_to_amps(
    const drv8323rs_current_t *sense,
    uint16_t raw,
    uint16_t offset_raw) {
    const float adc_mv_per_count =
        (float)sense->vref_mv / DRV8323RS_CURRENT_ADC_MAX_COUNTS;
    const float sense_mv =
        ((float)((int32_t)raw - (int32_t)offset_raw)) * adc_mv_per_count;

    return (sense_mv * 0.001f) / (sense->shunt_ohm * sense->csa_gain_vv);
}

HAL_StatusTypeDef drv8323rs_current_read_amps(
    drv8323rs_current_t *sense,
    drv8323rs_current_amps_t *amps) {
    drv8323rs_current_raw_t raw;

    if ((sense == NULL) || (amps == NULL)) {
        return HAL_ERROR;
    }

    if (drv8323rs_current_read_raw(sense, &raw) != HAL_OK) {
        return HAL_ERROR;
    }

    amps->a = drv8323rs_current_raw_to_amps(sense, raw.a, sense->offset_raw.a);
    amps->b = drv8323rs_current_raw_to_amps(sense, raw.b, sense->offset_raw.b);
    amps->c = drv8323rs_current_raw_to_amps(sense, raw.c, sense->offset_raw.c);

    return HAL_OK;
}
