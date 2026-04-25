#ifndef WS2812B_H
#define WS2812B_H

#include "stm32h5xx_hal.h"

#define WS2812B_NUM_LEDS 1

void WS2812B_Init(TIM_HandleTypeDef *htim, uint32_t channel);
void WS2812B_SetColor(uint16_t led_index, uint8_t r, uint8_t g, uint8_t b);
void WS2812B_Update(void);
uint8_t WS2812B_IsReady(void);
void WS2812B_TransferCompleteCallback(TIM_HandleTypeDef *htim);

#endif /* WS2812B_H */