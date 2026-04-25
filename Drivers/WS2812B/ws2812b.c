#include "ws2812b.h"

/* The RESET pulse needs to be >50us. At 800kHz (1.25us/bit), 50us is 40 bits of 0-duty cycle. We use 60. */
#define WS2812B_RESET_PULSE 60
#define WS2812B_BUFFER_SIZE (24 * WS2812B_NUM_LEDS + WS2812B_RESET_PULSE)

/*
 * Timer ARR is 311 (meaning 312 ticks per period).
 * For 800kHz WS2812B protocol:
 * 0 bit: ~0.4us high, ~0.85us low  -> ~32% duty cycle -> 312 * 0.32 = ~100
 * 1 bit: ~0.8us high, ~0.45us low  -> ~68% duty cycle -> 312 * 0.68 = ~212
 */
#define WS2812B_BIT_0 100
#define WS2812B_BIT_1 212

static uint32_t ws2812b_buffer[WS2812B_BUFFER_SIZE];
static TIM_HandleTypeDef *ws2812b_htim;
static uint32_t ws2812b_channel;
static volatile uint8_t ws2812b_transfer_complete = 1;

static uint8_t WS2812B_WaitReady(uint32_t timeout_ms) {
    const uint32_t start_tick = HAL_GetTick();

    while (!WS2812B_IsReady()) {
        if ((HAL_GetTick() - start_tick) >= timeout_ms) {
            return 0U;
        }
    }

    return 1U;
}

static uint8_t WS2812B_ShowAndWait(uint8_t r, uint8_t g, uint8_t b,
                                   uint32_t hold_time_ms) {
    if (!WS2812B_WaitReady(20U)) {
        return 0U;
    }

    WS2812B_SetColor(0, r, g, b);
    WS2812B_Update();

    if (!WS2812B_WaitReady(20U)) {
        return 0U;
    }

    HAL_Delay(hold_time_ms);
    return 1U;
}

void WS2812B_Init(TIM_HandleTypeDef *htim, uint32_t channel) {
    ws2812b_htim = htim;
    ws2812b_channel = channel;
    
    for (int i = 0; i < WS2812B_BUFFER_SIZE; i++) {
        ws2812b_buffer[i] = 0;
    }
}

void WS2812B_SetColor(uint16_t led_index, uint8_t r, uint8_t g, uint8_t b) {
    if (led_index >= WS2812B_NUM_LEDS) return;
    
    /* WS2812B color order is GRB */
    uint32_t color = ((uint32_t)g << 16) | ((uint32_t)r << 8) | b;
    uint32_t start_index = led_index * 24;
    
    for (int i = 23; i >= 0; i--) {
        if (color & (1 << i)) {
            ws2812b_buffer[start_index + (23 - i)] = WS2812B_BIT_1;
        } else {
            ws2812b_buffer[start_index + (23 - i)] = WS2812B_BIT_0;
        }
    }
}

void WS2812B_Update(void) {
    if (!ws2812b_transfer_complete) return;
    ws2812b_transfer_complete = 0;
    HAL_TIM_PWM_Start_DMA(ws2812b_htim, ws2812b_channel, ws2812b_buffer, WS2812B_BUFFER_SIZE);
}

uint8_t WS2812B_IsReady(void) {
    return ws2812b_transfer_complete;
}

void WS2812B_PowerOnSelfTest(uint8_t brightness, uint32_t on_time_ms) {
    if (!WS2812B_ShowAndWait(brightness, 0, 0, on_time_ms)) {
        return;
    }
    if (!WS2812B_ShowAndWait(0, 0, 0, 80U)) {
        return;
    }
    if (!WS2812B_ShowAndWait(0, brightness, 0, on_time_ms)) {
        return;
    }
    if (!WS2812B_ShowAndWait(0, 0, 0, 80U)) {
        return;
    }
    if (!WS2812B_ShowAndWait(0, 0, brightness, on_time_ms)) {
        return;
    }
    (void)WS2812B_ShowAndWait(0, 0, 0, 0U);
}

void WS2812B_TransferCompleteCallback(TIM_HandleTypeDef *htim) {
    if (htim == ws2812b_htim) {
        HAL_TIM_PWM_Stop_DMA(ws2812b_htim, ws2812b_channel);
        ws2812b_transfer_complete = 1;
    }
}
