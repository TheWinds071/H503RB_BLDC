#include "as5047p_port.h"
#include "spi.h"
#include "main.h"

void as5047p_spi_send(uint16_t data) {
    uint16_t rx_dummy;
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&data, (uint8_t*)&rx_dummy, 1, 10);
}

uint16_t as5047p_spi_read(void) {
    uint16_t rx_data = 0;
    uint16_t tx_data = 0xC000; // AS5047P read NOP command
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&tx_data, (uint8_t*)&rx_data, 1, 10);
    return rx_data;
}

void as5047p_spi_select(void) {
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
}

void as5047p_spi_deselect(void) {
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}

void as5047p_delay(void) {
    for(volatile int i = 0; i < 100; i++);
}

void as5047p_port_init(as5047p_handle_t *handle) {
    as5047p_make_handle(as5047p_spi_send, as5047p_spi_read, as5047p_spi_select, as5047p_spi_deselect, as5047p_delay, handle);
}