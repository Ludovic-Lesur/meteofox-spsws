/*
 * max11136.c
 *
 *  Created on: 01 sep. 2024
 *      Author: Ludo
 */

#include "sx1232.h"

#ifndef SX1232_DRIVER_DISABLE_FLAGS_FILE
#include "sx1232_driver_flags.h"
#endif
#include "error.h"
#include "error_base.h"
#include "gpio.h"
#include "lptim.h"
#include "mcu_mapping.h"
#include "spi.h"
#include "sx1232.h"
#include "types.h"

#ifndef SX1232_DRIVER_DISABLE

/*** SX1232 HW functions ***/

/*******************************************************************/
SX1232_status_t SX1232_HW_init(void) {
    // Local variables.
    SX1232_status_t status = SX1232_SUCCESS;
    SPI_status_t spi_status = SPI_SUCCESS;
    SPI_configuration_t spi_config;
    // Init SPI.
    spi_config.baud_rate_prescaler = SPI_BAUD_RATE_PRESCALER_4;
    spi_config.data_format = SPI_DATA_FORMAT_16_BITS;
    spi_config.clock_polarity = SPI_CLOCK_POLARITY_LOW;
    spi_status = SPI_init(SPI_INSTANCE_RADIO, &SPI_GPIO_SX1232, &spi_config);
    SPI_exit_error(SX1232_ERROR_BASE_SPI);
    // Configure chip select pin.
    GPIO_configure(&GPIO_SX1232_CS, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_write(&GPIO_SX1232_CS, 1);
errors:
    return status;
}

/*******************************************************************/
SX1232_status_t SX1232_HW_de_init(void) {
    // Local variables.
    SX1232_status_t status = SX1232_SUCCESS;
    SPI_status_t spi_status = SPI_SUCCESS;
    // Release chip select pin.
    GPIO_write(&GPIO_SX1232_CS, 0);
    // Release SPI.
    spi_status = SPI_de_init(SPI_INSTANCE_RADIO, &SPI_GPIO_SX1232);
    SPI_stack_error(ERROR_BASE_SX1232 + SX1232_ERROR_BASE_SPI);
    return status;
}

/*******************************************************************/
SX1232_status_t SX1232_HW_spi_write_read_16(uint16_t* tx_data, uint16_t* rx_data, uint8_t transfer_size) {
    // Local variables.
    SX1232_status_t status = SX1232_SUCCESS;
    SPI_status_t spi_status = SPI_SUCCESS;
    // CS low.
    GPIO_write(&GPIO_SX1232_CS, 0);
    // SPI transfer.
    spi_status = SPI_write_read_16(SPI_INSTANCE_RADIO, tx_data, rx_data, transfer_size);
    SPI_exit_error(SX1232_ERROR_BASE_SPI);
errors:
    // CS high.
    GPIO_write(&GPIO_SX1232_CS, 1);
    return status;
}

/*******************************************************************/
void SX1232_HW_spi_write_16(uint16_t tx_data) {
    GPIO_SX1232_CS_LOW();
    SPI_write_16(SPI_INSTANCE_RADIO, tx_data);
    GPIO_SX1232_CS_HIGH();
}

/*******************************************************************/
SX1232_status_t SX1232_HW_delay_milliseconds(uint32_t delay_ms) {
    // Local variables.
    SX1232_status_t status = SX1232_SUCCESS;
    LPTIM_status_t lptim_status = LPTIM_SUCCESS;
    // Perform delay.
    lptim_status = LPTIM_delay_milliseconds(delay_ms, LPTIM_DELAY_MODE_SLEEP);
    LPTIM_exit_error(SX1232_ERROR_BASE_DELAY);
errors:
    return status;
}

#endif /* SX1232_DRIVER_DISABLE */
