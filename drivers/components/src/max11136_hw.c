/*
 * max11136.c
 *
 *  Created on: 01 sep. 2024
 *      Author: Ludo
 */

#include "max111xx.h"

#ifndef MAX111XX_DRIVER_DISABLE_FLAGS_FILE
#include "max111xx_driver_flags.h"
#endif
#include "error.h"
#include "error_base.h"
#include "gpio.h"
#include "lptim.h"
#include "max111xx.h"
#include "mcu_mapping.h"
#include "spi.h"
#include "types.h"

#ifndef MAX111XX_DRIVER_DISABLE

/*** MAX111XX HW functions ***/

/*******************************************************************/
MAX111XX_status_t MAX111XX_HW_init(void) {
    // Local variables.
    MAX111XX_status_t status = MAX111XX_SUCCESS;
    SPI_status_t spi_status = SPI_SUCCESS;
    SPI_configuration_t spi_config;
    // Init SPI.
    spi_config.baud_rate_prescaler = SPI_BAUD_RATE_PRESCALER_4;
    spi_config.data_format = SPI_DATA_FORMAT_16_BITS;
    spi_config.clock_polarity = SPI_CLOCK_POLARITY_HIGH;
    spi_status = SPI_init(SPI_INSTANCE_ADC, &SPI_GPIO_MAX11136, &spi_config);
    SPI_exit_error(MAX111XX_ERROR_BASE_SPI);
    // Configure chip select pin.
    GPIO_configure(&GPIO_MAX11136_CS, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_write(&GPIO_MAX11136_CS, 1);
    // Configure EOC GPIO.
    GPIO_configure(&GPIO_MAX11136_EOC, GPIO_MODE_INPUT, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_UP);
errors:
    return status;
}

/*******************************************************************/
MAX111XX_status_t MAX111XX_HW_de_init(void) {
    // Local variables.
    MAX111XX_status_t status = MAX111XX_SUCCESS;
    SPI_status_t spi_status = SPI_SUCCESS;
    // Release chip select pin.
    GPIO_write(&GPIO_MAX11136_CS, 0);
    // Release EOC GPIO.
    GPIO_configure(&GPIO_MAX11136_EOC, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    // Release SPI.
    spi_status = SPI_de_init(SPI_INSTANCE_ADC, &SPI_GPIO_MAX11136);
    SPI_stack_error(ERROR_BASE_MAX11136 + MAX111XX_ERROR_BASE_SPI);
    return status;
}

/*******************************************************************/
MAX111XX_status_t MAX111XX_HW_spi_write_read_16(uint16_t* tx_data, uint16_t* rx_data, uint8_t transfer_size) {
    // Local variables.
    MAX111XX_status_t status = MAX111XX_SUCCESS;
    SPI_status_t spi_status = SPI_SUCCESS;
    // CS low.
    GPIO_write(&GPIO_MAX11136_CS, 0);
    // SPI transfer.
    spi_status = SPI_write_read_16(SPI_INSTANCE_ADC, tx_data, rx_data, transfer_size);
    SPI_exit_error(MAX111XX_ERROR_BASE_SPI);
errors:
    // CS high.
    GPIO_write(&GPIO_MAX11136_CS, 1);
    return status;
}

/*******************************************************************/
MAX111XX_status_t MAX111XX_HW_gpio_read_eoc(uint8_t* state) {
    // Local variables.
    MAX111XX_status_t status = MAX111XX_SUCCESS;
    // Check parameter.
    if (state == NULL) {
        status = MAX111XX_ERROR_NULL_PARAMETER;
        goto errors;
    }
    (*state) = GPIO_read(&GPIO_MAX11136_EOC);
errors:
    return status;
}

/*******************************************************************/
MAX111XX_status_t MAX111XX_HW_delay_milliseconds(uint32_t delay_ms) {
    // Local variables.
    MAX111XX_status_t status = MAX111XX_SUCCESS;
    LPTIM_status_t lptim_status = LPTIM_SUCCESS;
    // Perform delay.
    lptim_status = LPTIM_delay_milliseconds(delay_ms, LPTIM_DELAY_MODE_SLEEP);
    LPTIM_exit_error(MAX111XX_ERROR_BASE_DELAY);
errors:
    return status;
}

#endif /* MAX111XX_DRIVER_DISABLE */
