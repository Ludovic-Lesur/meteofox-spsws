/*
 * sht3x_hw.c
 *
 *  Created on: 27 aug. 2024
 *      Author: Ludo
 */

#include "sht3x_hw.h"

#ifndef SHT3X_DRIVER_DISABLE_FLAGS_FILE
#include "sht3x_driver_flags.h"
#endif
#include "error_base.h"
#include "sensors_hw.h"
#include "sht3x.h"
#include "types.h"

#ifndef SHT3X_DRIVER_DISABLE

/*** SHT3X HW functions ***/

/*******************************************************************/
SHT3X_status_t SHT3X_HW_init(void) {
    return ((SHT3X_status_t) SENSORS_HW_init(SHT3X_ERROR_BASE_I2C));
}

/*******************************************************************/
SHT3X_status_t SHT3X_HW_de_init(void) {
    return ((SHT3X_status_t) SENSORS_HW_de_init(ERROR_BASE_SHT3X + SHT3X_ERROR_BASE_I2C));
}

/*******************************************************************/
SHT3X_status_t SHT3X_HW_i2c_write(uint8_t i2c_address, uint8_t* data, uint8_t data_size_bytes, uint8_t stop_flag) {
    return ((SHT3X_status_t) SENSORS_HW_i2c_write(SHT3X_ERROR_BASE_I2C, i2c_address, data, data_size_bytes, stop_flag));
}

/*******************************************************************/
SHT3X_status_t SHT3X_HW_i2c_read(uint8_t i2c_address, uint8_t* data, uint8_t data_size_bytes) {
    return ((SHT3X_status_t) SENSORS_HW_i2c_read(SHT3X_ERROR_BASE_I2C, i2c_address, data, data_size_bytes));
}

/*******************************************************************/
SHT3X_status_t SHT3X_HW_delay_milliseconds(uint32_t delay_ms) {
    return ((SHT3X_status_t) SENSORS_HW_delay_milliseconds(SHT3X_ERROR_BASE_DELAY, delay_ms));
}

#endif /* SHT3X_DRIVER_DISABLE */
