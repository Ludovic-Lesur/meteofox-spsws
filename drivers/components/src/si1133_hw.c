/*
 * si1133_hw.c
 *
 *  Created on: 31 aug. 2024
 *      Author: Ludo
 */

#include "si1133_hw.h"

#ifndef SI1133_DRIVER_DISABLE_FLAGS_FILE
#include "si1133_driver_flags.h"
#endif
#include "error_base.h"
#include "sensors_hw.h"
#include "si1133.h"
#include "types.h"

#ifndef SI1133_DRIVER_DISABLE

/*** SI1133 HW functions ***/

/*******************************************************************/
SI1133_status_t SI1133_HW_init(void) {
    return ((SI1133_status_t) SENSORS_HW_init(SI1133_ERROR_BASE_I2C));
}

/*******************************************************************/
SI1133_status_t SI1133_HW_de_init(void) {
    return ((SI1133_status_t) SENSORS_HW_de_init(ERROR_BASE_SI1133 + SI1133_ERROR_BASE_I2C));
}

/*******************************************************************/
SI1133_status_t SI1133_HW_i2c_write(uint8_t i2c_address, uint8_t* data, uint8_t data_size_bytes, uint8_t stop_flag) {
    return ((SI1133_status_t) SENSORS_HW_i2c_write(SI1133_ERROR_BASE_I2C, i2c_address, data, data_size_bytes, stop_flag));
}

/*******************************************************************/
SI1133_status_t SI1133_HW_i2c_read(uint8_t i2c_address, uint8_t* data, uint8_t data_size_bytes) {
    return ((SI1133_status_t) SENSORS_HW_i2c_read(SI1133_ERROR_BASE_I2C, i2c_address, data, data_size_bytes));
}

/*******************************************************************/
SI1133_status_t SI1133_HW_delay_milliseconds(uint32_t delay_ms) {
    return ((SI1133_status_t) SENSORS_HW_delay_milliseconds(SI1133_ERROR_BASE_DELAY, delay_ms));
}

#endif /* SI1133_DRIVER_DISABLE */
