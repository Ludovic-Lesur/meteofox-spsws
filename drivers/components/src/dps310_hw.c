/*
 * dps310_hw.c
 *
 *  Created on: 30 aug. 2024
 *      Author: Ludo
 */

#include "dps310_hw.h"

#ifndef DPS310_DRIVER_DISABLE_FLAGS_FILE
#include "dps310_driver_flags.h"
#endif
#include "dps310.h"
#include "error_base.h"
#include "sensors_hw.h"
#include "types.h"

#ifndef DPS310_DRIVER_DISABLE

/*** DPS310 HW functions ***/

/*******************************************************************/
DPS310_status_t DPS310_HW_init(void) {
    return ((DPS310_status_t) SENSORS_HW_init(DPS310_ERROR_BASE_I2C));
}

/*******************************************************************/
DPS310_status_t DPS310_HW_de_init(void) {
    return ((DPS310_status_t) SENSORS_HW_de_init(ERROR_BASE_DPS310 + DPS310_ERROR_BASE_I2C));
}

/*******************************************************************/
DPS310_status_t DPS310_HW_i2c_write(uint8_t i2c_address, uint8_t* data, uint8_t data_size_bytes, uint8_t stop_flag) {
    return ((DPS310_status_t) SENSORS_HW_i2c_write(DPS310_ERROR_BASE_I2C, i2c_address, data, data_size_bytes, stop_flag));
}

/*******************************************************************/
DPS310_status_t DPS310_HW_i2c_read(uint8_t i2c_address, uint8_t* data, uint8_t data_size_bytes) {
    return ((DPS310_status_t) SENSORS_HW_i2c_read(DPS310_ERROR_BASE_I2C, i2c_address, data, data_size_bytes));
}

/*******************************************************************/
DPS310_status_t DPS310_HW_delay_milliseconds(uint32_t delay_ms) {
    return ((DPS310_status_t) SENSORS_HW_delay_milliseconds(DPS310_ERROR_BASE_DELAY, delay_ms));
}

#endif /* DPS310_DRIVER_DISABLE */
