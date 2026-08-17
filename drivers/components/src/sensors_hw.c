/*
 * sensors_hw.c
 *
 *  Created on: 31 aug. 2024
 *      Author: Ludo
 */

#include "sensors_hw.h"

#include "error.h"
#include "error_base.h"
#include "i2c.h"
#include "lptim.h"
#include "mcu_mapping.h"
#include "sen15901_hw.h"
#include "spsws_flags.h"
#include "types.h"
#include "ultimeter_hw.h"

/*** SENSORS HW local global variables ***/

#ifdef SPSWS_WIND_RAINFALL_MEASUREMENTS
#ifdef SPSWS_WIND_VANE_ULTIMETER
static ULTIMETER_HW_tick_second_irq_cb_t sensors_hw_wind_tick_second_callback = NULL;
#else
static SEN15901_HW_tick_second_irq_cb_t sensors_hw_wind_tick_second_callback = NULL;
#endif
#endif

/*** SENSORS HW functions ***/

/*******************************************************************/
ERROR_code_t SENSORS_HW_init(ERROR_code_t i2c_error_base) {
    // Local variables.
    ERROR_code_t status = SUCCESS;
    I2C_status_t i2c_status = I2C_SUCCESS;
    // Init I2C.
    i2c_status = I2C_init(I2C_INSTANCE_SENSORS, &I2C_GPIO_SENSORS);
    I2C_exit_error(i2c_error_base);
errors:
    return status;
}

/*******************************************************************/
ERROR_code_t SENSORS_HW_de_init(ERROR_code_t i2c_error_base) {
    // Local variables.
    ERROR_code_t status = SUCCESS;
    I2C_status_t i2c_status = I2C_SUCCESS;
    // Init I2C.
    i2c_status = I2C_de_init(I2C_INSTANCE_SENSORS, &I2C_GPIO_SENSORS);
    I2C_stack_error(i2c_error_base);
    return status;
}

/*******************************************************************/
ERROR_code_t SENSORS_HW_i2c_write(ERROR_code_t i2c_error_base, uint8_t i2c_address, uint8_t* data, uint8_t data_size_bytes, uint8_t stop_flag) {
    // Local variables.
    ERROR_code_t status = SUCCESS;
    I2C_status_t i2c_status = I2C_SUCCESS;
    // I2C transfer.
    i2c_status = I2C_write(I2C_INSTANCE_SENSORS, i2c_address, data, data_size_bytes, stop_flag);
    I2C_exit_error(i2c_error_base);
errors:
    return status;
}

/*******************************************************************/
ERROR_code_t SENSORS_HW_i2c_read(ERROR_code_t i2c_error_base, uint8_t i2c_address, uint8_t* data, uint8_t data_size_bytes) {
    // Local variables.
    ERROR_code_t status = SUCCESS;
    I2C_status_t i2c_status = I2C_SUCCESS;
    // I2C transfer.
    i2c_status = I2C_read(I2C_INSTANCE_SENSORS, i2c_address, data, data_size_bytes);
    I2C_exit_error(i2c_error_base);
errors:
    return status;
}

/*******************************************************************/
ERROR_code_t SENSORS_HW_delay_milliseconds(ERROR_code_t delay_error_base, uint32_t delay_ms) {
    // Local variables.
    ERROR_code_t status = SUCCESS;
    LPTIM_status_t lptim_status = LPTIM_SUCCESS;
    // Perform delay.
    lptim_status = LPTIM_delay_milliseconds(delay_ms, LPTIM_DELAY_MODE_SLEEP);
    LPTIM_exit_error(delay_error_base);
errors:
    return status;
}

#ifdef SPSWS_WIND_RAINFALL_MEASUREMENTS
/*******************************************************************/
void SENSORS_HW_set_wind_tick_second_callback(SENSORS_HW_wind_tick_second_irq_cb_t tick_second_callback) {
    sensors_hw_wind_tick_second_callback = tick_second_callback;
}
#endif

#ifdef SPSWS_WIND_RAINFALL_MEASUREMENTS
/*******************************************************************/
void SENSORS_HW_get_wind_tick_second_callback(SENSORS_HW_wind_tick_second_irq_cb_t* tick_second_callback) {
    (*tick_second_callback) = sensors_hw_wind_tick_second_callback;
}
#endif
