/*
 * ultimeter_hw.c
 *
 *  Created on: 13 may 2025
 *      Author: Ludo
 */

#include "ultimeter_hw.h"

#ifndef ULTIMETER_DRIVER_DISABLE_FLAGS_FILE
#include "ultimeter_driver_flags.h"
#endif
#include "error.h"
#include "error_base.h"
#include "exti.h"
#include "gpio.h"
#include "lptim.h"
#include "mcu_mapping.h"
#include "nvic_priority.h"
#include "sensors_hw.h"
#include "ultimeter.h"
#include "types.h"

#ifndef ULTIMETER_DRIVER_DISABLE

/*** ULTIMETER_HW local macros ***/

#define ULTIMETER_HW_GPIO_WIND_SPEED        GPIO_DIO0
#define ULTIMETER_HW_GPIO_WIND_DIRECTION    GPIO_DIO1

/*** ULTIMETER HW functions ***/

/*******************************************************************/
ULTIMETER_status_t ULTIMETER_HW_init(ULTIMETER_HW_configuration_t* configuration) {
    // Local variables.
    ULTIMETER_status_t status = ULTIMETER_SUCCESS;
    // Note: the most accurate configuration is: screw facing the south, pull-up resistors on both signals and using the rising edges.
    // Init wind speed GPIO.
    EXTI_configure_gpio(&ULTIMETER_HW_GPIO_WIND_SPEED, GPIO_PULL_NONE, EXTI_TRIGGER_RISING_EDGE, (configuration->wind_speed_edge_irq_callback), NVIC_PRIORITY_WIND_SPEED);
    // Init wind direction GPIO.
    EXTI_configure_gpio(&ULTIMETER_HW_GPIO_WIND_DIRECTION, GPIO_PULL_NONE, EXTI_TRIGGER_RISING_EDGE, (configuration->wind_direction_edge_irq_callback), NVIC_PRIORITY_WIND_DIRECTION);
    // Store tick second callback which will be used in main (RTC).
    SENSORS_HW_set_wind_tick_second_callback(configuration->tick_second_irq_callback);
    return status;
}

/*******************************************************************/
ULTIMETER_status_t ULTIMETER_HW_de_init(void) {
    // Local variables.
    ULTIMETER_status_t status = ULTIMETER_SUCCESS;
    // Release GPIOs.
    EXTI_release_gpio(&ULTIMETER_HW_GPIO_WIND_SPEED, GPIO_MODE_ANALOG);
    EXTI_release_gpio(&ULTIMETER_HW_GPIO_WIND_DIRECTION, GPIO_MODE_ANALOG);
    return status;
}

/*******************************************************************/
ULTIMETER_status_t ULTIMETER_HW_set_wind_speed_direction_interrupts(uint8_t enable) {
    // Local variables.
    ULTIMETER_status_t status = ULTIMETER_SUCCESS;
    // Check enable bit.
    if (enable == 0) {
        EXTI_disable_gpio_interrupt(&ULTIMETER_HW_GPIO_WIND_SPEED);
        EXTI_disable_gpio_interrupt(&ULTIMETER_HW_GPIO_WIND_DIRECTION);
        EXTI_clear_gpio_flag(&ULTIMETER_HW_GPIO_WIND_SPEED);
        EXTI_clear_gpio_flag(&ULTIMETER_HW_GPIO_WIND_DIRECTION);
    }
    else {
        EXTI_enable_gpio_interrupt(&ULTIMETER_HW_GPIO_WIND_SPEED);
        EXTI_enable_gpio_interrupt(&ULTIMETER_HW_GPIO_WIND_DIRECTION);
    }
    return status;
}

/*******************************************************************/
ULTIMETER_status_t ULTIMETER_HW_timer_start(void) {
    // Local variables.
    ULTIMETER_status_t status = ULTIMETER_SUCCESS;
    LPTIM_status_t lptim_status = LPTIM_SUCCESS;
    // Start timer.
    lptim_status = LPTIM_start(LPTIM_CLOCK_PRESCALER_4);
    LPTIM_exit_error(ULTIMETER_ERROR_BASE_TIMER);
errors:
    return status;
}

/*******************************************************************/
ULTIMETER_status_t ULTIMETER_HW_timer_stop(void) {
    // Local variables.
    ULTIMETER_status_t status = ULTIMETER_SUCCESS;
    LPTIM_status_t lptim_status = LPTIM_SUCCESS;
    // Stop timer.
    lptim_status = LPTIM_stop();
    LPTIM_stack_error(ERROR_BASE_ULTIMETER + ULTIMETER_ERROR_BASE_TIMER);
    /* To be implemented */
    return status;
}
/*******************************************************************/
uint32_t ULTIMETER_HW_timer_get_counter(void) {
    // Read counter.
    return LPTIM_get_counter();
}

#endif /* ULTIMETER_DRIVER_DISABLE */
