/*
 * sen15901_hw.c
 *
 *  Created on: 07 sep. 2024
 *      Author: Ludo
 */

#include "sen15901_hw.h"

#ifndef SEN15901_DRIVER_DISABLE_FLAGS_FILE
#include "sen15901_driver_flags.h"
#endif
#include "analog.h"
#include "error.h"
#include "exti.h"
#include "mcu_mapping.h"
#include "nvic.h"
#include "nvic_priority.h"
#include "power.h"
#include "sen15901.h"
#include "sensors_hw.h"
#include "spsws_flags.h"
#include "types.h"

#ifndef SEN15901_DRIVER_DISABLE

/*** SEN15901_HW local macros ***/

#define SEN15901_HW_GPIO_WIND_SPEED     GPIO_DIO0
#define SEN15901_HW_GPIO_RAINFALL       GPIO_DIO2

/*** SEN15901 HW functions ***/

/*******************************************************************/
SEN15901_status_t SEN15901_HW_init(SEN15901_HW_configuration_t* configuration) {
    // Local variables.
    SEN15901_status_t status = SEN15901_SUCCESS;
#ifndef SPSWS_WIND_VANE_ULTIMETER
    // Init wind speed GPIO.
    EXTI_configure_gpio(&SEN15901_HW_GPIO_WIND_SPEED, GPIO_PULL_NONE, EXTI_TRIGGER_FALLING_EDGE, (configuration->wind_speed_edge_irq_callback), NVIC_PRIORITY_WIND_SPEED);
#endif
    // Init rainfall GPIO.
    EXTI_configure_gpio(&SEN15901_HW_GPIO_RAINFALL, GPIO_PULL_NONE, EXTI_TRIGGER_FALLING_EDGE, (configuration->rainfall_edge_irq_callback), NVIC_PRIORITY_RAINFALL);
#ifndef SPSWS_WIND_VANE_ULTIMETER
    // Store tick second callback which will be used in main (RTC).
    SENSORS_HW_set_wind_tick_second_callback(configuration->tick_second_irq_callback);
    // Note: ADC will be initialized in the power enable function.
#endif
    return status;
}

/*******************************************************************/
SEN15901_status_t SEN15901_HW_de_init(void) {
    // Local variables.
    SEN15901_status_t status = SEN15901_SUCCESS;
    // Release GPIOs.
#ifndef SPSWS_WIND_VANE_ULTIMETER
    EXTI_release_gpio(&SEN15901_HW_GPIO_WIND_SPEED, GPIO_MODE_ANALOG);
#endif
    EXTI_release_gpio(&SEN15901_HW_GPIO_RAINFALL, GPIO_MODE_ANALOG);
    return status;
}

#ifndef SPSWS_WIND_VANE_ULTIMETER
/*******************************************************************/
SEN15901_status_t SEN15901_HW_set_wind_speed_interrupt(uint8_t enable) {
    // Local variables.
    SEN15901_status_t status = SEN15901_SUCCESS;
    // Check enable bit.
    if (enable == 0) {
        EXTI_disable_gpio_interrupt(&SEN15901_HW_GPIO_WIND_SPEED);
        EXTI_clear_gpio_flag(&SEN15901_HW_GPIO_WIND_SPEED);
    }
    else {
        EXTI_enable_gpio_interrupt(&SEN15901_HW_GPIO_WIND_SPEED);
    }
    return status;
}
#endif

/*******************************************************************/
SEN15901_status_t SEN15901_HW_set_rainfall_interrupt(uint8_t enable) {
    // Local variables.
    SEN15901_status_t status = SEN15901_SUCCESS;
    // Check enable bit.
    if (enable == 0) {
        EXTI_disable_gpio_interrupt(&SEN15901_HW_GPIO_RAINFALL);
        EXTI_clear_gpio_flag(&SEN15901_HW_GPIO_RAINFALL);
    }
    else {
        EXTI_enable_gpio_interrupt(&SEN15901_HW_GPIO_RAINFALL);
    }
    return status;
}

#ifndef SPSWS_WIND_VANE_ULTIMETER
/*******************************************************************/
SEN15901_status_t SEN15901_HW_adc_get_wind_direction_ratio(int32_t* wind_direction_ratio_permille) {
    // Local variables.
    SEN15901_status_t status = SEN15901_SUCCESS;
    ANALOG_status_t analog_status = ANALOG_SUCCESS;
    // Turn external ADC on.
    POWER_enable(POWER_REQUESTER_ID_SEN15901, POWER_DOMAIN_ANALOG, LPTIM_DELAY_MODE_SLEEP);
    // Get direction from ADC.
    analog_status = ANALOG_convert_channel(ANALOG_CHANNEL_WIND_DIRECTION_RATIO_PERMILLE, wind_direction_ratio_permille);
    ANALOG_exit_error(SEN15901_ERROR_BASE_ADC);
errors:
    // Turn external ADC off.
    POWER_disable(POWER_REQUESTER_ID_SEN15901, POWER_DOMAIN_ANALOG);
    return status;
}
#endif

#endif /* SEN15901_DRIVER_DISABLE */
