/*
 * terminal_hw.c
 *
 *  Created on: 29 sep. 2024
 *      Author: Ludo
 */

#include "terminal_hw.h"

#ifndef EMBEDDED_UTILS_DISABLE_FLAGS_FILE
#include "embedded_utils_flags.h"
#endif
#include "error.h"
#include "error_base.h"
#include "mcu_mapping.h"
#include "nvic_priority.h"
#include "terminal.h"
#include "types.h"
#include "usart.h"

#if (!(defined EMBEDDED_UTILS_TERMINAL_DRIVER_DISABLE) && (EMBEDDED_UTILS_TERMINAL_INSTANCES_NUMBER > 0))

/*** TERMINAL HW functions ***/

/*******************************************************************/
TERMINAL_status_t TERMINAL_HW_init(uint8_t instance, uint32_t baud_rate, TERMINAL_rx_irq_cb_t rx_irq_callback) {
    // Local variables.
    TERMINAL_status_t status = TERMINAL_SUCCESS;
    USART_status_t usart_status = USART_SUCCESS;
    USART_configuration_t usart_config;
    // Unused parameter.
    UNUSED(instance);
    // Init USART.
    usart_config.clock = RCC_CLOCK_HSI;
    usart_config.baud_rate = baud_rate;
    usart_config.nvic_priority = NVIC_PRIORITY_AT;
    usart_config.rxne_irq_callback = rx_irq_callback;
    usart_status = USART_init(USART_INSTANCE_AT, &USART_GPIO_AT, &usart_config);
    USART_exit_error(TERMINAL_ERROR_BASE_HW_INTERFACE);
errors:
    return status;
}

/*******************************************************************/
TERMINAL_status_t TERMINAL_HW_de_init(uint8_t instance) {
    // Local variables.
    TERMINAL_status_t status = TERMINAL_SUCCESS;
    USART_status_t usart_status = USART_SUCCESS;
    // Unused parameter.
    UNUSED(instance);
    // Release USART.
    usart_status = USART_de_init(USART_INSTANCE_AT, &USART_GPIO_AT);
    USART_stack_error(ERROR_BASE_TERMINAL_AT + TERMINAL_ERROR_BASE_HW_INTERFACE);
    return status;
}

/*******************************************************************/
TERMINAL_status_t TERMINAL_HW_enable_rx(uint8_t instance) {
    // Local variables.
    TERMINAL_status_t status = TERMINAL_SUCCESS;
    USART_status_t usart_status = USART_SUCCESS;
    // Unused parameter.
    UNUSED(instance);
    // Start reception.
    usart_status = USART_enable_rx(USART_INSTANCE_AT);
    USART_exit_error(TERMINAL_ERROR_BASE_HW_INTERFACE);
errors:
    return status;
}

/*******************************************************************/
TERMINAL_status_t TERMINAL_HW_disable_rx(uint8_t instance) {
    // Local variables.
    TERMINAL_status_t status = TERMINAL_SUCCESS;
    USART_status_t usart_status = USART_SUCCESS;
    // Unused parameter.
    UNUSED(instance);
    // Stop reception.
    usart_status = USART_disable_rx(USART_INSTANCE_AT);
    USART_exit_error(TERMINAL_ERROR_BASE_HW_INTERFACE);
errors:
    return status;
}

/*******************************************************************/
TERMINAL_status_t TERMINAL_HW_write(uint8_t instance, uint8_t* data, uint32_t data_size_bytes) {
    // Local variables.
    TERMINAL_status_t status = TERMINAL_SUCCESS;
    USART_status_t usart_status = USART_SUCCESS;
    // Unused parameter.
    UNUSED(instance);
    // Write data over USART.
    usart_status = USART_write(USART_INSTANCE_AT, data, data_size_bytes);
    USART_exit_error(TERMINAL_ERROR_BASE_HW_INTERFACE);
errors:
    return status;
}

#ifdef EMBEDDED_UTILS_TERMINAL_MODE_BUS
/*******************************************************************/
TERMINAL_status_t TERMINAL_HW_set_destination_address(uint8_t instance, uint8_t destination_address) {
    // Local variables.
    TERMINAL_status_t status = TERMINAL_SUCCESS;
    /* To be implemented */
    UNUSED(instance);
    UNUSED(destination_address);
    return status;
}
#endif

#endif /* EMBEDDED_UTILS_TERMINAL_DRIVER_DISABLE */
