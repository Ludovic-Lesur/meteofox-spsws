/*
 * rfe.c
 *
 *  Created on: 02 sep. 2024
 *      Author: Ludo
 */

#include "rfe.h"

#ifndef SIGFOX_EP_DISABLE_FLAGS_FILE
#include "sigfox_ep_flags.h"
#endif
#include "error.h"
#include "error_base.h"
#include "gpio.h"
#include "mcu_mapping.h"
#include "sx1232.h"
#include "types.h"

/*** RFE local macros ***/

#define RFE_RX_GAIN_DB  19

/*** RFE functions ***/

/*******************************************************************/
RFE_status_t RFE_init(void) {
    // Local variables.
    RFE_status_t status = RFE_SUCCESS;
    // Configure GPIOs.
#ifdef HW1_0
    GPIO_configure(&GPIO_RF_CHANNEL_A, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure(&GPIO_RF_CHANNEL_B, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_write(&GPIO_RF_CHANNEL_A, 0);
    GPIO_write(&GPIO_RF_CHANNEL_B, 0);
#endif
#ifdef HW2_0
    GPIO_configure(&GPIO_RF_TX_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure(&GPIO_RF_RX_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_write(&GPIO_RF_TX_ENABLE, 0);
    GPIO_write(&GPIO_RF_RX_ENABLE, 0);
#endif
    return status;
}

/*******************************************************************/
RFE_status_t RFE_de_init(void) {
    // Local variables.
    RFE_status_t status = RFE_SUCCESS;
    RFE_status_t rfe_status = RFE_SUCCESS;
    // Set all pins to output low.
    rfe_status = RFE_set_path(RFE_PATH_NONE);
    RFE_stack_error(ERROR_BASE_RFE);
    return status;
}

/*******************************************************************/
RFE_status_t RFE_set_path(RFE_path_t radio_path) {
    // Local variables.
    RFE_status_t status = RFE_SUCCESS;
    // Reset channels.
#ifdef HW1_0
    GPIO_write(&GPIO_RF_CHANNEL_A, 0);
    GPIO_write(&GPIO_RF_CHANNEL_B, 0);
#endif
#ifdef HW2_0
    GPIO_write(&GPIO_RF_TX_ENABLE, 0);
    GPIO_write(&GPIO_RF_RX_ENABLE, 0);
#endif
    // Select channel.
    switch (radio_path) {
    case RFE_PATH_NONE:
        // Already done by previous reset.
        break;
    case RFE_PATH_TX_BYPASS:
#ifdef HW1_0
        GPIO_write(&GPIO_RF_CHANNEL_A, 1);
#endif
#ifdef HW2_0
        GPIO_write(&GPIO_RF_TX_ENABLE, 1);
#endif
        break;
#ifdef HW1_0
    case RFE_PATH_TX_PA:
        GPIO_write(&GPIO_RF_CHANNEL_B, 1);
        break;
#endif
#ifdef SIGFOX_EP_BIDIRECTIONAL
    case RFE_PATH_RX_LNA:
#ifdef HW1_0
        GPIO_write(&GPIO_RF_CHANNEL_A, 1);
        GPIO_write(&GPIO_RF_CHANNEL_B, 1);
#endif
#ifdef HW2_0
        GPIO_write(&GPIO_RF_RX_ENABLE, 1);
#endif
        break;
#endif
    default:
        status = RFE_ERROR_PATH;
        goto errors;
    }
errors:
    return status;
}

#ifdef SIGFOX_EP_BIDIRECTIONAL
/*******************************************************************/
RFE_status_t RFE_get_rssi(int16_t* rssi_dbm) {
    // Local variables.
    RFE_status_t status = RFE_SUCCESS;
    SX1232_status_t sx1232_status = SX1232_SUCCESS;
    // Read raw RSSI.
    sx1232_status = SX1232_get_rssi(rssi_dbm);
    SX1232_exit_error(RFE_ERROR_BASE_SX1232);
    // Apply calibration gain.
    (*rssi_dbm) -= RFE_RX_GAIN_DB;
errors:
    return status;
}
#endif
