/*
 * gps.c
 *
 *  Created on: 21 aug. 2024
 *      Author: Ludo
 */

#include "gps.h"

#include "error.h"
#include "error_base.h"
#include "iwdg.h"
#include "neom8x.h"
#include "pwr.h"
#include "rtc.h"
#include "types.h"

/*** GPS local structures ***/

/*******************************************************************/
typedef struct {
    volatile uint8_t process_flag;
    NEOM8X_acquisition_status_t acquisition_status;
} GPS_context_t;

/*** GPS local global variables ***/

static GPS_context_t gps_ctx = {
    .process_flag = 0,
    .acquisition_status = NEOM8X_ACQUISITION_STATUS_FAIL
};

/*** GPS local functions ***/

/*******************************************************************/
static void _GPS_process_callback(void) {
    // Set local flag.
    gps_ctx.process_flag = 1;
}

/*******************************************************************/
static void _GPS_completion_callback(NEOM8X_acquisition_status_t acquisition_status) {
    // Update global variable.
    gps_ctx.acquisition_status = acquisition_status;
}

/*******************************************************************/
static GPS_status_t _GPS_perform_acquisition(NEOM8X_gps_data_t gps_data, NEOM8X_acquisition_status_t expected_acquisition_status, uint32_t timeout_seconds, uint32_t* acquisition_duration_seconds) {
    // Local variables.
    GPS_status_t status = GPS_SUCCESS;
    NEOM8X_status_t neom8x_status;
    NEOM8X_acquisition_t gps_acquisition;
    uint32_t uptime = RTC_get_uptime_seconds();
    uint32_t start_time = uptime;
    // Reset data.
    gps_ctx.acquisition_status = NEOM8X_ACQUISITION_STATUS_FAIL;
    (*acquisition_duration_seconds) = 0;
    // Configure GPS acquisition.
    gps_acquisition.gps_data = gps_data;
    gps_acquisition.completion_callback = &_GPS_completion_callback;
    gps_acquisition.process_callback = &_GPS_process_callback;
    // Start acquisition.
    neom8x_status = NEOM8X_start_acquisition(&gps_acquisition);
    NEOM8X_exit_error(GPS_ERROR_BASE_NEOM8N);
    // Processing loop.
    while (uptime < (start_time + timeout_seconds)) {
        // Ensure RTC is running.
        if (RTC_get_uptime_seconds() > uptime) {
            // Update time and reload watchdog.
            uptime = RTC_get_uptime_seconds();
            IWDG_reload();
        }
        // Enter sleep mode.
        PWR_enter_sleep_mode(PWR_SLEEP_MODE_NORMAL);
        // Update acquisition duration.
        (*acquisition_duration_seconds) = (uptime - start_time);
        // Check flag.
        if (gps_ctx.process_flag != 0) {
            // Process driver.
            neom8x_status = NEOM8X_process();
            NEOM8X_exit_error(GPS_ERROR_BASE_NEOM8N);
        }
        // Check acquisition status.
        if (gps_ctx.acquisition_status == expected_acquisition_status) break;
    }
    neom8x_status = NEOM8X_stop_acquisition();
    NEOM8X_exit_error(GPS_ERROR_BASE_NEOM8N);
errors:
    NEOM8X_stop_acquisition();
    return status;
}

/*** GPS functions ***/

/*******************************************************************/
GPS_status_t GPS_init(void) {
    // Local variables.
    GPS_status_t status = GPS_SUCCESS;
    NEOM8X_status_t neom8x_status = NEOM8X_SUCCESS;
    // Init GPS module.
    neom8x_status = NEOM8X_init();
    NEOM8X_exit_error(GPS_ERROR_BASE_NEOM8N);
errors:
    return status;
}

/*******************************************************************/
GPS_status_t GPS_de_init(void) {
    // Local variables.
    GPS_status_t status = GPS_SUCCESS;
    NEOM8X_status_t neom8x_status = NEOM8X_SUCCESS;
    // Init GPS module.
    neom8x_status = NEOM8X_de_init();
    NEOM8X_stack_error(ERROR_BASE_GPS + GPS_ERROR_BASE_NEOM8N);
    return status;
}

/*******************************************************************/
GPS_status_t GPS_get_time(GPS_time_t* gps_time, uint32_t timeout_seconds, uint32_t* acquisition_duration_seconds, GPS_acquisition_status_t* acquisition_status) {
    // Local variables.
    GPS_status_t status = GPS_SUCCESS;
    NEOM8X_status_t neom8x_status = NEOM8X_SUCCESS;
    // Check parameters.
    if ((acquisition_status == NULL) || (acquisition_status == NULL)) {
        status = GPS_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Reset output data.
    (*acquisition_duration_seconds) = 0;
    (*acquisition_status) = GPS_ACQUISITION_ERROR_TIMEOUT;
    // Perform time acquisition.
    status = _GPS_perform_acquisition(NEOM8X_GPS_DATA_TIME, NEOM8X_ACQUISITION_STATUS_FOUND, timeout_seconds, acquisition_duration_seconds);
    if (status != GPS_SUCCESS) goto errors;
    // Check status.
    if (gps_ctx.acquisition_status != NEOM8X_ACQUISITION_STATUS_FAIL) {
        // Read data.
        neom8x_status = NEOM8X_get_time(gps_time);
        NEOM8X_exit_error(GPS_ERROR_BASE_NEOM8N);
        // Update status.
        (*acquisition_status) = GPS_ACQUISITION_SUCCESS;
    }
errors:
    return status;
}

/*******************************************************************/
GPS_status_t GPS_get_position(GPS_position_t* gps_position, uint32_t timeout_seconds, uint32_t* acquisition_duration_seconds, GPS_acquisition_status_t* acquisition_status) {
    // Local variables.
    GPS_status_t status = GPS_SUCCESS;
    NEOM8X_status_t neom8x_status = NEOM8X_SUCCESS;
    // Check parameters.
    if ((acquisition_duration_seconds == NULL) || (acquisition_status == NULL)) {
        status = GPS_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Reset output data.
    (*acquisition_duration_seconds) = 0;
    (*acquisition_status) = GPS_ACQUISITION_ERROR_TIMEOUT;
    // Perform position acquisition.
    status = _GPS_perform_acquisition(NEOM8X_GPS_DATA_POSITION, NEOM8X_ACQUISITION_STATUS_STABLE, timeout_seconds, acquisition_duration_seconds);
    if (status != GPS_SUCCESS) goto errors;
    // Check status.
    if (gps_ctx.acquisition_status != NEOM8X_ACQUISITION_STATUS_FAIL) {
        // Read data.
        neom8x_status = NEOM8X_get_position(gps_position);
        NEOM8X_exit_error(GPS_ERROR_BASE_NEOM8N);
        // Update status.
        (*acquisition_status) = GPS_ACQUISITION_SUCCESS;
    }
errors:
    return status;
}
