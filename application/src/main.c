/*
 * main.c
 *
 *  Created on: 25 apr. 2018
 *      Author: Ludo
 */

// Peripherals.
#include "exti.h"
#include "gpio.h"
#include "i2c_address.h"
#include "iwdg.h"
#include "lptim.h"
#include "mcu_mapping.h"
#include "nvic.h"
#include "nvic_priority.h"
#include "nvm.h"
#include "nvm_address.h"
#include "pwr.h"
#include "rcc.h"
#include "rtc.h"
// Utils.
#include "error.h"
#include "maths.h"
#include "types.h"
// Components.
#include "dps310.h"
#include "sen15901.h"
#include "sen15901_hw.h"
#include "sensors_hw.h"
#include "sht3x.h"
#include "si1133.h"
#include "sigfox_types.h"
#include "ultimeter.h"
// Middleware.
#include "analog.h"
#include "cli.h"
#include "gps.h"
#include "power.h"
// Sigfox.
#include "sigfox_ep_flags.h"
#include "sigfox_ep_frames.h"
#include "sigfox_ep_api.h"
#include "sigfox_rc.h"
#include "sigfox_types.h"
// Applicative.
#include "error_base.h"
#include "spsws_flags.h"
#include "version.h"

/*** SPSWS macros ***/

// Timing.
#define SPSWS_POWER_ON_DELAY_MS                         7000
#define SPSWS_RTC_CALIBRATION_TIMEOUT_SECONDS           180
#define SPSWS_GEOLOC_TIMEOUT_SECONDS                    120
// Voltage hysteresis for radio.
#define SPSWS_RADIO_OFF_VCAP_THRESHOLD_MV               1000
#define SPSWS_RADIO_ON_VCAP_THRESHOLD_MV                1500
// Voltage hysteresis for uplink period.
#define SPSWS_WEATHER_REQUEST_OFF_VCAP_THRESHOLD_MV     1500
#define SPSWS_WEATHER_REQUEST_ON_VCAP_THRESHOLD_MV      2000
// Measurements buffers length.
#define SPSWS_MEASUREMENT_PERIOD_SECONDS                60
#define SPSWS_MEASUREMENT_BUFFER_SIZE                   (3600 / SPSWS_MEASUREMENT_PERIOD_SECONDS)
#ifdef SPSWS_SEN15901_EMULATOR
#define SPSWS_SEN15901_EMULATOR_SYNCHRO_GPIO            GPIO_DIO4
#endif

/*** SPSWS structures ***/

/*******************************************************************/
typedef enum {
    SPSWS_STATE_STARTUP,
    SPSWS_STATE_MEASURE,
    SPSWS_STATE_WEATHER,
    SPSWS_STATE_MONITORING,
    SPSWS_STATE_GEOLOC,
    SPSWS_STATE_ERROR_STACK,
    SPSWS_STATE_RTC_CALIBRATION,
    SPSWS_STATE_TASK_END,
    SPSWS_STATE_TASK_CHECK,
    SPSWS_STATE_SLEEP,
    SPSWS_STATE_LAST
} SPSWS_state_t;

/*******************************************************************/
typedef union {
    uint8_t all;
    struct {
        unsigned daily_downlink :1;
        unsigned daily_geoloc :1;
        unsigned daily_rtc_calibration :1;
        unsigned first_rtc_calibration :1;
        unsigned lse_status :1;
        unsigned lsi_status :1;
        unsigned mcu_clock_source :1;
        unsigned station_mode :1;
    } __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
} SPSWS_status_t;

/*******************************************************************/
typedef union {
    uint16_t all;
    struct {
        unsigned ultimeter_process :1;
        unsigned sen15901_process :1;
        unsigned radio_enabled : 1;
        unsigned reset_request : 1;
        unsigned rtc_calibration_request : 1;
        unsigned error_stack_request :1;
        unsigned geoloc_request :1;
        unsigned downlink_request :1;
        unsigned weather_request_intermediate :1;
        unsigned weather_request_enabled :1;
        unsigned weather_request :1;
        unsigned monitoring_request :1;
        unsigned measure_request :1;
        unsigned valid_wakeup :1;
        unsigned sharp_hour_alarm :1;
        unsigned first_sharp_hour_alarm :1;
    } __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
} SPSWS_flags_t;

/*******************************************************************/
typedef struct {
    int32_t sample_buffer[SPSWS_MEASUREMENT_BUFFER_SIZE];
    uint32_t sample_count;
    uint32_t last_sample_index;
    uint8_t full_flag;
} SPSWS_measurement_t;

/*******************************************************************/
typedef struct {
    SPSWS_measurement_t tamb_degrees;
    SPSWS_measurement_t hamb_percent;
    SPSWS_measurement_t light_percent;
    SPSWS_measurement_t uv_index;
    SPSWS_measurement_t patm_abs_pa;
    SPSWS_measurement_t tmcu_degrees;
    SPSWS_measurement_t tpcb_degrees;
    SPSWS_measurement_t hpcb_percent;
    SPSWS_measurement_t vsrc_mv;
    SPSWS_measurement_t vcap_mv;
    SPSWS_measurement_t vmcu_mv;
} SPSWS_measurements_t;

/*******************************************************************/
typedef enum {
    SPSWS_NVM_DATA_LAST_WAKE_UP = 0,
    SPSWS_NVM_DATA_LAST_GEOLOC,
    SPSWS_NVM_DATA_LAST_DOWNLINK,
    SPSWS_NVM_DATA_LAST
} SPSWS_nvm_data_t;

/*******************************************************************/
typedef struct {
    // State machine.
    SPSWS_state_t state;
    SPSWS_status_t status;
    volatile SPSWS_flags_t flags;
    // Intermediate measurements.
    uint32_t measurements_last_time_seconds;
    SPSWS_measurements_t measurements;
#ifdef SIGFOX_EP_BIDIRECTIONAL
    // Weather data.
    volatile uint32_t sharp_hour_uptime;
    uint8_t weather_data_period;
    uint32_t weather_message_count;
    uint32_t weather_last_time_seconds;
#endif
#ifdef SPSWS_WIND_RAINFALL_MEASUREMENTS
    SENSORS_HW_wind_tick_second_irq_cb_t wind_tick_second_callback;
#endif
    // Sigfox frames.
    SPSWS_EP_ul_payload_weather_t sigfox_ep_ul_payload_weather;
    SIGFOX_EP_ul_payload_monitoring_t sigfox_ep_ul_payload_monitoring;
} SPSWS_context_t;

/*** SPSWS global variables ***/

#if (!(defined SPSWS_MODE_CLI) && (defined SIGFOX_EP_BIDIRECTIONAL))
static uint32_t SPSWS_WEATHER_DATA_PERIOD_SECONDS[SIGFOX_EP_DL_WEATHER_DATA_PERIOD_LAST] = { 3600, 1800, 1200, 900, 720, 600 };
#endif
#ifndef SPSWS_MODE_CLI
static SPSWS_context_t spsws_ctx;
#endif

/*** SPSWS local functions ***/

#ifndef SPSWS_MODE_CLI
/*******************************************************************/
static void _SPSWS_sharp_hour_alarm_callback(void) {
#ifdef SIGFOX_EP_BIDIRECTIONAL
    // Synchronize weather period.
    spsws_ctx.flags.first_sharp_hour_alarm = 1;
    spsws_ctx.flags.sharp_hour_alarm = 1;
    spsws_ctx.sharp_hour_uptime = RTC_get_uptime_seconds();
#else
    spsws_ctx.flags.weather_request = 1;
    spsws_ctx.flags.monitoring_request = 1;
#endif
}
#endif

#if ((defined SPSWS_WIND_RAINFALL_MEASUREMENTS) && !(defined SPSWS_MODE_CLI))
/*******************************************************************/
static void _SPSWS_tick_second_callback(void) {
    // Execute wind driver callback.
    if (spsws_ctx.wind_tick_second_callback != NULL) {
        spsws_ctx.wind_tick_second_callback();
    }
}
#endif

#if ((defined SPSWS_WIND_RAINFALL_MEASUREMENTS) && !(defined SPSWS_MODE_CLI))
/*******************************************************************/
static void _SPSWS_sen15901_process_callback(void) {
    // Update local flag.
    spsws_ctx.flags.sen15901_process = 1;
}
#endif

#if ((defined SPSWS_WIND_RAINFALL_MEASUREMENTS) && (defined SPSWS_WIND_VANE_ULTIMETER))
/*******************************************************************/
static void _SPSWS_ultimeter_process_callback(void) {
    // Update local flag.
    spsws_ctx.flags.ultimeter_process = 1;
}
#endif

#if (!(defined SPSWS_MODE_CLI) && (defined SIGFOX_EP_BIDIRECTIONAL))
/*******************************************************************/
static void _SPSWS_load_weather_data_period(void) {
    // Local variables.
    NVM_status_t nvm_status = NVM_SUCCESS;
    uint8_t weather_data_period = 0;
    // Read monitoring period.
    nvm_status = NVM_read_byte(NVM_ADDRESS_WEATHER_DATA_PERIOD, &weather_data_period);
    NVM_stack_error(ERROR_BASE_NVM);
    // Check value.
    if (weather_data_period >= SIGFOX_EP_DL_WEATHER_DATA_PERIOD_LAST) {
        // Reset to default value.
        weather_data_period = SIGFOX_EP_DL_WEATHER_DATA_PERIOD_60_MINUTES;
    }
    spsws_ctx.weather_data_period = weather_data_period;
}
#endif

#if (!(defined SPSWS_MODE_CLI) && (defined SIGFOX_EP_BIDIRECTIONAL))
/*******************************************************************/
static void _SPSWS_store_weather_data_period(uint8_t weather_data_period) {
    // Local variables.
    NVM_status_t nvm_status = NVM_SUCCESS;
    // Monitoring period.
    if (weather_data_period < SIGFOX_EP_DL_WEATHER_DATA_PERIOD_LAST) {
        // Update context.
        spsws_ctx.weather_data_period  = weather_data_period;
        // Write new value in NVM.
        nvm_status = NVM_write_byte(NVM_ADDRESS_WEATHER_DATA_PERIOD, weather_data_period);
        NVM_stack_error(ERROR_BASE_NVM);
    }
    else {
        ERROR_stack_add(ERROR_SIGFOX_EP_DL_WEATHER_DATA_PERIOD);
    }
}
#endif

#ifndef SPSWS_MODE_CLI
/*******************************************************************/
static void _SPSWS_measurement_add_sample(SPSWS_measurement_t* measurement, int32_t sample) {
    /* Update last sample index */
    measurement->last_sample_index = (measurement->sample_count);
    /* Add sample to buffer */
    measurement->sample_buffer[measurement->sample_count] = sample;
    /* Increment index */
    (measurement->sample_count)++;
    /* Manage rollover and flag */
    if ((measurement->sample_count) >= SPSWS_MEASUREMENT_BUFFER_SIZE) {
        (measurement->sample_count) = 0;
        (measurement->full_flag) = 1;
    }
}
#endif

#ifndef SPSWS_MODE_CLI
/*******************************************************************/
static void _SPSWS_reset_measurements(void) {
    // Weather data
    spsws_ctx.measurements.tamb_degrees.sample_count = 0;
    spsws_ctx.measurements.tamb_degrees.full_flag = 0;
    spsws_ctx.measurements.hamb_percent.sample_count = 0;
    spsws_ctx.measurements.hamb_percent.full_flag = 0;
    spsws_ctx.measurements.light_percent.sample_count = 0;
    spsws_ctx.measurements.light_percent.full_flag = 0;
    spsws_ctx.measurements.uv_index.sample_count = 0;
    spsws_ctx.measurements.uv_index.full_flag = 0;
    spsws_ctx.measurements.patm_abs_pa.sample_count = 0;
    spsws_ctx.measurements.patm_abs_pa.full_flag = 0;
#ifdef SPSWS_WIND_RAINFALL_MEASUREMENTS
#ifdef SPSWS_WIND_VANE_ULTIMETER
    ULTIMETER_reset_measurements();
#endif
    SEN15901_reset_measurements();
#endif
    // Monitoring data.
    spsws_ctx.measurements.tmcu_degrees.sample_count = 0;
    spsws_ctx.measurements.tmcu_degrees.full_flag = 0;
    spsws_ctx.measurements.tpcb_degrees.sample_count = 0;
    spsws_ctx.measurements.tpcb_degrees.full_flag = 0;
    spsws_ctx.measurements.hpcb_percent.sample_count = 0;
    spsws_ctx.measurements.hpcb_percent.full_flag = 0;
    spsws_ctx.measurements.vsrc_mv.sample_count = 0;
    spsws_ctx.measurements.vsrc_mv.full_flag = 0;
    spsws_ctx.measurements.vmcu_mv.sample_count = 0;
    spsws_ctx.measurements.vmcu_mv.full_flag = 0;
}
#endif

#ifndef SPSWS_MODE_CLI
/*******************************************************************/
static void _SPSWS_compute_final_measurements(void) {
    // Local variables.
    MATH_status_t math_status = MATH_SUCCESS;
    uint32_t sample_count = 0;
    int32_t generic_s32_1 = 0;
    uint32_t generic_u32 = 0;
#ifdef SPSWS_WIND_RAINFALL_MEASUREMENTS
#ifdef SPSWS_WIND_VANE_ULTIMETER
    ULTIMETER_status_t ultimeter_status = ULTIMETER_SUCCESS;
    ULTIMETER_wind_direction_status_t wind_direction_status = ULTIMETER_WIND_DIRECTION_STATUS_AVAILABLE;
#else
    SEN15901_wind_direction_status_t wind_direction_status = SEN15901_WIND_DIRECTION_STATUS_AVAILABLE;
#endif
    SEN15901_status_t sen15901_status = SEN15901_SUCCESS;
    int32_t generic_s32_2 = 0;
#endif
    // Temperature.
    spsws_ctx.sigfox_ep_ul_payload_weather.tamb_degrees = SIGFOX_EP_ERROR_VALUE_TEMPERATURE;
    sample_count = (spsws_ctx.measurements.tamb_degrees.full_flag != 0) ? SPSWS_MEASUREMENT_BUFFER_SIZE : spsws_ctx.measurements.tamb_degrees.sample_count;
    if (sample_count > 0) {
        // Compute single value.
        math_status = MATH_min(spsws_ctx.measurements.tamb_degrees.sample_buffer, sample_count, &generic_s32_1);
        MATH_stack_error(ERROR_BASE_MATH);
        if (math_status == MATH_SUCCESS) {
            // Convert temperature.
            math_status = MATH_integer_to_signed_magnitude(generic_s32_1, (MATH_U8_SIZE_BITS - 1), &generic_u32);
            MATH_stack_error(ERROR_BASE_MATH);
            if (math_status == MATH_SUCCESS) {
                spsws_ctx.sigfox_ep_ul_payload_weather.tamb_degrees = (uint8_t) generic_u32;
            }
        }
    }
    // Humidity.
    spsws_ctx.sigfox_ep_ul_payload_weather.hamb_percent = SIGFOX_EP_ERROR_VALUE_HUMIDITY;
    sample_count = (spsws_ctx.measurements.hamb_percent.full_flag != 0) ? SPSWS_MEASUREMENT_BUFFER_SIZE : spsws_ctx.measurements.hamb_percent.sample_count;
    if (sample_count > 0) {
        // Compute single value.
        math_status = MATH_median_filter(spsws_ctx.measurements.hamb_percent.sample_buffer, sample_count, 0, &generic_s32_1);
        MATH_stack_error(ERROR_BASE_MATH);
        if (math_status == MATH_SUCCESS) {
            spsws_ctx.sigfox_ep_ul_payload_weather.hamb_percent = (uint8_t) generic_s32_1;
        }
    }
    // Light.
    spsws_ctx.sigfox_ep_ul_payload_weather.light_percent = SIGFOX_EP_ERROR_VALUE_LIGHT;
    sample_count = (spsws_ctx.measurements.light_percent.full_flag != 0) ? SPSWS_MEASUREMENT_BUFFER_SIZE : spsws_ctx.measurements.light_percent.sample_count;
    if (sample_count > 0) {
        // Compute single value.
        math_status = MATH_median_filter(spsws_ctx.measurements.light_percent.sample_buffer, sample_count, 0, &generic_s32_1);
        MATH_stack_error(ERROR_BASE_MATH);
        if (math_status == MATH_SUCCESS) {
            spsws_ctx.sigfox_ep_ul_payload_weather.light_percent = (uint8_t) generic_s32_1;
        }
    }
    // UV index.
    spsws_ctx.sigfox_ep_ul_payload_weather.uv_index = SIGFOX_EP_ERROR_VALUE_UV_INDEX;
    sample_count = (spsws_ctx.measurements.uv_index.full_flag != 0) ? SPSWS_MEASUREMENT_BUFFER_SIZE : spsws_ctx.measurements.uv_index.sample_count;
    if (sample_count > 0) {
        // Compute single value.
        math_status = MATH_max(spsws_ctx.measurements.uv_index.sample_buffer, sample_count, &generic_s32_1);
        MATH_stack_error(ERROR_BASE_MATH);
        if (math_status == MATH_SUCCESS) {
            spsws_ctx.sigfox_ep_ul_payload_weather.uv_index = (uint8_t) generic_s32_1;
        }
    }
    // Absolute pressure.
    spsws_ctx.sigfox_ep_ul_payload_weather.patm_abs_tenth_hpa = SIGFOX_EP_ERROR_VALUE_PRESSURE;
    sample_count = (spsws_ctx.measurements.patm_abs_pa.full_flag != 0) ? SPSWS_MEASUREMENT_BUFFER_SIZE : spsws_ctx.measurements.patm_abs_pa.sample_count;
    if (sample_count > 0) {
        // Compute single value.
        math_status = MATH_median_filter(spsws_ctx.measurements.patm_abs_pa.sample_buffer, sample_count, 0, &generic_s32_1);
        MATH_stack_error(ERROR_BASE_MATH);
        if (math_status == MATH_SUCCESS) {
            spsws_ctx.sigfox_ep_ul_payload_weather.patm_abs_tenth_hpa = (uint16_t) (generic_s32_1 / 10);
        }
    }
    // MCU temperature.
    spsws_ctx.sigfox_ep_ul_payload_monitoring.tmcu_degrees = SIGFOX_EP_ERROR_VALUE_TEMPERATURE;
    sample_count = (spsws_ctx.measurements.tmcu_degrees.full_flag != 0) ? SPSWS_MEASUREMENT_BUFFER_SIZE : spsws_ctx.measurements.tmcu_degrees.sample_count;
    if (sample_count > 0) {
        // Compute single value.
        math_status = MATH_min(spsws_ctx.measurements.tmcu_degrees.sample_buffer, sample_count, &generic_s32_1);
        MATH_stack_error(ERROR_BASE_MATH);
        if (math_status == MATH_SUCCESS) {
            // Convert temperature.
            math_status = MATH_integer_to_signed_magnitude(generic_s32_1, (MATH_U8_SIZE_BITS - 1), &generic_u32);
            MATH_stack_error(ERROR_BASE_MATH);
            if (math_status == MATH_SUCCESS) {
                spsws_ctx.sigfox_ep_ul_payload_monitoring.tmcu_degrees = (uint8_t) generic_u32;
            }
        }
    }
    // PCB temperature.
    spsws_ctx.sigfox_ep_ul_payload_monitoring.tpcb_degrees = SIGFOX_EP_ERROR_VALUE_TEMPERATURE;
    sample_count = (spsws_ctx.measurements.tpcb_degrees.full_flag != 0) ? SPSWS_MEASUREMENT_BUFFER_SIZE : spsws_ctx.measurements.tpcb_degrees.sample_count;
    if (sample_count > 0) {
        // Compute single value.
        math_status = MATH_min(spsws_ctx.measurements.tpcb_degrees.sample_buffer, sample_count, &generic_s32_1);
        MATH_stack_error(ERROR_BASE_MATH);
        if (math_status == MATH_SUCCESS) {
            // Convert temperature.
            math_status = MATH_integer_to_signed_magnitude(generic_s32_1, (MATH_U8_SIZE_BITS - 1), &generic_u32);
            MATH_stack_error(ERROR_BASE_MATH);
            if (math_status == MATH_SUCCESS) {
                spsws_ctx.sigfox_ep_ul_payload_monitoring.tpcb_degrees = (uint8_t) generic_u32;
            }
        }
    }
    // PCB humidity.
    spsws_ctx.sigfox_ep_ul_payload_monitoring.hpcb_percent = SIGFOX_EP_ERROR_VALUE_HUMIDITY;
    sample_count = (spsws_ctx.measurements.hpcb_percent.full_flag != 0) ? SPSWS_MEASUREMENT_BUFFER_SIZE : spsws_ctx.measurements.hpcb_percent.sample_count;
    if (sample_count > 0) {
        // Compute single value.
        math_status = MATH_median_filter(spsws_ctx.measurements.hpcb_percent.sample_buffer, sample_count, 0, &generic_s32_1);
        MATH_stack_error(ERROR_BASE_MATH);
        if (math_status == MATH_SUCCESS) {
            spsws_ctx.sigfox_ep_ul_payload_monitoring.hpcb_percent = (uint8_t) generic_s32_1;
        }
    }
    // Solar cell voltage.
    spsws_ctx.sigfox_ep_ul_payload_monitoring.vsrc_mv = SIGFOX_EP_ERROR_VALUE_ANALOG_16BITS;
    sample_count = (spsws_ctx.measurements.vsrc_mv.full_flag != 0) ? SPSWS_MEASUREMENT_BUFFER_SIZE : spsws_ctx.measurements.vsrc_mv.sample_count;
    if (sample_count > 0) {
        // Compute single value.
        math_status = MATH_median_filter(spsws_ctx.measurements.vsrc_mv.sample_buffer, sample_count, 0, &generic_s32_1);
        MATH_stack_error(ERROR_BASE_MATH);
        if (math_status == MATH_SUCCESS) {
            spsws_ctx.sigfox_ep_ul_payload_monitoring.vsrc_mv = (uint16_t) generic_s32_1;
        }
    }
    // Supercap voltage.
    spsws_ctx.sigfox_ep_ul_payload_monitoring.vcap_mv = SIGFOX_EP_ERROR_VALUE_ANALOG_12BITS;
    sample_count = (spsws_ctx.measurements.vcap_mv.full_flag != 0) ? SPSWS_MEASUREMENT_BUFFER_SIZE : spsws_ctx.measurements.vcap_mv.sample_count;
    if (sample_count > 0) {
        // Select last value.
        spsws_ctx.sigfox_ep_ul_payload_monitoring.vcap_mv = spsws_ctx.measurements.vcap_mv.sample_buffer[spsws_ctx.measurements.vcap_mv.last_sample_index];
    }
    // MCU voltage.
    spsws_ctx.sigfox_ep_ul_payload_monitoring.vmcu_mv = SIGFOX_EP_ERROR_VALUE_ANALOG_12BITS;
    sample_count = (spsws_ctx.measurements.vmcu_mv.full_flag != 0) ? SPSWS_MEASUREMENT_BUFFER_SIZE : spsws_ctx.measurements.vmcu_mv.sample_count;
    if (sample_count > 0) {
        // Compute single value.
        math_status = MATH_median_filter(spsws_ctx.measurements.vmcu_mv.sample_buffer, sample_count, 0, &generic_s32_1);
        MATH_stack_error(ERROR_BASE_MATH);
        if (math_status == MATH_SUCCESS) {
            spsws_ctx.sigfox_ep_ul_payload_monitoring.vmcu_mv = (uint16_t) generic_s32_1;
        }
    }
#ifdef SPSWS_WIND_RAINFALL_MEASUREMENTS
    // Wind speed.
    spsws_ctx.sigfox_ep_ul_payload_weather.wind_speed_average_kmh = SIGFOX_EP_ERROR_VALUE_WIND;
    spsws_ctx.sigfox_ep_ul_payload_weather.wind_speed_peak_kmh = SIGFOX_EP_ERROR_VALUE_WIND;
#ifdef SPSWS_WIND_VANE_ULTIMETER
    ultimeter_status = ULTIMETER_get_wind_speed(&generic_s32_1, &generic_s32_2);
    ULTIMETER_stack_error(ERROR_BASE_ULTIMETER);
    // Check status.
    if (ultimeter_status == ULTIMETER_SUCCESS) {
#else
    sen15901_status = SEN15901_get_wind_speed(&generic_s32_1, &generic_s32_2);
    SEN15901_stack_error(ERROR_BASE_SEN15901);
    // Check status.
    if (sen15901_status == SEN15901_SUCCESS) {
#endif
        spsws_ctx.sigfox_ep_ul_payload_weather.wind_speed_average_kmh = (generic_s32_1 / 1000);
        spsws_ctx.sigfox_ep_ul_payload_weather.wind_speed_peak_kmh = (generic_s32_2 / 1000);
    }
    // Wind direction.
    spsws_ctx.sigfox_ep_ul_payload_weather.wind_direction_average_two_degrees = SIGFOX_EP_ERROR_VALUE_WIND;
#ifdef SPSWS_WIND_VANE_ULTIMETER
    ultimeter_status = ULTIMETER_get_wind_direction(&generic_s32_1, &wind_direction_status);
    ULTIMETER_stack_error(ERROR_BASE_ULTIMETER);
    // Check status.
    if ((ultimeter_status == ULTIMETER_SUCCESS) && (wind_direction_status == ULTIMETER_WIND_DIRECTION_STATUS_AVAILABLE)) {
#else
    sen15901_status = SEN15901_get_wind_direction(&generic_s32_1, &wind_direction_status);
    SEN15901_stack_error(ERROR_BASE_SEN15901);
    // Check status.
    if ((sen15901_status == SEN15901_SUCCESS) && (wind_direction_status == SEN15901_WIND_DIRECTION_STATUS_AVAILABLE)) {
#endif
        spsws_ctx.sigfox_ep_ul_payload_weather.wind_direction_average_two_degrees = (generic_s32_1 >> 1);
    }
    // Rainfall.
    spsws_ctx.sigfox_ep_ul_payload_weather.rainfall_mm = SIGFOX_EP_ERROR_VALUE_RAIN;
    sen15901_status = SEN15901_get_rainfall(&generic_s32_1);
    SEN15901_stack_error(ERROR_BASE_SEN15901);
    // Check status.
    if (sen15901_status == SEN15901_SUCCESS) {
        spsws_ctx.sigfox_ep_ul_payload_weather.rainfall_mm = (generic_s32_1 / 1000);
        // Rounding operation.
        if ((generic_s32_1 - (spsws_ctx.sigfox_ep_ul_payload_weather.rainfall_mm * 1000)) >= 500) {
            spsws_ctx.sigfox_ep_ul_payload_weather.rainfall_mm++;
        }
    }
#endif
}
#endif

/*******************************************************************/
static void _SPSWS_set_clock(uint8_t device_state) {
    // Local variables.
    RCC_status_t rcc_status = RCC_SUCCESS;
#ifndef SPSWS_MODE_CLI
    RCC_clock_t mcu_clock_source = RCC_CLOCK_NONE;
    uint8_t clock_status = 0;
#endif
    // Switch to HSE or HSI depending on state.
    if (device_state == 0) {
        // Switch to internal clock.
        rcc_status = RCC_switch_to_hsi();
        RCC_stack_error(ERROR_BASE_RCC);
        // Turn TCXO off.
        POWER_disable(POWER_REQUESTER_ID_MAIN, POWER_DOMAIN_MCU_TCXO);
    }
    else {
        // Turn TCXO on.
        POWER_enable(POWER_REQUESTER_ID_MAIN, POWER_DOMAIN_MCU_TCXO, LPTIM_DELAY_MODE_SLEEP);
        // Switch to external clock.
        rcc_status = RCC_switch_to_hse(RCC_HSE_MODE_BYPASS);
        RCC_stack_error(ERROR_BASE_RCC);
    }
#ifndef SPSWS_MODE_CLI
    // Update MCU clock source.
    mcu_clock_source = RCC_get_system_clock();
    spsws_ctx.status.mcu_clock_source = (mcu_clock_source == RCC_CLOCK_HSE) ? 0b1 : 0b0;
    // Update LSI status.
    rcc_status = RCC_get_status(RCC_CLOCK_LSI, &clock_status);
    RCC_stack_error(ERROR_BASE_RCC);
    spsws_ctx.status.lsi_status = (clock_status == 0) ? 0b0 : 0b1;
    // Update LSE status.
    rcc_status = RCC_get_status(RCC_CLOCK_LSE, &clock_status);
    RCC_stack_error(ERROR_BASE_RCC);
    spsws_ctx.status.lse_status = (clock_status == 0) ? 0b0 : 0b1;
#endif
}

#ifndef SPSWS_MODE_CLI
/*******************************************************************/
static void _SPSWS_update_additional_requests(void) {
    // Local variables.
    RTC_status_t rtc_status = RTC_SUCCESS;
    NVM_status_t nvm_status = NVM_SUCCESS;
    RTC_time_t current_time;
    RTC_time_t previous_wake_up_time;
    RTC_time_t previous_geoloc_time;
#ifdef SIGFOX_EP_BIDIRECTIONAL
    RTC_time_t previous_downlink_time;
#endif
    uint8_t nvm_byte = 0;
    uint8_t local_utc_offset = 0;
    int8_t local_hour = 0;
    uint8_t is_afternoon = 0;
    // Retrieve current time from RTC.
    rtc_status = RTC_get_time(&current_time);
    RTC_stack_error(ERROR_BASE_RTC);
    // Retrieve last wake-up time from NVM.
    nvm_status = NVM_read_byte((NVM_ADDRESS_LAST_WAKE_UP_YEAR + 0), &nvm_byte);
    NVM_stack_error(ERROR_BASE_NVM);
    previous_wake_up_time.year = (nvm_byte << 8);
    nvm_status = NVM_read_byte((NVM_ADDRESS_LAST_WAKE_UP_YEAR + 1), &nvm_byte);
    NVM_stack_error(ERROR_BASE_NVM);
    previous_wake_up_time.year |= nvm_byte;
    nvm_status = NVM_read_byte(NVM_ADDRESS_LAST_WAKE_UP_MONTH, &(previous_wake_up_time.month));
    NVM_stack_error(ERROR_BASE_NVM);
    nvm_status = NVM_read_byte(NVM_ADDRESS_LAST_WAKE_UP_DATE, &(previous_wake_up_time.date));
    NVM_stack_error(ERROR_BASE_NVM);
    nvm_status = NVM_read_byte(NVM_ADDRESS_LAST_WAKE_UP_HOUR, &(previous_wake_up_time.hours));
    NVM_stack_error(ERROR_BASE_NVM);
    nvm_status = NVM_read_byte(NVM_ADDRESS_LAST_WAKE_UP_MINUTES, &(previous_wake_up_time.minutes));
    NVM_stack_error(ERROR_BASE_NVM);
    // Update last geolocation time and status.
    nvm_status = NVM_read_byte((NVM_ADDRESS_LAST_GEOLOC_YEAR + 0), &nvm_byte);
    NVM_stack_error(ERROR_BASE_NVM);
    previous_geoloc_time.year = (nvm_byte << 8);
    nvm_status = NVM_read_byte((NVM_ADDRESS_LAST_GEOLOC_YEAR + 1), &nvm_byte);
    NVM_stack_error(ERROR_BASE_NVM);
    previous_geoloc_time.year |= nvm_byte;
    nvm_status = NVM_read_byte(NVM_ADDRESS_LAST_GEOLOC_MONTH, &(previous_geoloc_time.month));
    NVM_stack_error(ERROR_BASE_NVM);
    nvm_status = NVM_read_byte(NVM_ADDRESS_LAST_GEOLOC_DATE, &(previous_geoloc_time.date));
    NVM_stack_error(ERROR_BASE_NVM);
    nvm_status = NVM_read_byte(NVM_ADDRESS_LAST_GEOLOC_STATUS, &nvm_byte);
    NVM_stack_error(ERROR_BASE_NVM);
    spsws_ctx.status.daily_geoloc = (nvm_byte & 0x01);
#ifdef SIGFOX_EP_BIDIRECTIONAL
    // Update last downlink time and status.
    nvm_status = NVM_read_byte((NVM_ADDRESS_LAST_DOWNLINK_YEAR + 0), &nvm_byte);
    NVM_stack_error(ERROR_BASE_NVM);
    previous_downlink_time.year = (nvm_byte << 8);
    nvm_status = NVM_read_byte((NVM_ADDRESS_LAST_DOWNLINK_YEAR + 1), &nvm_byte);
    NVM_stack_error(ERROR_BASE_NVM);
    previous_downlink_time.year |= nvm_byte;
    nvm_status = NVM_read_byte(NVM_ADDRESS_LAST_DOWNLINK_MONTH, &(previous_downlink_time.month));
    NVM_stack_error(ERROR_BASE_NVM);
    nvm_status = NVM_read_byte(NVM_ADDRESS_LAST_DOWNLINK_DATE, &(previous_downlink_time.date));
    NVM_stack_error(ERROR_BASE_NVM);
    nvm_status = NVM_read_byte(NVM_ADDRESS_LAST_DOWNLINK_STATUS, &nvm_byte);
    NVM_stack_error(ERROR_BASE_NVM);
    spsws_ctx.status.daily_downlink = (nvm_byte & 0x01);
#endif
    // Reset valid wakeup flag.
    spsws_ctx.flags.valid_wakeup = 0;
    // Check if day has changed
    if ((current_time.year != previous_wake_up_time.year) || (current_time.month != previous_wake_up_time.month) || (current_time.date != previous_wake_up_time.date)) {
        // Day and thus time have changed.
        spsws_ctx.flags.rtc_calibration_request = 1;
        spsws_ctx.flags.valid_wakeup = 1;
    }
    // Check time are different (avoiding false wake-up due to RTC calibration).
    if ((current_time.hours != previous_wake_up_time.hours) || (current_time.minutes != previous_wake_up_time.minutes)) {
        // Time has changed.
        spsws_ctx.flags.valid_wakeup = 1;
    }
    // Compute afternoon flag.
    local_utc_offset = RTC_LOCAL_UTC_OFFSET_WINTER;
    if ((current_time.month > RTC_WINTER_TIME_LAST_MONTH) && (current_time.month < RTC_WINTER_TIME_FIRST_MONTH)) {
        local_utc_offset = RTC_LOCAL_UTC_OFFSET_SUMMER;
    }
    local_hour = (current_time.hours + local_utc_offset) % RTC_NUMBER_OF_HOURS_PER_DAY;
    if (local_hour < 0) {
        local_hour += RTC_NUMBER_OF_HOURS_PER_DAY;
    }
    if (local_hour >= RTC_AFTERNOON_HOUR_THRESHOLD) {
        is_afternoon = 1;
    }
    // Enable device geolocation and error stack message once a day in the afternoon.
    if (((current_time.year != previous_geoloc_time.year) || (current_time.month != previous_geoloc_time.month) || (current_time.date != previous_geoloc_time.date)) && (is_afternoon != 0)) {
        spsws_ctx.flags.geoloc_request = 1;
        spsws_ctx.flags.error_stack_request = 1;
    }
#ifdef SIGFOX_EP_BIDIRECTIONAL
    // Enable downlink transaction once a day in the afternoon.
    if (((current_time.year != previous_downlink_time.year) || (current_time.month != previous_downlink_time.month) || (current_time.date != previous_downlink_time.date)) && (is_afternoon != 0)) {
        spsws_ctx.flags.downlink_request = 1;
    }
#endif
}
#endif

#ifndef SPSWS_MODE_CLI
/*******************************************************************/
static void _SPSWS_update_nvm_data(SPSWS_nvm_data_t timestamp_type) {
    // Local variables.
    NVM_status_t nvm_status = NVM_SUCCESS;
    RTC_status_t rtc_status = RTC_SUCCESS;
    RTC_time_t current_time;
    // Retrieve current time from RTC.
    rtc_status = RTC_get_time(&current_time);
    RTC_stack_error(ERROR_BASE_RTC);
    // Check timestamp type.
    switch (timestamp_type) {
    case SPSWS_NVM_DATA_LAST_WAKE_UP:
        // Update last wake-up time.
        nvm_status = NVM_write_byte((NVM_ADDRESS_LAST_WAKE_UP_YEAR + 0), (uint8_t) (current_time.year >> 8));
        NVM_stack_error(ERROR_BASE_NVM);
        nvm_status = NVM_write_byte((NVM_ADDRESS_LAST_WAKE_UP_YEAR + 1), (uint8_t) (current_time.year >> 0));
        NVM_stack_error(ERROR_BASE_NVM);
        nvm_status = NVM_write_byte(NVM_ADDRESS_LAST_WAKE_UP_MONTH, current_time.month);
        NVM_stack_error(ERROR_BASE_NVM);
        nvm_status = NVM_write_byte(NVM_ADDRESS_LAST_WAKE_UP_DATE, current_time.date);
        NVM_stack_error(ERROR_BASE_NVM);
        nvm_status = NVM_write_byte(NVM_ADDRESS_LAST_WAKE_UP_HOUR, current_time.hours);
        NVM_stack_error(ERROR_BASE_NVM);
        nvm_status = NVM_write_byte(NVM_ADDRESS_LAST_WAKE_UP_MINUTES, current_time.minutes);
        NVM_stack_error(ERROR_BASE_NVM);
        break;
    case SPSWS_NVM_DATA_LAST_GEOLOC:
        // Update last geoloc time and status.
        nvm_status = NVM_write_byte((NVM_ADDRESS_LAST_GEOLOC_YEAR + 0), (uint8_t) (current_time.year >> 8));
        NVM_stack_error(ERROR_BASE_NVM);
        nvm_status = NVM_write_byte((NVM_ADDRESS_LAST_GEOLOC_YEAR + 1), (uint8_t) (current_time.year >> 0));
        NVM_stack_error(ERROR_BASE_NVM);
        nvm_status = NVM_write_byte(NVM_ADDRESS_LAST_GEOLOC_MONTH, current_time.month);
        NVM_stack_error(ERROR_BASE_NVM);
        nvm_status = NVM_write_byte(NVM_ADDRESS_LAST_GEOLOC_DATE, current_time.date);
        NVM_stack_error(ERROR_BASE_NVM);
        nvm_status = NVM_write_byte(NVM_ADDRESS_LAST_GEOLOC_STATUS, (uint8_t) (spsws_ctx.status.daily_geoloc));
        NVM_stack_error(ERROR_BASE_NVM);
        break;
#ifdef SIGFOX_EP_BIDIRECTIONAL
    case SPSWS_NVM_DATA_LAST_DOWNLINK:
        // Update last downlink time and status.
        nvm_status = NVM_write_byte((NVM_ADDRESS_LAST_DOWNLINK_YEAR + 0), (uint8_t) (current_time.year >> 8));
        NVM_stack_error(ERROR_BASE_NVM);
        nvm_status = NVM_write_byte((NVM_ADDRESS_LAST_DOWNLINK_YEAR + 1), (uint8_t) (current_time.year >> 0));
        NVM_stack_error(ERROR_BASE_NVM);
        nvm_status = NVM_write_byte(NVM_ADDRESS_LAST_DOWNLINK_MONTH, current_time.month);
        NVM_stack_error(ERROR_BASE_NVM);
        nvm_status = NVM_write_byte(NVM_ADDRESS_LAST_DOWNLINK_DATE, current_time.date);
        NVM_stack_error(ERROR_BASE_NVM);
        nvm_status = NVM_write_byte(NVM_ADDRESS_LAST_DOWNLINK_STATUS, (uint8_t) (spsws_ctx.status.daily_downlink));
        NVM_stack_error(ERROR_BASE_NVM);
        break;
#endif
    default:
        break;
    }
}
#endif

#ifndef SPSWS_MODE_CLI
/*******************************************************************/
static void _SPSWS_send_sigfox_message(SIGFOX_EP_API_application_message_t* application_message) {
    // Local variables.
    SIGFOX_EP_API_status_t sigfox_ep_api_status = SIGFOX_EP_API_SUCCESS;
    SIGFOX_EP_API_config_t lib_config;
    uint8_t status = 0;
#ifdef SIGFOX_EP_BIDIRECTIONAL
    SIGFOX_EP_API_message_status_t message_status;
    SIGFOX_EP_dl_payload_t dl_payload;
    int16_t dl_rssi = 0;
#endif
    // Directly exit of the radio is disabled due to low supercap voltage.
    if (spsws_ctx.flags.radio_enabled == 0) goto errors;
    // Library configuration.
    lib_config.rc = &SIGFOX_RC1;
    // Open library.
    sigfox_ep_api_status = SIGFOX_EP_API_open(&lib_config);
    SIGFOX_EP_API_check_status(0);
    // Send message.
    sigfox_ep_api_status = SIGFOX_EP_API_send_application_message(application_message);
    SIGFOX_EP_API_check_status(0);
#ifdef SIGFOX_EP_BIDIRECTIONAL
    // Check bidirectional flag.
    if ((application_message->bidirectional_flag) == SIGFOX_TRUE) {
        // Reset status.
        spsws_ctx.status.daily_downlink = 0;
        // Read message status.
        message_status = SIGFOX_EP_API_get_message_status();
        // Check if downlink data is available.
        if (message_status.field.dl_frame != 0) {
            // Update status.
            spsws_ctx.status.daily_downlink = 1;
            // Read downlink payload.
            sigfox_ep_api_status = SIGFOX_EP_API_get_dl_payload(dl_payload.frame, SIGFOX_DL_PAYLOAD_SIZE_BYTES, &dl_rssi);
            SIGFOX_EP_API_check_status(0);
            if (sigfox_ep_api_status == SIGFOX_EP_API_SUCCESS) {
                // Parse payload.
                switch (dl_payload.op_code) {
                case SIGFOX_EP_DL_OP_CODE_NOP:
                    // Nothing to do.
                    break;
                case SIGFOX_EP_DL_OP_CODE_RESET:
                    // Set reset request.
                    spsws_ctx.flags.reset_request = 1;
                    break;
                case SIGFOX_EP_DL_OP_CODE_SET_WEATHER_DATA_PERIOD:
                    // Check and store new configuration.
                    _SPSWS_store_weather_data_period(dl_payload.set_weather_data_period.weather_data_period);
                    break;
                default:
                    ERROR_stack_add(ERROR_DL_OP_CODE);
                    break;
                }
            }
        }
        // Update timestamp and status.
        _SPSWS_update_nvm_data(SPSWS_NVM_DATA_LAST_DOWNLINK);
        // Clear request.
        spsws_ctx.flags.downlink_request = 0;
    }
#endif
    // Close library.
    sigfox_ep_api_status = SIGFOX_EP_API_close();
    SIGFOX_EP_API_check_status(0);
    return;
errors:
    SIGFOX_EP_API_close();
    UNUSED(status);
    return;
}
#endif

#ifndef SPSWS_MODE_CLI
/*******************************************************************/
static void _SPSWS_init_context(void) {
    // Init context.
    spsws_ctx.state = SPSWS_STATE_STARTUP;
    spsws_ctx.flags.all = 0;
    spsws_ctx.flags.rtc_calibration_request = 1;
    spsws_ctx.flags.radio_enabled = 1;
    spsws_ctx.flags.weather_request_enabled = 1;
    spsws_ctx.status.all = 0;
    // Intermediate measurements.
    spsws_ctx.measurements_last_time_seconds = 0;
    _SPSWS_reset_measurements();
#ifdef SIGFOX_EP_BIDIRECTIONAL
    // Weather data.
    spsws_ctx.sharp_hour_uptime = 0;
    spsws_ctx.weather_last_time_seconds = 0;
    spsws_ctx.weather_message_count = 0;
    // Load configuration from NVM.
    _SPSWS_load_weather_data_period();
    _SPSWS_store_weather_data_period(spsws_ctx.weather_data_period);
#endif
    // Init station mode.
#ifdef SPSWS_WIND_RAINFALL_MEASUREMENTS
    spsws_ctx.status.station_mode = 0b1;
#else
    spsws_ctx.status.station_mode = 0b0;
#endif
}
#endif

/*******************************************************************/
static void _SPSWS_init_hw(void) {
    // Local variables.
    RCC_status_t rcc_status = RCC_SUCCESS;
    NVM_status_t nvm_status = NVM_SUCCESS;
    RTC_status_t rtc_status = RTC_SUCCESS;
    LPTIM_status_t lptim_status = LPTIM_SUCCESS;
#ifndef SPSWS_MODE_DEBUG
    IWDG_status_t iwdg_status = IWDG_SUCCESS;
#endif
#if ((defined SPSWS_WIND_RAINFALL_MEASUREMENTS) && !(defined SPSWS_MODE_CLI))
    SEN15901_status_t sen15901_status = SEN15901_SUCCESS;
#ifdef SPSWS_WIND_VANE_ULTIMETER
    ULTIMETER_status_t ultimeter_status = ULTIMETER_SUCCESS;
#endif
#endif
#ifndef SPSWS_MODE_CLI
    RTC_alarm_configuration_t rtc_alarm_config;
#endif
    uint8_t device_id_lsbyte = 0;
    // Init error stack
    ERROR_stack_init();
    // Init memory.
    NVIC_init();
    // Init power module and clock tree.
    PWR_init();
    rcc_status = RCC_init(NVIC_PRIORITY_CLOCK);
    RCC_stack_error(ERROR_BASE_RCC);
    // Init GPIOs.
    GPIO_init();
    POWER_init();
    EXTI_init();
#ifndef SPSWS_MODE_DEBUG
    // Start independent watchdog.
    iwdg_status = IWDG_init();
    IWDG_stack_error(ERROR_BASE_IWDG);
#endif
    // High speed oscillator.
    rcc_status = RCC_switch_to_hsi();
    RCC_stack_error(ERROR_BASE_RCC);
    // Calibrate clocks.
    rcc_status = RCC_calibrate_internal_clocks(NVIC_PRIORITY_CLOCK_CALIBRATION);
    RCC_stack_error(ERROR_BASE_RCC);
    // Init RTC.
#if ((defined SPSWS_WIND_RAINFALL_MEASUREMENTS) && !(defined SPSWS_MODE_CLI))
    rtc_status = RTC_init(&_SPSWS_tick_second_callback, NVIC_PRIORITY_RTC);
#else
    rtc_status = RTC_init(NULL, NVIC_PRIORITY_RTC);
#endif
    RTC_stack_error(ERROR_BASE_RTC);
    // Read LS byte of the device ID to add a random delay in RTC alarm.
    nvm_status = NVM_read_byte((NVM_ADDRESS_SIGFOX_EP_ID + SIGFOX_EP_ID_SIZE_BYTES - 1), &device_id_lsbyte);
    NVM_stack_error(ERROR_BASE_NVM);
#ifndef SPSWS_MODE_CLI
    // Init RTC alarm.
    rtc_alarm_config.mode = RTC_ALARM_MODE_DATE;
    rtc_alarm_config.date.mask = 1;
    rtc_alarm_config.date.value = 0;
    rtc_alarm_config.hours.mask = 1;
    rtc_alarm_config.hours.value = 0;
    rtc_alarm_config.minutes.mask = 0;
    rtc_alarm_config.minutes.value = 0;
    rtc_alarm_config.seconds.mask = 0;
    rtc_alarm_config.seconds.value = (device_id_lsbyte % 60);
    rtc_status = RTC_start_alarm(RTC_ALARM_A, &rtc_alarm_config, &_SPSWS_sharp_hour_alarm_callback);
    RTC_stack_error(ERROR_BASE_RTC);
#endif
    // Init delay timer.
    lptim_status = LPTIM_init(NVIC_PRIORITY_DELAY);
    LPTIM_stack_error(ERROR_BASE_LPTIM);
#if ((defined SPSWS_WIND_RAINFALL_MEASUREMENTS) && !(defined SPSWS_MODE_CLI))
    // Init wind vane and rainfall driver.
    sen15901_status = SEN15901_init(&_SPSWS_sen15901_process_callback);
    SEN15901_stack_error(ERROR_BASE_SEN15901);
#ifdef SPSWS_WIND_VANE_ULTIMETER
    ultimeter_status = ULTIMETER_init(&_SPSWS_ultimeter_process_callback);
    ULTIMETER_stack_error(ERROR_BASE_ULTIMETER);
#endif
    SENSORS_HW_get_wind_tick_second_callback(&spsws_ctx.wind_tick_second_callback);
#endif
    // Init LED pin.
    GPIO_configure(&GPIO_LED, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
#ifdef SPSWS_SEN15901_EMULATOR
    // Init SEN15901 emulator synchronization pin.
    GPIO_configure(&SPSWS_SEN15901_EMULATOR_SYNCHRO_GPIO, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
#endif
}

/*** SPSWS main function ***/

#ifndef SPSWS_MODE_CLI
/*******************************************************************/
int main(void) {
    // Local variables.
    RCC_status_t rcc_status = RCC_SUCCESS;
    RTC_status_t rtc_status = RTC_SUCCESS;
    LPTIM_status_t lptim_status = LPTIM_SUCCESS;
    ANALOG_status_t analog_status = ANALOG_SUCCESS;
    GPS_status_t gps_status = GPS_SUCCESS;
    SHT3X_status_t sht3x_status = SHT3X_SUCCESS;
    DPS310_status_t dps310_status = DPS310_SUCCESS;
    SI1133_status_t si1133_status = SI1133_SUCCESS;
#ifdef SPSWS_WIND_RAINFALL_MEASUREMENTS
    SEN15901_status_t sen15901_status = SEN15901_SUCCESS;
#ifdef SPSWS_WIND_VANE_ULTIMETER
    ULTIMETER_status_t ultimeter_status = ULTIMETER_SUCCESS;
#endif
#endif
    GPS_time_t gps_time;
    RTC_time_t rtc_time;
    GPS_position_t gps_position;
    GPS_acquisition_status_t gps_acquisition_status = GPS_ACQUISITION_SUCCESS;
    SIGFOX_EP_API_application_message_t application_message;
    SIGFOX_EP_ul_payload_startup_t sigfox_ep_ul_payload_startup;
    SIGFOX_EP_ul_payload_geoloc_t sigfox_ep_ul_payload_geoloc;
    SIGFOX_EP_ul_payload_geoloc_timeout_t sigfox_ep_ul_payload_geoloc_timeout;
    ERROR_code_t error_code = 0;
    uint8_t sigfox_ep_ul_payload_error_stack[SIGFOX_EP_UL_PAYLOAD_SIZE_ERROR_STACK];
    uint32_t generic_u32_1 = 0;
#ifdef SIGFOX_EP_BIDIRECTIONAL
    uint32_t generic_u32_2 = 0;
#endif
    int32_t generic_s32_1 = 0;
    int32_t generic_s32_2 = 0;
    uint8_t por_flag = 1;
    uint8_t idx = 0;
    // Init board.
    _SPSWS_init_context();
    _SPSWS_init_hw();
    // Application message default parameters.
    application_message.common_parameters.number_of_frames = 3;
    application_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_100BPS;
    application_message.type = SIGFOX_APPLICATION_MESSAGE_TYPE_BYTE_ARRAY;
    application_message.ul_payload = SIGFOX_NULL;
    application_message.ul_payload_size_bytes = 0;
#ifdef SIGFOX_EP_BIDIRECTIONAL
    application_message.bidirectional_flag = SIGFOX_FALSE;
#endif
    // Main loop.
    while (1) {
        // Reload watchdog.
        IWDG_reload();
        // Perform state machine.
        switch (spsws_ctx.state) {
        case SPSWS_STATE_STARTUP:
            IWDG_reload();
            // Power on delay to wait for main power supply stabilization.
            lptim_status = LPTIM_delay_milliseconds(SPSWS_POWER_ON_DELAY_MS, LPTIM_DELAY_MODE_STOP);
            LPTIM_stack_error(ERROR_BASE_LPTIM);
            // Switch to accurate clock.
            _SPSWS_set_clock(1);
            // Fill reset reason and software version.
            sigfox_ep_ul_payload_startup.reset_reason = PWR_get_reset_flags();
            sigfox_ep_ul_payload_startup.major_version = GIT_MAJOR_VERSION;
            sigfox_ep_ul_payload_startup.minor_version = GIT_MINOR_VERSION;
            sigfox_ep_ul_payload_startup.commit_index = GIT_COMMIT_INDEX;
            sigfox_ep_ul_payload_startup.commit_id = GIT_COMMIT_ID;
            sigfox_ep_ul_payload_startup.dirty_flag = GIT_DIRTY_FLAG;
            // Clear reset flags.
            PWR_clear_reset_flags();
            // Send SW version frame.
            application_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_600BPS;
            application_message.ul_payload = (sfx_u8*) (sigfox_ep_ul_payload_startup.frame);
            application_message.ul_payload_size_bytes = SIGFOX_EP_UL_PAYLOAD_SIZE_STARTUP;
#ifdef SIGFOX_EP_BIDIRECTIONAL
            application_message.bidirectional_flag = SIGFOX_FALSE;
#endif
            _SPSWS_send_sigfox_message(&application_message);
            // Perform first RTC calibration.
            spsws_ctx.state = SPSWS_STATE_RTC_CALIBRATION;
            break;
        case SPSWS_STATE_MEASURE:
            IWDG_reload();
            // Note: digital sensors must also be powered at this step to read the LDR.
            POWER_enable(POWER_REQUESTER_ID_MAIN, POWER_DOMAIN_ANALOG, LPTIM_DELAY_MODE_SLEEP);
            POWER_enable(POWER_REQUESTER_ID_MAIN, POWER_DOMAIN_SENSORS, LPTIM_DELAY_MODE_SLEEP);
            // MCU voltage.
            analog_status = ANALOG_convert_channel(ANALOG_CHANNEL_VMCU_MV, &generic_s32_1);
            ANALOG_stack_error(ERROR_BASE_ANALOG);
            if (analog_status == ANALOG_SUCCESS) {
                _SPSWS_measurement_add_sample(&(spsws_ctx.measurements.vmcu_mv), generic_s32_1);
            }
            // MCU temperature.
            analog_status = ANALOG_convert_channel(ANALOG_CHANNEL_TMCU_DEGREES, &generic_s32_1);
            ANALOG_stack_error(ERROR_BASE_ANALOG);
            if (analog_status == ANALOG_SUCCESS) {
                _SPSWS_measurement_add_sample(&(spsws_ctx.measurements.tmcu_degrees), generic_s32_1);
            }
            // Solar cell voltage.
            analog_status = ANALOG_convert_channel(ANALOG_CHANNEL_VPV_MV, &generic_s32_1);
            ANALOG_stack_error(ERROR_BASE_ANALOG);
            if (analog_status == ANALOG_SUCCESS) {
                _SPSWS_measurement_add_sample(&(spsws_ctx.measurements.vsrc_mv), generic_s32_1);
            }
            // Supercap voltage.
            analog_status = ANALOG_convert_channel(ANALOG_CHANNEL_VCAP_MV, &generic_s32_1);
            ANALOG_stack_error(ERROR_BASE_ANALOG);
            if (analog_status == ANALOG_SUCCESS) {
                _SPSWS_measurement_add_sample(&(spsws_ctx.measurements.vcap_mv), generic_s32_1);
                // Voltage hysteresis for radio.
                if (generic_s32_1 < SPSWS_RADIO_OFF_VCAP_THRESHOLD_MV) {
                    spsws_ctx.flags.radio_enabled = 0;
                }
                if (generic_s32_1 > SPSWS_RADIO_ON_VCAP_THRESHOLD_MV) {
                    spsws_ctx.flags.radio_enabled = 1;
                }
                // Voltage hysteresis for uplink period.
                if (generic_s32_1 < SPSWS_WEATHER_REQUEST_OFF_VCAP_THRESHOLD_MV) {
                    spsws_ctx.flags.weather_request_enabled = 0;
                }
                if (generic_s32_1 > SPSWS_WEATHER_REQUEST_ON_VCAP_THRESHOLD_MV) {
                    spsws_ctx.flags.weather_request_enabled = 1;
                }
            }
            // Light sensor.
            analog_status = ANALOG_convert_channel(ANALOG_CHANNEL_LDR_PERCENT, &generic_s32_1);
            ANALOG_stack_error(ERROR_BASE_ANALOG);
            if (analog_status == ANALOG_SUCCESS) {
                _SPSWS_measurement_add_sample(&(spsws_ctx.measurements.light_percent), generic_s32_1);
            }
            POWER_disable(POWER_REQUESTER_ID_MAIN, POWER_DOMAIN_ANALOG);
            // Internal temperature/humidity sensor.
            sht3x_status = SHT3X_get_temperature_humidity(I2C_ADDRESS_SHT30_INTERNAL, &generic_s32_1, &generic_s32_2);
            SHT3X_stack_error(ERROR_BASE_SHT30_INTERNAL);
            // Check status.
            if (sht3x_status == SHT3X_SUCCESS) {
                // Store temperature and humidity.
                _SPSWS_measurement_add_sample(&(spsws_ctx.measurements.tpcb_degrees), (generic_s32_1 / 10));
                _SPSWS_measurement_add_sample(&(spsws_ctx.measurements.hpcb_percent), generic_s32_2);
#ifdef HW1_0
                _SPSWS_measurement_add_sample(&(spsws_ctx.measurements.tamb_degrees), (generic_s32_1 / 10));
                _SPSWS_measurement_add_sample(&(spsws_ctx.measurements.hamb_percent), generic_s32_2);
#endif
            }
#ifdef HW2_0
            // External temperature/humidity sensor.
            sht3x_status = SHT3X_get_temperature_humidity(I2C_ADDRESS_SHT30_EXTERNAL, &generic_s32_1, &generic_s32_2);
            SHT3X_stack_error(ERROR_BASE_SHT30_EXTERNAL);
            // Check status.
            if (sht3x_status == SHT3X_SUCCESS) {
                // Store temperature and humidity.
                _SPSWS_measurement_add_sample(&(spsws_ctx.measurements.tamb_degrees), (generic_s32_1 / 10));
                _SPSWS_measurement_add_sample(&(spsws_ctx.measurements.hamb_percent), generic_s32_2);
            }
#endif
            // External pressure and temperature sensor.
            dps310_status = DPS310_get_pressure_temperature(I2C_ADDRESS_DPS310, &generic_s32_1, &generic_s32_2);
            DPS310_stack_error(ERROR_BASE_DPS310);
            // Check status.
            if (dps310_status == DPS310_SUCCESS) {
                // Store pressure.
                _SPSWS_measurement_add_sample(&(spsws_ctx.measurements.patm_abs_pa), generic_s32_1);
            }
            // External UV index sensor.
            si1133_status = SI1133_get_uv_index(I2C_ADDRESS_SI1133, &generic_s32_1);
            SI1133_stack_error(ERROR_BASE_SI1133);
            // Check status.
            if (si1133_status == SI1133_SUCCESS) {
                // Store UV index.
                _SPSWS_measurement_add_sample(&(spsws_ctx.measurements.uv_index), generic_s32_1);
            }
            POWER_disable(POWER_REQUESTER_ID_MAIN, POWER_DOMAIN_SENSORS);
            // Clear flag.
            spsws_ctx.flags.measure_request = 0;
            // Go to off state.
            spsws_ctx.state = SPSWS_STATE_TASK_CHECK;
            break;
        case SPSWS_STATE_WEATHER:
            IWDG_reload();
            // Compute average data.
            _SPSWS_compute_final_measurements();
            _SPSWS_reset_measurements();
#ifdef SPSWS_SEN15901_EMULATOR
            // Synchronize emulator on weather data message transmission.
            GPIO_write(&SPSWS_SEN15901_EMULATOR_SYNCHRO_GPIO, 1);
#endif
            // Send uplink weather frame.
#ifdef SIGFOX_EP_BIDIRECTIONAL
            application_message.common_parameters.ul_bit_rate = ((spsws_ctx.flags.weather_request_intermediate == 0) ? SIGFOX_UL_BIT_RATE_100BPS : SIGFOX_UL_BIT_RATE_600BPS);
#else
            application_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_100BPS;
#endif
            application_message.ul_payload = (sfx_u8*) (spsws_ctx.sigfox_ep_ul_payload_weather.frame);
            application_message.ul_payload_size_bytes = SIGFOX_EP_UL_PAYLOAD_SIZE_WEATHER;
#ifdef SIGFOX_EP_BIDIRECTIONAL
            application_message.bidirectional_flag = (spsws_ctx.flags.downlink_request == 0) ? SIGFOX_FALSE : SIGFOX_TRUE;
#endif
            _SPSWS_send_sigfox_message(&application_message);
#ifdef SPSWS_SEN15901_EMULATOR
            GPIO_write(&SPSWS_SEN15901_EMULATOR_SYNCHRO_GPIO, 0);
#endif
            // Compute next state.
            spsws_ctx.state = SPSWS_STATE_MONITORING;
            break;
        case SPSWS_STATE_MONITORING:
            // Check request.
            if (spsws_ctx.flags.monitoring_request != 0) {
                // Read status byte.
                spsws_ctx.sigfox_ep_ul_payload_monitoring.status = spsws_ctx.status.all;
                // Send uplink monitoring frame.
                application_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_600BPS;
                application_message.ul_payload = (sfx_u8*) (spsws_ctx.sigfox_ep_ul_payload_monitoring.frame);
                application_message.ul_payload_size_bytes = SIGFOX_EP_UL_PAYLOAD_SIZE_MONITORING;
#ifdef SIGFOX_EP_BIDIRECTIONAL
                application_message.bidirectional_flag = SIGFOX_FALSE;
#endif
                _SPSWS_send_sigfox_message(&application_message);
                // Clear request.
                spsws_ctx.flags.monitoring_request = 0;
            }
            // Compute next state.
            spsws_ctx.state = SPSWS_STATE_GEOLOC;
            break;
        case SPSWS_STATE_GEOLOC:
            IWDG_reload();
            // Check request flag.
            if (spsws_ctx.flags.geoloc_request != 0) {
                // Reset status to default.
                spsws_ctx.status.daily_geoloc = 0;
                // Turn GPS on.
                POWER_enable(POWER_REQUESTER_ID_MAIN, POWER_DOMAIN_GPS, LPTIM_DELAY_MODE_SLEEP);
                // Get geolocation from GPS.
                gps_status = GPS_get_position(&gps_position, SPSWS_GEOLOC_TIMEOUT_SECONDS, &generic_u32_1, &gps_acquisition_status);
                GPS_stack_error(ERROR_BASE_GPS);
                // Turn GPS off.
                POWER_disable(POWER_REQUESTER_ID_MAIN, POWER_DOMAIN_GPS);
                // Build Sigfox frame.
                if (gps_acquisition_status == GPS_ACQUISITION_SUCCESS) {
                    sigfox_ep_ul_payload_geoloc.latitude_degrees = gps_position.lat_degrees;
                    sigfox_ep_ul_payload_geoloc.latitude_minutes = gps_position.lat_minutes;
                    sigfox_ep_ul_payload_geoloc.latitude_seconds = gps_position.lat_seconds;
                    sigfox_ep_ul_payload_geoloc.latitude_north_flag = gps_position.lat_north_flag;
                    sigfox_ep_ul_payload_geoloc.longitude_degrees = gps_position.long_degrees;
                    sigfox_ep_ul_payload_geoloc.longitude_minutes = gps_position.long_minutes;
                    sigfox_ep_ul_payload_geoloc.longitude_seconds = gps_position.long_seconds;
                    sigfox_ep_ul_payload_geoloc.longitude_east_flag = gps_position.long_east_flag;
                    sigfox_ep_ul_payload_geoloc.altitude_meters = gps_position.altitude;
                    sigfox_ep_ul_payload_geoloc.gps_acquisition_duration_seconds = generic_u32_1;
                    // Update message parameters.
                    application_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_100BPS;
                    application_message.ul_payload = (sfx_u8*) (sigfox_ep_ul_payload_geoloc.frame);
                    application_message.ul_payload_size_bytes = SIGFOX_EP_UL_PAYLOAD_SIZE_GEOLOC;
                    // Update status bit.
                    spsws_ctx.status.daily_geoloc = 1;
                }
                else {
                    sigfox_ep_ul_payload_geoloc_timeout.gps_acquisition_status = gps_acquisition_status;
                    sigfox_ep_ul_payload_geoloc_timeout.gps_acquisition_duration_seconds = generic_u32_1;
                    // Update message parameters.
                    application_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_100BPS;
                    application_message.ul_payload = (sfx_u8*) (sigfox_ep_ul_payload_geoloc_timeout.frame);
                    application_message.ul_payload_size_bytes = SIGFOX_EP_UL_PAYLOAD_SIZE_GEOLOC_TIMEOUT;
                }
#ifdef SIGFOX_EP_BIDIRECTIONAL
                application_message.bidirectional_flag = SIGFOX_FALSE;
#endif
                // Send uplink geolocation frame.
                _SPSWS_send_sigfox_message(&application_message);
                // Update timestamp and status.
                _SPSWS_update_nvm_data(SPSWS_NVM_DATA_LAST_GEOLOC);
                // Clear request.
                spsws_ctx.flags.geoloc_request = 0;
            }
            // Send error stack frame.
            spsws_ctx.state = SPSWS_STATE_ERROR_STACK;
            break;
        case SPSWS_STATE_ERROR_STACK:
            IWDG_reload();
            // Check request flag.
            if (spsws_ctx.flags.error_stack_request != 0) {
                // Import Sigfox library error stack.
                ERROR_import_sigfox_stack();
                // Check stack.
                if (ERROR_stack_is_empty() == 0) {
                    // Read error stack.
                    for (idx = 0; idx < (SIGFOX_EP_UL_PAYLOAD_SIZE_ERROR_STACK >> 1); idx++) {
                        error_code = ERROR_stack_read();
                        sigfox_ep_ul_payload_error_stack[(idx << 1) + 0] = (uint8_t) ((error_code >> 8) & 0x00FF);
                        sigfox_ep_ul_payload_error_stack[(idx << 1) + 1] = (uint8_t) ((error_code >> 0) & 0x00FF);
                    }
                    // Send frame.
                    application_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_600BPS;
                    application_message.ul_payload = (sfx_u8*) (sigfox_ep_ul_payload_error_stack);
                    application_message.ul_payload_size_bytes = SIGFOX_EP_UL_PAYLOAD_SIZE_ERROR_STACK;
#ifdef SIGFOX_EP_BIDIRECTIONAL
                    application_message.bidirectional_flag = SIGFOX_FALSE;
#endif
                    _SPSWS_send_sigfox_message(&application_message);
                    // Reset error stack.
                    ERROR_stack_init();
                }
                // Clear request.
                spsws_ctx.flags.error_stack_request = 0;
            }
            // Enter sleep mode.
            spsws_ctx.state = SPSWS_STATE_RTC_CALIBRATION;
            break;
        case SPSWS_STATE_RTC_CALIBRATION:
            IWDG_reload();
            // Check request flag.
            if (spsws_ctx.flags.rtc_calibration_request != 0) {
                // Reset status to default.
                spsws_ctx.status.daily_rtc_calibration = 0;
                // Turn GPS on.
                POWER_enable(POWER_REQUESTER_ID_MAIN, POWER_DOMAIN_GPS, LPTIM_DELAY_MODE_SLEEP);
                // Get current time from GPS.
                gps_status = GPS_get_time(&gps_time, SPSWS_RTC_CALIBRATION_TIMEOUT_SECONDS, &generic_u32_1, &gps_acquisition_status);
                GPS_stack_error(ERROR_BASE_GPS);
                // Turn GPS off.
                POWER_disable(POWER_REQUESTER_ID_MAIN, POWER_DOMAIN_GPS);
                // Calibrate RTC if time is available.
                if (gps_acquisition_status == GPS_ACQUISITION_SUCCESS) {
                    // Copy structure.
                    rtc_time.year = gps_time.year;
                    rtc_time.month = gps_time.month;
                    rtc_time.date = gps_time.date;
                    rtc_time.hours = gps_time.hours;
                    rtc_time.minutes = gps_time.minutes;
                    rtc_time.seconds = gps_time.seconds;
                    // Update RTC registers.
                    rtc_status = RTC_set_time(&rtc_time);
                    RTC_stack_error(ERROR_BASE_RTC);
                    // Update status bit.
                    if (rtc_status == RTC_SUCCESS) {
                        // Update timestamp in case of first calibration.
                        if (spsws_ctx.status.first_rtc_calibration == 0) {
                            _SPSWS_update_nvm_data(SPSWS_NVM_DATA_LAST_WAKE_UP);
                        }
                        // Update status.
                        spsws_ctx.status.first_rtc_calibration = 1;
                        spsws_ctx.status.daily_rtc_calibration = 1;
                    }
                }
                if (por_flag != 0) {
                    // In POR condition, RTC alarm will occur during the first GPS time acquisition because of the RTC reset and the random delay.
                    // Flags are manually cleared to avoid wake-up directly after the first RTC calibration.
                    spsws_ctx.flags.first_sharp_hour_alarm = 0;
                    spsws_ctx.flags.sharp_hour_alarm = 0;
                    spsws_ctx.flags.weather_request = 0;
                    spsws_ctx.flags.measure_request = 0;
                }
                por_flag = 0;
                // Clear request.
                spsws_ctx.flags.rtc_calibration_request = 0;
            }
            // Update flags.
            spsws_ctx.state = SPSWS_STATE_TASK_END;
            break;
        case SPSWS_STATE_TASK_END:
            IWDG_reload();
            // Switch to internal clock.
            _SPSWS_set_clock(0);
#ifdef SPSWS_WIND_RAINFALL_MEASUREMENTS
#ifdef SPSWS_WIND_VANE_ULTIMETER
            ultimeter_status = ULTIMETER_set_wind_measurement(1);
            ULTIMETER_stack_error(ERROR_BASE_ULTIMETER);
#else
            sen15901_status = SEN15901_set_wind_measurement(1);
            SEN15901_stack_error(ERROR_BASE_SEN15901);
#endif
            sen15901_status = SEN15901_set_rainfall_measurement(1);
            SEN15901_stack_error(ERROR_BASE_SEN15901);
#endif
            // Enter sleep mode.
            spsws_ctx.state = SPSWS_STATE_TASK_CHECK;
            break;
        case SPSWS_STATE_TASK_CHECK:
            IWDG_reload();
            // Read uptime.
            generic_u32_1 = RTC_get_uptime_seconds();
#ifdef SPSWS_WIND_RAINFALL_MEASUREMENTS
            // Check wind driver process flag.
#ifdef SPSWS_WIND_VANE_ULTIMETER
            if (spsws_ctx.flags.ultimeter_process != 0) {
                // Clear flag.
                spsws_ctx.flags.ultimeter_process = 0;
                // Process driver.
                ultimeter_status = ULTIMETER_process();
                ULTIMETER_stack_error(ERROR_BASE_ULTIMETER);
            }
#else
            if (spsws_ctx.flags.sen15901_process != 0) {
                // Clear flag.
                spsws_ctx.flags.sen15901_process = 0;
                // Process driver.
                sen15901_status = SEN15901_process();
                SEN15901_stack_error(ERROR_BASE_SEN15901);
            }
#endif
#endif
            // Check measurements period.
            if (generic_u32_1 >= (spsws_ctx.measurements_last_time_seconds + SPSWS_MEASUREMENT_PERIOD_SECONDS)) {
                // Set request and update last time.
                spsws_ctx.flags.measure_request = 1;
                spsws_ctx.measurements_last_time_seconds = generic_u32_1;
            }
#ifdef SIGFOX_EP_BIDIRECTIONAL
            // Synchronize weather period.
            if (spsws_ctx.flags.sharp_hour_alarm != 0) {
                // Clear flag.
                spsws_ctx.flags.sharp_hour_alarm = 0;
                // Set requests and update last time.
                spsws_ctx.flags.monitoring_request = 1;
                spsws_ctx.flags.weather_request = 1;
                spsws_ctx.flags.weather_request_intermediate = 0;
                spsws_ctx.weather_last_time_seconds = spsws_ctx.sharp_hour_uptime;
                // Reset message count.
                spsws_ctx.weather_message_count = 1;
            }
            else if (spsws_ctx.flags.first_sharp_hour_alarm != 0) {
                // Get current period.
                generic_u32_2 = SPSWS_WEATHER_DATA_PERIOD_SECONDS[spsws_ctx.weather_data_period];
                // Check weather period.
                if ((generic_u32_1 >= (spsws_ctx.weather_last_time_seconds + generic_u32_2)) && (spsws_ctx.weather_message_count < (3600 / generic_u32_2))) {
                    // Set request and update last time.
                    spsws_ctx.flags.weather_request = spsws_ctx.flags.weather_request_enabled;
                    spsws_ctx.flags.weather_request_intermediate = 1;
                    spsws_ctx.weather_last_time_seconds = generic_u32_1;
                    // Update message count.
                    spsws_ctx.weather_message_count++;
                }
            }
#endif
            // Go to sleep by default.
            spsws_ctx.state = SPSWS_STATE_SLEEP;
            // Check wake-up flags.
            if (spsws_ctx.flags.weather_request != 0) {
                // Clear flag directly because wakeup is not guaranteed.
                spsws_ctx.flags.weather_request = 0;
                // Update requests.
                _SPSWS_update_additional_requests();
                // Check hour change flag.
                if (spsws_ctx.flags.valid_wakeup != 0) {
#ifdef SPSWS_WIND_RAINFALL_MEASUREMENTS
#ifdef SPSWS_WIND_VANE_ULTIMETER
                    ultimeter_status = ULTIMETER_set_wind_measurement(0);
                    ULTIMETER_stack_error(ERROR_BASE_ULTIMETER);
#else
                    sen15901_status = SEN15901_set_wind_measurement(0);
                    SEN15901_stack_error(ERROR_BASE_SEN15901);
#endif
                    sen15901_status = SEN15901_set_rainfall_measurement(0);
                    SEN15901_stack_error(ERROR_BASE_SEN15901);
#endif
                    // Update last wakeup time.
                    _SPSWS_update_nvm_data(SPSWS_NVM_DATA_LAST_WAKE_UP);
                    // Calibrate clocks.
                    rcc_status = RCC_calibrate_internal_clocks(NVIC_PRIORITY_CLOCK_CALIBRATION);
                    RCC_stack_error(ERROR_BASE_RCC);
                    // Switch to accurate clock.
                    _SPSWS_set_clock(1);
                    // Compute next state
                    spsws_ctx.state = SPSWS_STATE_WEATHER;
                }
            }
            else if (spsws_ctx.flags.measure_request != 0) {
                // Compute next state
                spsws_ctx.state = SPSWS_STATE_MEASURE;
            }
            break;
        case SPSWS_STATE_SLEEP:
#ifdef SIGFOX_EP_BIDIRECTIONAL
            // Check reset request.
            if (spsws_ctx.flags.reset_request != 0) {
                PWR_software_reset();
            }
#endif
            // Enter sleep mode.
            IWDG_reload();
            PWR_enter_deepsleep_mode(PWR_DEEPSLEEP_MODE_STOP);
            IWDG_reload();
            // Check wake-up reason.
            spsws_ctx.state = SPSWS_STATE_TASK_CHECK;
            break;
        default:
            // Enter standby mode.
            spsws_ctx.state = SPSWS_STATE_TASK_END;
            break;
        }
    }
    return 0;
}
#endif

#ifdef SPSWS_MODE_CLI
/*******************************************************************/
int main (void) {
    // Local variables.
    CLI_status_t cli_status = CLI_SUCCESS;
    // Init board.
    _SPSWS_init_hw();
    // Switch to accurate clock.
    _SPSWS_set_clock(1);
    // Init command line interface.
    cli_status = CLI_init();
    CLI_stack_error(ERROR_BASE_CLI);
    // Main loop.
    while (1) {
        // Enter sleep mode.
        IWDG_reload();
        PWR_enter_sleep_mode(PWR_SLEEP_MODE_NORMAL);
        IWDG_reload();
        // Process command line interface.
        cli_status = CLI_process();
        CLI_stack_error(ERROR_BASE_CLI);
    }
    return 0;
}
#endif
