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
#include "sigfox_ep_api.h"
#include "sigfox_rc.h"
#include "sigfox_types.h"
// Applicative.
#include "error_base.h"
#include "spsws_flags.h"
#include "version.h"

/*** SPSWS macros ***/

// Timing.
#define SPSWS_POWER_ON_DELAY_MS                     7000
#define SPSWS_RTC_CALIBRATION_TIMEOUT_SECONDS       180
#define SPSWS_GEOLOC_TIMEOUT_SECONDS                120
// Voltage hysteresis for radio.
#define SPSWS_RADIO_OFF_VCAP_THRESHOLD_MV           1000
#define SPSWS_RADIO_ON_VCAP_THRESHOLD_MV            1500
// Sigfox UL payloads size.
#define SPSWS_SIGFOX_STARTUP_DATA_SIZE              8
#define SPSWS_SIGFOX_ERROR_DATA_SIZE                12
#ifdef SPSWS_WIND_RAINFALL_MEASUREMENTS
#define SPSWS_SIGFOX_WEATHER_DATA_SIZE              10
#else
#define SPSWS_SIGFOX_WEATHER_DATA_SIZE              6
#endif
#define SPSWS_SIGFOX_MONITORING_DATA_SIZE           9
#define SPSWS_SIGFOX_GEOLOC_DATA_SIZE               11
#define SPSWS_SIGFOX_GEOLOC_TIMEOUT_DATA_SIZE       2
#define SPSWS_SIGFOX_ERROR_STACK_DATA_SIZE          12
// Error values.
#define SPSWS_ERROR_VALUE_ANALOG_12BITS             0xFFF
#define SPSWS_ERROR_VALUE_ANALOG_16BITS             0xFFFF
#define SPSWS_ERROR_VALUE_LIGHT                     0xFF
#define SPSWS_ERROR_VALUE_TEMPERATURE               0x7F
#define SPSWS_ERROR_VALUE_HUMIDITY                  0xFF
#define SPSWS_ERROR_VALUE_UV_INDEX                  0xFF
#define SPSWS_ERROR_VALUE_PRESSURE                  0xFFFF
#define SPSWS_ERROR_VALUE_WIND                      0xFF
#define SPSWS_ERROR_VALUE_RAIN                      0xFF
// Measurements buffers length.
#define SPSWS_MEASUREMENT_PERIOD_SECONDS            60
#define SPSWS_MEASUREMENT_BUFFER_SIZE               (3600 / SPSWS_MEASUREMENT_PERIOD_SECONDS)
#ifdef SPSWS_SEN15901_EMULATOR
#define SPSWS_SEN15901_EMULATOR_SYNCHRO_GPIO        GPIO_DIO4
#endif

/*** SPSWS structures ***/

/*******************************************************************/
typedef enum {
    SPSWS_STATE_STARTUP,
    SPSWS_STATE_WAKEUP,
    SPSWS_STATE_MEASURE,
    SPSWS_STATE_MONITORING,
    SPSWS_STATE_WEATHER_DATA,
    SPSWS_STATE_GEOLOC,
    SPSWS_STATE_RTC_CALIBRATION,
    SPSWS_STATE_ERROR_STACK,
    SPSWS_STATE_OFF,
    SPSWS_STATE_SLEEP,
    SPSWS_STATE_LAST
} SPSWS_state_t;

/*******************************************************************/
typedef union {
    struct {
        unsigned daily_downlink :1;
        unsigned daily_geoloc :1;
        unsigned daily_rtc_calibration :1;
        unsigned first_rtc_calibration :1;
        unsigned lse_status :1;
        unsigned lsi_status :1;
        unsigned mcu_clock_source :1;
        unsigned station_mode :1;
    } __attribute__((scalar_storage_order("big-endian")))__attribute__((packed));
    uint8_t all;
} SPSWS_status_t;

/*******************************************************************/
typedef union {
    struct {
#ifdef SPSWS_WIND_RAINFALL_MEASUREMENTS
#ifdef SPSWS_WIND_VANE_ULTIMETER
        unsigned ultimeter_process :1;
#endif
        unsigned sen15901_process :1;
#endif
        unsigned radio_enabled : 1;
        unsigned daily_geoloc_done : 1;
        unsigned daily_rtc_calibration_done : 1;
        unsigned fixed_hour_alarm :1;
        unsigned wake_up :1;
        unsigned is_afternoon :1;
        unsigned day_changed :1;
        unsigned hour_changed :1;
        unsigned por :1;
    } __attribute__((scalar_storage_order("big-endian")))__attribute__((packed));
    uint16_t all;
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
typedef union {
    uint8_t frame[SPSWS_SIGFOX_STARTUP_DATA_SIZE];
    struct {
        unsigned reset_reason :8;
        unsigned major_version :8;
        unsigned minor_version :8;
        unsigned commit_index :8;
        unsigned commit_id :28;
        unsigned dirty_flag :4;
    } __attribute__((scalar_storage_order("big-endian")))__attribute__((packed));
} SPSWS_sigfox_startup_data_t;

/*******************************************************************/
typedef union {
    uint8_t frame[SPSWS_SIGFOX_WEATHER_DATA_SIZE];
    struct {
        unsigned tamb_degrees :8;
        unsigned hamb_percent :8;
        unsigned light_percent :8;
        unsigned uv_index :8;
        unsigned patm_abs_tenth_hpa :16;
#ifdef SPSWS_WIND_RAINFALL_MEASUREMENTS
        unsigned wind_speed_average_kmh :8;
        unsigned wind_speed_peak_kmh :8;
        unsigned wind_direction_average_two_degrees :8;
        unsigned rainfall_mm :8;
#endif
    } __attribute__((scalar_storage_order("big-endian")))__attribute__((packed));
} SPSWS_sigfox_weather_data_t;

/*******************************************************************/
typedef union {
    uint8_t frame[SPSWS_SIGFOX_MONITORING_DATA_SIZE];
    struct {
        unsigned tmcu_degrees :8;
        unsigned tpcb_degrees :8;
        unsigned hpcb_percent :8;
        unsigned vsrc_mv :16;
        unsigned vcap_mv :12;
        unsigned vmcu_mv :12;
        unsigned status :8;
    } __attribute__((scalar_storage_order("big-endian")))__attribute__((packed));
} SPSWS_sigfox_monitoring_data_t;

/*******************************************************************/
typedef union {
    uint8_t frame[SPSWS_SIGFOX_GEOLOC_DATA_SIZE];
    struct {
        unsigned latitude_degrees :8;
        unsigned latitude_minutes :6;
        unsigned latitude_seconds :17;
        unsigned latitude_north_flag :1;
        unsigned longitude_degrees :8;
        unsigned longitude_minutes :6;
        unsigned longitude_seconds :17;
        unsigned longitude_east_flag :1;
        unsigned altitude_meters :16;
        unsigned gps_acquisition_duration_seconds :8;
    } __attribute__((scalar_storage_order("big-endian")))__attribute__((packed));
} SPSWS_sigfox_geoloc_data_t;

/*******************************************************************/
typedef union {
    uint8_t frame[SPSWS_SIGFOX_GEOLOC_TIMEOUT_DATA_SIZE];
    struct {
        unsigned gps_acquisition_status :8;
        unsigned gps_acquisition_duration_seconds :8;
    } __attribute__((scalar_storage_order("big-endian")))__attribute__((packed));
} SPSWS_sigfox_geoloc_timeout_data_t;

/*******************************************************************/
typedef struct {
    // Global.
    SPSWS_state_t state;
    SPSWS_status_t status;
    volatile SPSWS_flags_t flags;
    volatile uint32_t seconds_counter;
#ifdef SPSWS_WIND_RAINFALL_MEASUREMENTS
    SENSORS_HW_wind_tick_second_irq_cb_t wind_tick_second_callback;
#endif
#ifndef SPSWS_MODE_CLI
    // Wake-up management.
    RTC_time_t current_time;
    RTC_time_t previous_wake_up_time;
    // Measurements buffers.
    SPSWS_measurements_t measurements;
    // Sigfox frames.
    SPSWS_sigfox_startup_data_t sigfox_startup_data;
    SPSWS_sigfox_monitoring_data_t sigfox_monitoring_data;
    SPSWS_sigfox_weather_data_t sigfox_weather_data;
    SPSWS_sigfox_geoloc_data_t sigfox_geoloc_data;
    SPSWS_sigfox_geoloc_timeout_data_t sigfox_geoloc_timeout_data;
    // Error stack.
    uint8_t sigfox_error_stack_data[SPSWS_SIGFOX_ERROR_STACK_DATA_SIZE];
#endif
} SPSWS_context_t;

/*** SPSWS global variables ***/

static SPSWS_context_t spsws_ctx;

/*** SPSWS local functions ***/

/*******************************************************************/
static void _SPSWS_tick_second_callback(void) {
    // Update second counter.
    spsws_ctx.seconds_counter++;
#ifdef SPSWS_WIND_RAINFALL_MEASUREMENTS
    if (spsws_ctx.wind_tick_second_callback != NULL) {
        spsws_ctx.wind_tick_second_callback();
    }
#endif
}

/*******************************************************************/
static void _SPSWS_fixed_hour_alarm_callback(void) {
    // Update local flags.
    spsws_ctx.flags.fixed_hour_alarm = 1;
}

#ifdef SPSWS_WIND_RAINFALL_MEASUREMENTS
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
    spsws_ctx.sigfox_weather_data.tamb_degrees = SPSWS_ERROR_VALUE_TEMPERATURE;
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
                spsws_ctx.sigfox_weather_data.tamb_degrees = (uint8_t) generic_u32;
            }
        }
    }
    // Humidity.
    spsws_ctx.sigfox_weather_data.hamb_percent = SPSWS_ERROR_VALUE_HUMIDITY;
    sample_count = (spsws_ctx.measurements.hamb_percent.full_flag != 0) ? SPSWS_MEASUREMENT_BUFFER_SIZE : spsws_ctx.measurements.hamb_percent.sample_count;
    if (sample_count > 0) {
        // Compute single value.
        math_status = MATH_median_filter(spsws_ctx.measurements.hamb_percent.sample_buffer, sample_count, 0, &generic_s32_1);
        MATH_stack_error(ERROR_BASE_MATH);
        if (math_status == MATH_SUCCESS) {
            spsws_ctx.sigfox_weather_data.hamb_percent = (uint8_t) generic_s32_1;
        }
    }
    // Light.
    spsws_ctx.sigfox_weather_data.light_percent = SPSWS_ERROR_VALUE_LIGHT;
    sample_count = (spsws_ctx.measurements.light_percent.full_flag != 0) ? SPSWS_MEASUREMENT_BUFFER_SIZE : spsws_ctx.measurements.light_percent.sample_count;
    if (sample_count > 0) {
        // Compute single value.
        math_status = MATH_median_filter(spsws_ctx.measurements.light_percent.sample_buffer, sample_count, 0, &generic_s32_1);
        MATH_stack_error(ERROR_BASE_MATH);
        if (math_status == MATH_SUCCESS) {
            spsws_ctx.sigfox_weather_data.light_percent = (uint8_t) generic_s32_1;
        }
    }
    // UV index.
    spsws_ctx.sigfox_weather_data.uv_index = SPSWS_ERROR_VALUE_UV_INDEX;
    sample_count = (spsws_ctx.measurements.uv_index.full_flag != 0) ? SPSWS_MEASUREMENT_BUFFER_SIZE : spsws_ctx.measurements.uv_index.sample_count;
    if (sample_count > 0) {
        // Compute single value.
        math_status = MATH_max(spsws_ctx.measurements.uv_index.sample_buffer, sample_count, &generic_s32_1);
        MATH_stack_error(ERROR_BASE_MATH);
        if (math_status == MATH_SUCCESS) {
            spsws_ctx.sigfox_weather_data.uv_index = (uint8_t) generic_s32_1;
        }
    }
    // Absolute pressure.
    spsws_ctx.sigfox_weather_data.patm_abs_tenth_hpa = SPSWS_ERROR_VALUE_PRESSURE;
    sample_count = (spsws_ctx.measurements.patm_abs_pa.full_flag != 0) ? SPSWS_MEASUREMENT_BUFFER_SIZE : spsws_ctx.measurements.patm_abs_pa.sample_count;
    if (sample_count > 0) {
        // Compute single value.
        math_status = MATH_median_filter(spsws_ctx.measurements.patm_abs_pa.sample_buffer, sample_count, 0, &generic_s32_1);
        MATH_stack_error(ERROR_BASE_MATH);
        if (math_status == MATH_SUCCESS) {
            spsws_ctx.sigfox_weather_data.patm_abs_tenth_hpa = (uint16_t) (generic_s32_1 / 10);
        }
    }
    // MCU temperature.
    spsws_ctx.sigfox_monitoring_data.tmcu_degrees = SPSWS_ERROR_VALUE_TEMPERATURE;
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
                spsws_ctx.sigfox_monitoring_data.tmcu_degrees = (uint8_t) generic_u32;
            }
        }
    }
    // PCB temperature.
    spsws_ctx.sigfox_monitoring_data.tpcb_degrees = SPSWS_ERROR_VALUE_TEMPERATURE;
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
                spsws_ctx.sigfox_monitoring_data.tpcb_degrees = (uint8_t) generic_u32;
            }
        }
    }
    // PCB humidity.
    spsws_ctx.sigfox_monitoring_data.hpcb_percent = SPSWS_ERROR_VALUE_HUMIDITY;
    sample_count = (spsws_ctx.measurements.hpcb_percent.full_flag != 0) ? SPSWS_MEASUREMENT_BUFFER_SIZE : spsws_ctx.measurements.hpcb_percent.sample_count;
    if (sample_count > 0) {
        // Compute single value.
        math_status = MATH_median_filter(spsws_ctx.measurements.hpcb_percent.sample_buffer, sample_count, 0, &generic_s32_1);
        MATH_stack_error(ERROR_BASE_MATH);
        if (math_status == MATH_SUCCESS) {
            spsws_ctx.sigfox_monitoring_data.hpcb_percent = (uint8_t) generic_s32_1;
        }
    }
    // Solar cell voltage.
    spsws_ctx.sigfox_monitoring_data.vsrc_mv = SPSWS_ERROR_VALUE_ANALOG_16BITS;
    sample_count = (spsws_ctx.measurements.vsrc_mv.full_flag != 0) ? SPSWS_MEASUREMENT_BUFFER_SIZE : spsws_ctx.measurements.vsrc_mv.sample_count;
    if (sample_count > 0) {
        // Compute single value.
        math_status = MATH_median_filter(spsws_ctx.measurements.vsrc_mv.sample_buffer, sample_count, 0, &generic_s32_1);
        MATH_stack_error(ERROR_BASE_MATH);
        if (math_status == MATH_SUCCESS) {
            spsws_ctx.sigfox_monitoring_data.vsrc_mv = (uint16_t) generic_s32_1;
        }
    }
    // Supercap voltage.
    spsws_ctx.sigfox_monitoring_data.vcap_mv = SPSWS_ERROR_VALUE_ANALOG_12BITS;
    sample_count = (spsws_ctx.measurements.vcap_mv.full_flag != 0) ? SPSWS_MEASUREMENT_BUFFER_SIZE : spsws_ctx.measurements.vcap_mv.sample_count;
    if (sample_count > 0) {
        // Select last value.
        spsws_ctx.sigfox_monitoring_data.vcap_mv = spsws_ctx.measurements.vcap_mv.sample_buffer[spsws_ctx.measurements.vcap_mv.last_sample_index];
    }
    // MCU voltage.
    spsws_ctx.sigfox_monitoring_data.vmcu_mv = SPSWS_ERROR_VALUE_ANALOG_12BITS;
    sample_count = (spsws_ctx.measurements.vmcu_mv.full_flag != 0) ? SPSWS_MEASUREMENT_BUFFER_SIZE : spsws_ctx.measurements.vmcu_mv.sample_count;
    if (sample_count > 0) {
        // Compute single value.
        math_status = MATH_median_filter(spsws_ctx.measurements.vmcu_mv.sample_buffer, sample_count, 0, &generic_s32_1);
        MATH_stack_error(ERROR_BASE_MATH);
        if (math_status == MATH_SUCCESS) {
            spsws_ctx.sigfox_monitoring_data.vmcu_mv = (uint16_t) generic_s32_1;
        }
    }
#ifdef SPSWS_WIND_RAINFALL_MEASUREMENTS
    // Wind speed.
    spsws_ctx.sigfox_weather_data.wind_speed_average_kmh = SPSWS_ERROR_VALUE_WIND;
    spsws_ctx.sigfox_weather_data.wind_speed_peak_kmh = SPSWS_ERROR_VALUE_WIND;
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
        spsws_ctx.sigfox_weather_data.wind_speed_average_kmh = (generic_s32_1 / 1000);
        spsws_ctx.sigfox_weather_data.wind_speed_peak_kmh = (generic_s32_2 / 1000);
    }
    // Wind direction.
    spsws_ctx.sigfox_weather_data.wind_direction_average_two_degrees = SPSWS_ERROR_VALUE_WIND;
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
        spsws_ctx.sigfox_weather_data.wind_direction_average_two_degrees = (generic_s32_1 >> 1);
    }
    // Rainfall.
    spsws_ctx.sigfox_weather_data.rainfall_mm = SPSWS_ERROR_VALUE_RAIN;
    sen15901_status = SEN15901_get_rainfall(&generic_s32_1);
    SEN15901_stack_error(ERROR_BASE_SEN15901);
    // Check status.
    if (sen15901_status == SEN15901_SUCCESS) {
        spsws_ctx.sigfox_weather_data.rainfall_mm = (generic_s32_1 / 1000);
        // Rounding operation.
        if ((generic_s32_1 - (spsws_ctx.sigfox_weather_data.rainfall_mm * 1000)) >= 500) {
            spsws_ctx.sigfox_weather_data.rainfall_mm++;
        }
    }
#endif
}
#endif

/*******************************************************************/
static void _SPSWS_set_clock(uint8_t device_state) {
    // Local variables.
    RCC_status_t rcc_status = RCC_SUCCESS;
    RCC_clock_t mcu_clock_source = RCC_CLOCK_NONE;
    uint8_t clock_status = 0;
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
}

#ifndef SPSWS_MODE_CLI
/*******************************************************************/
static void _SPSWS_update_time_flags(void) {
    // Local variables.
    RTC_status_t rtc_status = RTC_SUCCESS;
    NVM_status_t nvm_status = NVM_SUCCESS;
    uint8_t nvm_byte = 0;
    uint8_t local_utc_offset = 0;
    int8_t local_hour = 0;
    // Retrieve current time from RTC.
    rtc_status = RTC_get_time(&spsws_ctx.current_time);
    RTC_stack_error(ERROR_BASE_RTC);
    // Retrieve previous wake-up time from NVM.
    nvm_status = NVM_read_byte((NVM_ADDRESS_PREVIOUS_WAKE_UP_YEAR + 0), &nvm_byte);
    NVM_stack_error(ERROR_BASE_NVM);
    spsws_ctx.previous_wake_up_time.year = (nvm_byte << 8);
    nvm_status = NVM_read_byte((NVM_ADDRESS_PREVIOUS_WAKE_UP_YEAR + 1), &nvm_byte);
    NVM_stack_error(ERROR_BASE_NVM);
    spsws_ctx.previous_wake_up_time.year |= nvm_byte;
    nvm_status = NVM_read_byte(NVM_ADDRESS_PREVIOUS_WAKE_UP_MONTH, &spsws_ctx.previous_wake_up_time.month);
    NVM_stack_error(ERROR_BASE_NVM);
    nvm_status = NVM_read_byte(NVM_ADDRESS_PREVIOUS_WAKE_UP_DATE, &spsws_ctx.previous_wake_up_time.date);
    NVM_stack_error(ERROR_BASE_NVM);
    nvm_status = NVM_read_byte(NVM_ADDRESS_PREVIOUS_WAKE_UP_HOUR, &spsws_ctx.previous_wake_up_time.hours);
    NVM_stack_error(ERROR_BASE_NVM);
    // Check time are different (avoiding false wake-up due to RTC calibration).
    if ((spsws_ctx.current_time.year != spsws_ctx.previous_wake_up_time.year) || (spsws_ctx.current_time.month != spsws_ctx.previous_wake_up_time.month) || (spsws_ctx.current_time.date != spsws_ctx.previous_wake_up_time.date)) {
        // Day (and thus hour) has changed.
        spsws_ctx.flags.day_changed = 1;
        spsws_ctx.flags.hour_changed = 1;
    }
    if (spsws_ctx.current_time.hours != spsws_ctx.previous_wake_up_time.hours) {
        // Hour as changed.
        spsws_ctx.flags.hour_changed = 1;
    }
    // Check if we are in afternoon (to enable device geolocation).
    local_utc_offset = RTC_LOCAL_UTC_OFFSET_WINTER;
    if ((spsws_ctx.current_time.month > RTC_WINTER_TIME_LAST_MONTH) && (spsws_ctx.current_time.month < RTC_WINTER_TIME_FIRST_MONTH)) {
        local_utc_offset = RTC_LOCAL_UTC_OFFSET_SUMMER;
    }
    local_hour = (spsws_ctx.current_time.hours + local_utc_offset) % RTC_NUMBER_OF_HOURS_PER_DAY;
    if (local_hour < 0) {
        local_hour += RTC_NUMBER_OF_HOURS_PER_DAY;
    }
    if (local_hour >= RTC_AFTERNOON_HOUR_THRESHOLD) {
        spsws_ctx.flags.is_afternoon = 1;
    }
}
#endif

#ifndef SPSWS_MODE_CLI
/*******************************************************************/
static void _SPSWS_update_pwut(void) {
    // Local variables.
    RTC_status_t rtc_status = RTC_SUCCESS;
    NVM_status_t nvm_status = NVM_SUCCESS;
    // Retrieve current time from RTC.
    rtc_status = RTC_get_time(&spsws_ctx.current_time);
    RTC_stack_error(ERROR_BASE_RTC);
    // Update previous wake-up time.
    nvm_status = NVM_write_byte((NVM_ADDRESS_PREVIOUS_WAKE_UP_YEAR + 0), (uint8_t) (spsws_ctx.current_time.year >> 8));
    NVM_stack_error(ERROR_BASE_NVM);
    nvm_status = NVM_write_byte((NVM_ADDRESS_PREVIOUS_WAKE_UP_YEAR + 1), (uint8_t) (spsws_ctx.current_time.year >> 0));
    NVM_stack_error(ERROR_BASE_NVM);
    nvm_status = NVM_write_byte(NVM_ADDRESS_PREVIOUS_WAKE_UP_MONTH, spsws_ctx.current_time.month);
    NVM_stack_error(ERROR_BASE_NVM);
    nvm_status = NVM_write_byte(NVM_ADDRESS_PREVIOUS_WAKE_UP_DATE, spsws_ctx.current_time.date);
    NVM_stack_error(ERROR_BASE_NVM);
    nvm_status = NVM_write_byte(NVM_ADDRESS_PREVIOUS_WAKE_UP_HOUR, spsws_ctx.current_time.hours);
    NVM_stack_error(ERROR_BASE_NVM);
}
#endif

#ifndef SPSWS_MODE_CLI
/*******************************************************************/
static void _SPSWS_send_sigfox_message(SIGFOX_EP_API_application_message_t* application_message) {
    // Local variables.
    SIGFOX_EP_API_status_t sigfox_ep_api_status = SIGFOX_EP_API_SUCCESS;
    SIGFOX_EP_API_config_t lib_config;
    // Directly exit of the radio is disabled due to low supercap voltage.
    if (spsws_ctx.flags.radio_enabled == 0) goto errors;
    // Library configuration.
    lib_config.rc = &SIGFOX_RC1;
    // Open library.
    sigfox_ep_api_status = SIGFOX_EP_API_open(&lib_config);
    SIGFOX_EP_API_stack_error();
    // Send message.
    sigfox_ep_api_status = SIGFOX_EP_API_send_application_message(application_message);
    SIGFOX_EP_API_stack_error();
    // Close library.
    sigfox_ep_api_status = SIGFOX_EP_API_close();
    SIGFOX_EP_API_stack_error();
errors:
    return;
}
#endif

/*******************************************************************/
static void _SPSWS_init_context(void) {
    // Init context.
    spsws_ctx.state = SPSWS_STATE_STARTUP;
    spsws_ctx.flags.all = 0;
    spsws_ctx.flags.por = 1;
    spsws_ctx.flags.radio_enabled = 1;
    spsws_ctx.status.all = 0;
#ifndef SPSWS_MODE_CLI
    spsws_ctx.seconds_counter = 0;
#endif
    // Init station mode.
#ifdef SPSWS_WIND_RAINFALL_MEASUREMENTS
    spsws_ctx.status.station_mode = 0b1;
#else
    spsws_ctx.status.station_mode = 0b0;
#endif
#ifndef SPSWS_MODE_CLI
    // Reset measurements.
    _SPSWS_reset_measurements();
#endif
}

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
#ifdef SPSWS_WIND_RAINFALL_MEASUREMENTS
    SEN15901_status_t sen15901_status = SEN15901_SUCCESS;
#ifdef SPSWS_WIND_VANE_ULTIMETER
    ULTIMETER_status_t ultimeter_status = ULTIMETER_SUCCESS;
#endif
#endif
    RTC_alarm_configuration_t rtc_alarm_config;
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
    rtc_status = RTC_init(&_SPSWS_tick_second_callback, NVIC_PRIORITY_RTC);
    RTC_stack_error(ERROR_BASE_RTC);
    // Read LS byte of the device ID to add a random delay in RTC alarm.
    nvm_status = NVM_read_byte((NVM_ADDRESS_SIGFOX_EP_ID + SIGFOX_EP_ID_SIZE_BYTES - 1), &device_id_lsbyte);
    NVM_stack_error(ERROR_BASE_NVM);
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
    rtc_status = RTC_start_alarm(RTC_ALARM_A, &rtc_alarm_config, &_SPSWS_fixed_hour_alarm_callback);
    RTC_stack_error(ERROR_BASE_RTC);
    // Init delay timer.
    lptim_status = LPTIM_init(NVIC_PRIORITY_DELAY);
    LPTIM_stack_error(ERROR_BASE_LPTIM);
#ifdef SPSWS_WIND_RAINFALL_MEASUREMENTS
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
    GPS_position_t gps_position;
    GPS_acquisition_status_t gps_acquisition_status = GPS_ACQUISITION_SUCCESS;
    uint32_t gps_acquisition_duration_seconds = 0;
    SIGFOX_EP_API_application_message_t application_message;
    ERROR_code_t error_code = 0;
    int32_t generic_s32_1 = 0;
    int32_t generic_s32_2 = 0;
    uint8_t idx = 0;
    // Init board.
    _SPSWS_init_context();
    _SPSWS_init_hw();
    // Application message default parameters.
    application_message.common_parameters.number_of_frames = 3;
    application_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_100BPS;
    application_message.type = SIGFOX_APPLICATION_MESSAGE_TYPE_BYTE_ARRAY;
#ifdef SIGFOX_EP_BIDIRECTIONAL
    application_message.bidirectional_flag = 0;
#endif
    application_message.ul_payload = SIGFOX_NULL;
    application_message.ul_payload_size_bytes = 0;
    // Main loop.
    while (1) {
        // Reload watchdog.
        IWDG_reload();
        // Perform state machine.
        switch (spsws_ctx.state) {
        case SPSWS_STATE_STARTUP:
            // Power on delay to wait for main power supply stabilization.
            lptim_status = LPTIM_delay_milliseconds(SPSWS_POWER_ON_DELAY_MS, LPTIM_DELAY_MODE_STOP);
            LPTIM_stack_error(ERROR_BASE_LPTIM);
            // Switch to accurate clock.
            _SPSWS_set_clock(1);
            // Fill reset reason and software version.
            spsws_ctx.sigfox_startup_data.reset_reason = PWR_get_reset_flags();
            spsws_ctx.sigfox_startup_data.major_version = GIT_MAJOR_VERSION;
            spsws_ctx.sigfox_startup_data.minor_version = GIT_MINOR_VERSION;
            spsws_ctx.sigfox_startup_data.commit_index = GIT_COMMIT_INDEX;
            spsws_ctx.sigfox_startup_data.commit_id = GIT_COMMIT_ID;
            spsws_ctx.sigfox_startup_data.dirty_flag = GIT_DIRTY_FLAG;
            // Clear reset flags.
            PWR_clear_reset_flags();
            // Send SW version frame.
            application_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_600BPS;
            application_message.ul_payload = (sfx_u8*) (spsws_ctx.sigfox_startup_data.frame);
            application_message.ul_payload_size_bytes = SPSWS_SIGFOX_STARTUP_DATA_SIZE;
            _SPSWS_send_sigfox_message(&application_message);
            // Perform first RTC calibration.
            spsws_ctx.state = SPSWS_STATE_RTC_CALIBRATION;
            break;
        case SPSWS_STATE_WAKEUP:
            // Check alarm flag..
            if (spsws_ctx.flags.fixed_hour_alarm != 0) {
                // Clear flag.
                spsws_ctx.flags.fixed_hour_alarm = 0;
                // Update flags.
                _SPSWS_update_time_flags();
                // Check hour change flag.
                if (spsws_ctx.flags.hour_changed != 0) {
                    // Valid fixed hour wake-up.
                    _SPSWS_update_pwut();
                    // Check if day changed.
                    if (spsws_ctx.flags.day_changed != 0) {
                        // Reset daily flags.
                        spsws_ctx.flags.day_changed = 0;
                        spsws_ctx.flags.is_afternoon = 0;
                        spsws_ctx.flags.daily_geoloc_done = 0;
                        spsws_ctx.flags.daily_rtc_calibration_done = 0;
                    }
                    spsws_ctx.flags.hour_changed = 0;
                    // Calibrate clocks.
                    rcc_status = RCC_calibrate_internal_clocks(NVIC_PRIORITY_CLOCK_CALIBRATION);
                    RCC_stack_error(ERROR_BASE_RCC);
                    // Switch to accurate clock.
                    _SPSWS_set_clock(1);
                    // Next state
                    spsws_ctx.state = SPSWS_STATE_MONITORING;
                }
                else {
                    // False detection due to RTC calibration.
                    spsws_ctx.state = SPSWS_STATE_OFF;
                }
            }
            else {
                // Intermediate analog wake-up.
                spsws_ctx.state = SPSWS_STATE_MEASURE;
            }
            break;
        case SPSWS_STATE_RTC_CALIBRATION:
            // Reset status to default.
            spsws_ctx.status.daily_rtc_calibration = 0;
            // Turn GPS on.
            POWER_enable(POWER_REQUESTER_ID_MAIN, POWER_DOMAIN_GPS, LPTIM_DELAY_MODE_SLEEP);
            // Get current time from GPS.
            gps_status = GPS_get_time(&gps_time, SPSWS_RTC_CALIBRATION_TIMEOUT_SECONDS, &gps_acquisition_duration_seconds, &gps_acquisition_status);
            GPS_stack_error(ERROR_BASE_GPS);
            // Turn GPS off.
            POWER_disable(POWER_REQUESTER_ID_MAIN, POWER_DOMAIN_GPS);
            // Calibrate RTC if time is available.
            if (gps_acquisition_status == GPS_ACQUISITION_SUCCESS) {
                // Copy structure.
                spsws_ctx.current_time.year = gps_time.year;
                spsws_ctx.current_time.month = gps_time.month;
                spsws_ctx.current_time.date = gps_time.date;
                spsws_ctx.current_time.hours = gps_time.hours;
                spsws_ctx.current_time.minutes = gps_time.minutes;
                spsws_ctx.current_time.seconds = gps_time.seconds;
                // Update RTC registers.
                rtc_status = RTC_set_time(&spsws_ctx.current_time);
                RTC_stack_error(ERROR_BASE_RTC);
                // Update status bit.
                if (rtc_status == RTC_SUCCESS) {
                    spsws_ctx.status.first_rtc_calibration = 1;
                    spsws_ctx.status.daily_rtc_calibration = 1;
                }
            }
            // Compute next state.
            if (spsws_ctx.flags.por != 0) {
                // In POR condition, RTC alarm will occur during the first GPS time acquisition because of the RTC reset and the random delay.
                // Flags are manually cleared to avoid wake-up directly after the first RTC calibration.
                spsws_ctx.flags.fixed_hour_alarm = 0;
                spsws_ctx.flags.wake_up = 0;
                // Update state.
                spsws_ctx.state = SPSWS_STATE_ERROR_STACK;
            }
            else {
                spsws_ctx.state = SPSWS_STATE_OFF;
            }
            // Update flags.
            spsws_ctx.flags.daily_rtc_calibration_done = 1;
            spsws_ctx.flags.por = 0;
            break;
        case SPSWS_STATE_MEASURE:
            // Retrieve internal ADC data.
            POWER_enable(POWER_REQUESTER_ID_MAIN, POWER_DOMAIN_ANALOG, LPTIM_DELAY_MODE_SLEEP);
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
            // Retrieve external ADC data.
            // Note: digital sensors power supply must also be enabled at this step to power the LDR.
            POWER_enable(POWER_REQUESTER_ID_MAIN, POWER_DOMAIN_SENSORS, LPTIM_DELAY_MODE_SLEEP);
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
                _SPSWS_measurement_add_sample(&(spsws_ctx.measurements.tpcb_degrees), generic_s32_1);
                _SPSWS_measurement_add_sample(&(spsws_ctx.measurements.hpcb_percent), generic_s32_2);
#ifdef HW1_0
                _SPSWS_measurement_add_sample(&(spsws_ctx.measurements.tamb_degrees), generic_s32_1);
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
                _SPSWS_measurement_add_sample(&(spsws_ctx.measurements.tamb_degrees), generic_s32_1);
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
            // Go to off state.
            spsws_ctx.state = SPSWS_STATE_OFF;
            break;
        case SPSWS_STATE_MONITORING:
            // Compute average data.
            _SPSWS_compute_final_measurements();
            _SPSWS_reset_measurements();
            // Read status byte.
            spsws_ctx.sigfox_monitoring_data.status = spsws_ctx.status.all;
            // Send uplink monitoring frame.
            application_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_600BPS;
            application_message.ul_payload = (sfx_u8*) (spsws_ctx.sigfox_monitoring_data.frame);
            application_message.ul_payload_size_bytes = SPSWS_SIGFOX_MONITORING_DATA_SIZE;
            _SPSWS_send_sigfox_message(&application_message);
            // Compute next state.
            spsws_ctx.state = SPSWS_STATE_WEATHER_DATA;
            break;
        case SPSWS_STATE_WEATHER_DATA:
#ifdef SPSWS_SEN15901_EMULATOR
            // Synchronize emulator on weather data message transmission.
            GPIO_write(&SPSWS_SEN15901_EMULATOR_SYNCHRO_GPIO, 1);
#endif
            // Send uplink weather frame.
            application_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_100BPS;
            application_message.ul_payload = (sfx_u8*) (spsws_ctx.sigfox_weather_data.frame);
            application_message.ul_payload_size_bytes = SPSWS_SIGFOX_WEATHER_DATA_SIZE;
            _SPSWS_send_sigfox_message(&application_message);
#ifdef SPSWS_SEN15901_EMULATOR
            GPIO_write(&SPSWS_SEN15901_EMULATOR_SYNCHRO_GPIO, 0);
#endif
            // Compute next state.
            if (spsws_ctx.flags.daily_rtc_calibration_done == 0) {
                // Perform RTC calibration.
                spsws_ctx.state = SPSWS_STATE_RTC_CALIBRATION;
            }
            else {
                if ((spsws_ctx.flags.daily_geoloc_done == 0) && (spsws_ctx.flags.is_afternoon != 0)) {
                    // Perform device geolocation.
                    spsws_ctx.state = SPSWS_STATE_GEOLOC;
                }
                else {
                    // Enter stop mode.
                    spsws_ctx.state = SPSWS_STATE_OFF;
                }
            }
            break;
        case SPSWS_STATE_GEOLOC:
            // Reset status to default.
            spsws_ctx.status.daily_geoloc = 0;
            // Turn GPS on.
            POWER_enable(POWER_REQUESTER_ID_MAIN, POWER_DOMAIN_GPS, LPTIM_DELAY_MODE_SLEEP);
            // Get geolocation from GPS.
            gps_status = GPS_get_position(&gps_position, SPSWS_GEOLOC_TIMEOUT_SECONDS, &gps_acquisition_duration_seconds, &gps_acquisition_status);
            GPS_stack_error(ERROR_BASE_GPS);
            // Turn GPS off.
            POWER_disable(POWER_REQUESTER_ID_MAIN, POWER_DOMAIN_GPS);
            // Build Sigfox frame.
            if (gps_acquisition_status == GPS_ACQUISITION_SUCCESS) {
                spsws_ctx.sigfox_geoloc_data.latitude_degrees = gps_position.lat_degrees;
                spsws_ctx.sigfox_geoloc_data.latitude_minutes = gps_position.lat_minutes;
                spsws_ctx.sigfox_geoloc_data.latitude_seconds = gps_position.lat_seconds;
                spsws_ctx.sigfox_geoloc_data.latitude_north_flag = gps_position.lat_north_flag;
                spsws_ctx.sigfox_geoloc_data.longitude_degrees = gps_position.long_degrees;
                spsws_ctx.sigfox_geoloc_data.longitude_minutes = gps_position.long_minutes;
                spsws_ctx.sigfox_geoloc_data.longitude_seconds = gps_position.long_seconds;
                spsws_ctx.sigfox_geoloc_data.longitude_east_flag = gps_position.long_east_flag;
                spsws_ctx.sigfox_geoloc_data.altitude_meters = gps_position.altitude;
                spsws_ctx.sigfox_geoloc_data.gps_acquisition_duration_seconds = gps_acquisition_duration_seconds;
                // Update message parameters.
                application_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_100BPS;
                application_message.ul_payload = (sfx_u8*) (spsws_ctx.sigfox_geoloc_data.frame);
                application_message.ul_payload_size_bytes = SPSWS_SIGFOX_GEOLOC_DATA_SIZE;
                // Update status bit.
                spsws_ctx.status.daily_geoloc = 1;
            }
            else {
                spsws_ctx.sigfox_geoloc_timeout_data.gps_acquisition_status = gps_acquisition_status;
                spsws_ctx.sigfox_geoloc_timeout_data.gps_acquisition_duration_seconds = gps_acquisition_duration_seconds;
                // Update message parameters.
                application_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_100BPS;
                application_message.ul_payload = (sfx_u8*) (spsws_ctx.sigfox_geoloc_timeout_data.frame);
                application_message.ul_payload_size_bytes = SPSWS_SIGFOX_GEOLOC_TIMEOUT_DATA_SIZE;
            }
            // Send uplink geolocation frame.
            _SPSWS_send_sigfox_message(&application_message);
            // Update done flag.
            spsws_ctx.flags.daily_geoloc_done = 1;
            // Send error stack frame.
            spsws_ctx.state = SPSWS_STATE_ERROR_STACK;
            break;
        case SPSWS_STATE_ERROR_STACK:
            // Import Sigfox library error stack.
            ERROR_import_sigfox_stack();
            // Check stack.
            if (ERROR_stack_is_empty() == 0) {
                // Read error stack.
                for (idx = 0; idx < (SPSWS_SIGFOX_ERROR_STACK_DATA_SIZE >> 1); idx++) {
                    error_code = ERROR_stack_read();
                    spsws_ctx.sigfox_error_stack_data[(idx << 1) + 0] = (uint8_t) ((error_code >> 8) & 0x00FF);
                    spsws_ctx.sigfox_error_stack_data[(idx << 1) + 1] = (uint8_t) ((error_code >> 0) & 0x00FF);
                }
                // Send frame.
                application_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_600BPS;
                application_message.ul_payload = (sfx_u8*) (spsws_ctx.sigfox_error_stack_data);
                application_message.ul_payload_size_bytes = SPSWS_SIGFOX_ERROR_STACK_DATA_SIZE;
                _SPSWS_send_sigfox_message(&application_message);
                // Reset error stack.
                ERROR_stack_init();
            }
            // Enter sleep mode.
            spsws_ctx.state = SPSWS_STATE_OFF;
            break;
        case SPSWS_STATE_OFF:
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
            spsws_ctx.state = SPSWS_STATE_SLEEP;
            break;
        case SPSWS_STATE_SLEEP:
            // Enter sleep mode.
            IWDG_reload();
#ifndef SPSWS_MODE_DEBUG
            PWR_enter_deepsleep_mode(PWR_DEEPSLEEP_MODE_STOP);
            IWDG_reload();
#endif
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
            if (spsws_ctx.seconds_counter >= SPSWS_MEASUREMENT_PERIOD_SECONDS) {
                // Reset counter and wake-up for intermediate measurement.
                spsws_ctx.seconds_counter = 0;
                spsws_ctx.flags.wake_up = 1;
            }
            // Check RTC alarm.
            if (spsws_ctx.flags.fixed_hour_alarm != 0) {
                // Wake up for radio transmission.
                spsws_ctx.flags.wake_up = 1;
#ifdef SPSWS_WIND_RAINFALL_MEASUREMENTS
                // Stop rainfall measurement.
                sen15901_status = SEN15901_set_rainfall_measurement(0);
                SEN15901_stack_error(ERROR_BASE_SEN15901);
#endif
            }
            // Wake-up if required.
            if (spsws_ctx.flags.wake_up != 0) {
#ifdef SPSWS_WIND_RAINFALL_MEASUREMENTS
                // Stop wind measurement.
#ifdef SPSWS_WIND_VANE_ULTIMETER
                ultimeter_status = ULTIMETER_set_wind_measurement(0);
                ULTIMETER_stack_error(ERROR_BASE_ULTIMETER);
#else
                sen15901_status = SEN15901_set_wind_measurement(0);
                SEN15901_stack_error(ERROR_BASE_SEN15901);
#endif
#endif
                // Clear flag and update state.
                spsws_ctx.flags.wake_up = 0;
                spsws_ctx.state = SPSWS_STATE_WAKEUP;
            }
            break;
        default:
            // Enter standby mode.
            spsws_ctx.state = SPSWS_STATE_OFF;
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
    _SPSWS_init_context();
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
