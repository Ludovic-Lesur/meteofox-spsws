/*
 * sigfox_ep_frames.h
 *
 *  Created on: 14 nov. 2025
 *      Author: Ludo
 */

#ifndef __SIGFOX_EP_FRAMES_H__
#define __SIGFOX_EP_FRAMES_H__

#include "sigfox_ep_flags.h"
#include "sigfox_types.h"
#include "spsws_flags.h"
#include "types.h"

/*** SIGFOX EP FRAMES macros ***/

// Uplink payload sizes.
#define SIGFOX_EP_UL_PAYLOAD_SIZE_STARTUP                       8
#define SIGFOX_EP_UL_PAYLOAD_SIZE_ERROR_STACK                   10
#ifdef SPSWS_WIND_RAINFALL_MEASUREMENTS
#define SIGFOX_EP_UL_PAYLOAD_SIZE_WEATHER                       12
#else
#define SIGFOX_EP_UL_PAYLOAD_SIZE_WEATHER                       7
#endif
#define SIGFOX_EP_UL_PAYLOAD_SIZE_MONITORING                    9
#define SIGFOX_EP_UL_PAYLOAD_SIZE_GEOLOC                        11
#define SIGFOX_EP_UL_PAYLOAD_SIZE_GEOLOC_TIMEOUT                2
// Error values.
#define SIGFOX_EP_ERROR_VALUE_TEMPERATURE                       0x7FF
#define SIGFOX_EP_ERROR_VALUE_HUMIDITY                          0x7F
#define SIGFOX_EP_ERROR_VALUE_SUNSHINE_LIGHT                    0xFFFF
#define SIGFOX_EP_ERROR_VALUE_SUNSHINE_UV_INDEX                 0x7F
#define SIGFOX_EP_ERROR_VALUE_PRESSURE                          0x3FFF
#define SIGFOX_EP_ERROR_VALUE_WIND_SPEED                        0x7FF
#define SIGFOX_EP_ERROR_VALUE_WIND_DIRECTION                    0x1FF
#define SIGFOX_EP_ERROR_VALUE_RAINFALL                          0x1FF
#define SIGFOX_EP_ERROR_VALUE_SOURCE_VOLTAGE                    0xFFF
#define SIGFOX_EP_ERROR_VALUE_STORAGE_VOLTAGE                   0xFFF
#define SIGFOX_EP_ERROR_VALUE_MCU_TEMPERATURE                   0x7F
#define SIGFOX_EP_ERROR_VALUE_MCU_VOLTAGE                       0xFFF
// Sunshine light representation.
#define SIGFOX_EP_SHUNSHINE_LIGHT_MAX_MLUX                      163820000
#define SIGFOX_EP_SHUNSHINE_LIGHT_UNIT_THRESHOLD_HIGH_MLUX      16383000
#define SIGFOX_EP_SHUNSHINE_LIGHT_UNIT_THRESHOLD_MIDDLE_MLUX    1638300
#define SIGFOX_EP_SHUNSHINE_LIGHT_UNIT_THRESHOLD_LOW_MLUX       163830
// Rainfall representation.
#define SIGFOX_EP_RAINFALL_MAX_UM                               254000
#define SIGFOX_EP_RAINFALL_UNIT_THRESHOLD_UM                    25500

/*** SIGFOX EP FRAMES structures ***/

/*!******************************************************************
 * \struct SIGFOX_EP_ul_payload_startup_t
 * \brief Sigfox uplink startup frame format.
 *******************************************************************/
typedef union {
    uint8_t frame[SIGFOX_EP_UL_PAYLOAD_SIZE_STARTUP];
    struct {
        unsigned reset_reason :8;
        unsigned major_version :8;
        unsigned minor_version :8;
        unsigned commit_index :8;
        unsigned commit_id :28;
        unsigned dirty_flag :4;
    } __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
} SIGFOX_EP_ul_payload_startup_t;

/*!******************************************************************
 * \struct SIGFOX_EP_ul_payload_rainfall_unit_t
 * \brief Sigfox uplink sunshine light unit format.
 *******************************************************************/
typedef enum {
    SIGFOX_EP_UL_PAYLOAD_SUNSHINE_LIGHT_UNIT_HUNDREDTH_LUX = 0b00,
    SIGFOX_EP_UL_PAYLOAD_SUNSHINE_LIGHT_UNIT_TENTH_LUX = 0b01,
    SIGFOX_EP_UL_PAYLOAD_SUNSHINE_LIGHT_UNIT_LUX = 0b10,
    SIGFOX_EP_UL_PAYLOAD_SUNSHINE_LIGHT_UNIT_TEN_LUX = 0b11
} SIGFOX_EP_ul_payload_sunshine_light_unit_t;

/*!******************************************************************
 * \struct SIGFOX_EP_ul_payload_sunshine_light_t
 * \brief Sigfox uplink sunshine light field format.
 *******************************************************************/
typedef union {
    uint16_t all;
    struct {
        unsigned value: 14;
        SIGFOX_EP_ul_payload_sunshine_light_unit_t unit :2;
    } __attribute__((scalar_storage_order("little-endian"))) __attribute__((packed));
} SIGFOX_EP_ul_payload_sunshine_light_t;

/*!******************************************************************
 * \struct SIGFOX_EP_ul_payload_rainfall_unit_t
 * \brief Sigfox uplink rainfall unit format.
 *******************************************************************/
typedef enum {
    SIGFOX_EP_UL_PAYLOAD_RAINFALL_UNIT_TENTH_MM = 0b0,
    SIGFOX_EP_UL_PAYLOAD_RAINFALL_UNIT_MM = 0b1
} SIGFOX_EP_ul_payload_rainfall_unit_t;

/*!******************************************************************
 * \struct SIGFOX_EP_ul_payload_rainfall_t
 * \brief Sigfox uplink rainfall field format.
 *******************************************************************/
typedef union {
    uint16_t all;
    struct {
        unsigned value: 8;
        SIGFOX_EP_ul_payload_rainfall_unit_t unit :1;
    } __attribute__((scalar_storage_order("little-endian"))) __attribute__((packed));
} SIGFOX_EP_ul_payload_rainfall_t;

/*!******************************************************************
 * \struct SPSWS_EP_ul_payload_weather_t
 * \brief Sigfox uplink weather frame format.
 *******************************************************************/
typedef union {
    uint8_t frame[SIGFOX_EP_UL_PAYLOAD_SIZE_WEATHER];
    struct {
        unsigned temperature_tenth_degrees :12;                 // [-204.7 to 204.6°C / 0.1°C]
        unsigned humidity_percent :7;                           // [0 to 126% / 1%]
        unsigned sunshine_light :16;                            // [0.00 to 163.84lux / 0.01lux] + [0.0 to 1638.4lux / 0.1lux] + [0 to 16384lux / 1lux] + [0 to 163830lux / 10lux]
        unsigned sunshine_uv_index_duvi :7;                     // [0.0 to 12.6UVI / 0.1UVI]
        unsigned pressure_atmospheric_absolute_tenth_hpa :14;   // [0.0 to 1638.2hPa / 0.1hPa]
#ifdef SPSWS_WIND_RAINFALL_MEASUREMENTS
        unsigned wind_speed_average_tenth_kmh :11;              // [0.0 to 204.6km/h / 0.1km/h]
        unsigned wind_speed_peak_tenth_kmh :11;                 // [0.0 to 204.6km/h / 0.1km/h]
        unsigned wind_direction_average_degrees :9;             // [0.0 to 360d / 1d]
        unsigned rainfall :9;                                   // [0.0 to 25.5mm / 0.1mm] + [0 to 254mm / 1mm]
#endif
    } __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
} SPSWS_EP_ul_payload_weather_t;

/*!******************************************************************
 * \struct SIGFOX_EP_ul_payload_monitoring_t
 * \brief Sigfox uplink monitoring frame format.
 *******************************************************************/
typedef union {
    uint8_t frame[SIGFOX_EP_UL_PAYLOAD_SIZE_MONITORING];
    struct {
        unsigned temperature_tenth_degrees :12;
        unsigned humidity_percent :8;
        unsigned source_voltage_ten_mv :12;
        unsigned storage_voltage_mv :12;
        unsigned mcu_temperature_degrees :8;
        unsigned mcu_voltage_mv :12;
        unsigned status :8;
    } __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
} SIGFOX_EP_ul_payload_monitoring_t;

/*!******************************************************************
 * \struct SIGFOX_EP_ul_payload_geoloc_t
 * \brief Sigfox uplink geolocation frame format.
 *******************************************************************/
typedef union {
    uint8_t frame[SIGFOX_EP_UL_PAYLOAD_SIZE_GEOLOC];
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
    } __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
} SIGFOX_EP_ul_payload_geoloc_t;

/*!******************************************************************
 * \struct SIGFOX_EP_ul_payload_geoloc_timeout_t
 * \brief Sigfox uplink geolocation timeout frame format.
 *******************************************************************/
typedef union {
    uint8_t frame[SIGFOX_EP_UL_PAYLOAD_SIZE_GEOLOC_TIMEOUT];
    struct {
        unsigned gps_acquisition_status :8;
        unsigned gps_acquisition_duration_seconds :8;
    } __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
} SIGFOX_EP_ul_payload_geoloc_timeout_t;

#ifdef SIGFOX_EP_BIDIRECTIONAL
/*!******************************************************************
 * \enum SIGFOX_EP_dl_op_code_t
 * \brief Sigfox downlink operation codes.
 *******************************************************************/
typedef enum {
    SIGFOX_EP_DL_OP_CODE_NOP = 0,
    SIGFOX_EP_DL_OP_CODE_RESET,
    SIGFOX_EP_DL_OP_CODE_SET_WEATHER_DATA_PERIOD,
    SIGFOX_EP_DL_OP_CODE_SET_DATE_TIME,
    SIGFOX_EP_DL_OP_CODE_LAST
} SIGFOX_EP_dl_op_code_t;
#endif

#ifdef SIGFOX_EP_BIDIRECTIONAL
/*!******************************************************************
 * \enum SIGFOX_EP_dl_weather_data_period_t
 * \brief Sigfox downlink weather data period field values.
 *******************************************************************/
typedef enum {
    SIGFOX_EP_DL_WEATHER_DATA_PERIOD_60_MINUTES = 0,
    SIGFOX_EP_DL_WEATHER_DATA_PERIOD_30_MINUTES,
    SIGFOX_EP_DL_WEATHER_DATA_PERIOD_20_MINUTES,
    SIGFOX_EP_DL_WEATHER_DATA_PERIOD_15_MINUTES,
    SIGFOX_EP_DL_WEATHER_DATA_PERIOD_12_MINUTES,
    SIGFOX_EP_DL_WEATHER_DATA_PERIOD_10_MINUTES,
    SIGFOX_EP_DL_WEATHER_DATA_PERIOD_LAST
} SIGFOX_EP_dl_weather_data_period_t;
#endif

#ifdef SIGFOX_EP_BIDIRECTIONAL
/*!******************************************************************
 * \enum SIGFOX_EP_dl_payload_t
 * \brief Sigfox downlink frames format.
 *******************************************************************/
typedef union {
    uint8_t frame[SIGFOX_DL_PAYLOAD_SIZE_BYTES];
    struct {
        unsigned op_code :8;
        union {
            struct {
                unsigned weather_data_period :8;
                unsigned unused0 :16;
                unsigned unused1 :16;
                unsigned unused2 :16;
            } __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed)) set_weather_data_period;
            struct {
                unsigned year :16;
                unsigned month :8;
                unsigned date :8;
                unsigned hours :8;
                unsigned minutes :8;
                unsigned seconds :8;
            } __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed)) set_date_time;
        };
    } __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
} SIGFOX_EP_dl_payload_t;
#endif

#endif /* __SIGFOX_EP_FRAMES_H__ */
