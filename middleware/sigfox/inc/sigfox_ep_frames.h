/*
 * sigfox_ep_frames.h
 *
 *  Created on: 14 nov. 2025
 *      Author: Ludo
 */

#ifndef __SIGFOX_EP_FRAMES_H__
#define __SIGFOX_EP_FRAMES_H__

#include "sigfox_types.h"
#include "spsws_flags.h"
#include "types.h"

/*** SIGFOX EP FRAMES macros ***/

// Uplink payload sizes.
#define SIGFOX_EP_UL_PAYLOAD_SIZE_STARTUP           8
#define SIGFOX_EP_UL_PAYLOAD_SIZE_ERROR_STACK       12
#ifdef SPSWS_WIND_RAINFALL_MEASUREMENTS
#define SIGFOX_EP_UL_PAYLOAD_SIZE_WEATHER           10
#else
#define SIGFOX_EP_UL_PAYLOAD_SIZE_WEATHER           6
#endif
#define SIGFOX_EP_UL_PAYLOAD_SIZE_MONITORING        9
#define SIGFOX_EP_UL_PAYLOAD_SIZE_GEOLOC            11
#define SIGFOX_EP_UL_PAYLOAD_SIZE_GEOLOC_TIMEOUT    2
// Error values.
#define SIGFOX_EP_ERROR_VALUE_ANALOG_12BITS         0xFFF
#define SIGFOX_EP_ERROR_VALUE_ANALOG_16BITS         0xFFFF
#define SIGFOX_EP_ERROR_VALUE_TEMPERATURE           0x7F
#define SIGFOX_EP_ERROR_VALUE_HUMIDITY              0xFF
#define SIGFOX_EP_ERROR_VALUE_LIGHT                 0xFF
#define SIGFOX_EP_ERROR_VALUE_UV_INDEX              0xFF
#define SIGFOX_EP_ERROR_VALUE_PRESSURE              0xFFFF
#define SIGFOX_EP_ERROR_VALUE_WIND                  0xFF
#define SIGFOX_EP_ERROR_VALUE_RAIN                  0xFF

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

/*******************************************************************/
typedef union {
    uint8_t frame[SIGFOX_EP_UL_PAYLOAD_SIZE_WEATHER];
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
    } __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
} SPSWS_EP_ul_payload_weather_t;

/*!******************************************************************
 * \struct SIGFOX_EP_ul_payload_monitoring_t
 * \brief Sigfox uplink monitoring frame format.
 *******************************************************************/
typedef union {
    uint8_t frame[SIGFOX_EP_UL_PAYLOAD_SIZE_MONITORING];
    struct {
        unsigned tmcu_degrees :8;
        unsigned tpcb_degrees :8;
        unsigned hpcb_percent :8;
        unsigned vsrc_mv :16;
        unsigned vcap_mv :12;
        unsigned vmcu_mv :12;
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

/*!******************************************************************
 * \enum SIGFOX_EP_dl_op_code_t
 * \brief Sigfox downlink operation codes.
 *******************************************************************/
typedef enum {
    SIGFOX_EP_DL_OP_CODE_NOP = 0,
    SIGFOX_EP_DL_OP_CODE_RESET,
    SIGFOX_EP_DL_OP_CODE_LAST
} SIGFOX_EP_dl_op_code_t;

#endif /* __SIGFOX_EP_FRAMES_H__ */
