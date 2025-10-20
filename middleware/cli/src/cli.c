/*
 * cli.c
 *
 *  Created on: 09 nov. 2018
 *      Author: Ludo
 */

#include "cli.h"

// Peripherals.
#include "i2c_address.h"
#include "iwdg.h"
#include "lptim.h"
#include "nvic_priority.h"
#include "nvm.h"
#include "nvm_address.h"
#include "pwr.h"
#include "rcc.h"
// Utils.
#include "at.h"
#include "error.h"
#include "maths.h"
#include "parser.h"
#include "strings.h"
#include "terminal_instance.h"
#include "types.h"
// Components.
#include "dps310.h"
#include "sht3x.h"
#include "si1133.h"
#include "sx1232.h"
// Middleware.
#include "analog.h"
#include "gps.h"
#include "power.h"
#include "rfe.h"
// Sigfox.
#include "manuf/rf_api.h"
#include "sigfox_ep_addon_rfp_api.h"
#include "sigfox_ep_api.h"
#include "sigfox_ep_flags.h"
#include "sigfox_error.h"
#include "sigfox_rc.h"
#include "sigfox_types.h"
// Applicative.
#include "error_base.h"
#include "spsws_flags.h"
#include "version.h"

#ifdef SPSWS_MODE_CLI

/*** CLI local macros ***/

#define CLI_CHAR_SEPARATOR              STRING_CHAR_COMMA
#define CLI_RSSI_REPORT_PERIOD_MS       500
#define CLI_TEMPERATURE_STRING_SIZE     5

/*** CLI local structures ***/

/*******************************************************************/
typedef struct {
    volatile uint8_t at_process_flag;
    PARSER_context_t* at_parser_ptr;
} CLI_context_t;

/*** CLI local functions declaration ***/

static AT_status_t _CLI_z_callback(void);
static AT_status_t _CLI_rcc_callback(void);
static AT_status_t _CLI_get_ep_id_callback(void);
static AT_status_t _CLI_set_ep_id_callback(void);
static AT_status_t _CLI_get_ep_key_callback(void);
static AT_status_t _CLI_set_ep_key_callback(void);
static AT_status_t _CLI_adc_callback(void);
static AT_status_t _CLI_iths_callback(void);
#ifdef HW2_0
static AT_status_t _CLI_eths_callback(void);
#endif
static AT_status_t _CLI_epts_callback(void);
static AT_status_t _CLI_euvs_callback(void);
static AT_status_t _CLI_time_callback(void);
static AT_status_t _CLI_gps_callback(void);
#ifdef SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE
static AT_status_t _CLI_so_callback(void);
#endif
static AT_status_t _CLI_sb_callback(void);
static AT_status_t _CLI_sf_callback(void);
static AT_status_t _CLI_tm_callback(void);
static AT_status_t _CLI_cw_callback(void);
#ifdef SIGFOX_EP_BIDIRECTIONAL
static AT_status_t _CLI_rssi_callback(void);
#endif

/*** CLI local global variables ***/

static const AT_command_t CLI_COMMANDS_LIST[] = {
    {
        .syntax = "Z",
        .parameters = NULL,
        .description = "Reset MCU",
        .callback = &_CLI_z_callback
    },
    {
        .syntax = "$RCC?",
        .parameters = NULL,
        .description = "Get clocks frequency",
        .callback = &_CLI_rcc_callback
    },
    {
        .syntax = "$ID?",
        .parameters = NULL,
        .description = "Get Sigfox EP ID",
        .callback = &_CLI_get_ep_id_callback
    },
    {
        .syntax = "$ID=",
        .parameters = "<id[hex]>",
        .description = "Set Sigfox EP ID",
        .callback = &_CLI_set_ep_id_callback
    },
    {
        .syntax = "$KEY?",
        .parameters = NULL,
        .description = "Get Sigfox EP key",
        .callback = &_CLI_get_ep_key_callback
    },
    {
        .syntax = "$KEY=",
        .parameters = "<key[hex]>",
        .description = "Set Sigfox EP key",
        .callback = &_CLI_set_ep_key_callback
    },
    {
        .syntax = "$ADC?",
        .parameters = NULL,
        .description = "Read analog measurements",
        .callback = &_CLI_adc_callback
    },
    {
        .syntax = "$ITHS?",
        .parameters = NULL,
        .description = "Read internal PCB temperature and humidity",
        .callback = &_CLI_iths_callback
    },
#ifdef HW2_0
    {
        .syntax = "$ETHS?",
        .parameters = NULL,
        .description = "Read external temperature and humidity",
        .callback = &_CLI_eths_callback
    },
#endif
    {
        .syntax = "$EPTS?",
        .parameters = NULL,
        .description = "Read pressure and temperature",
        .callback = &_CLI_epts_callback
    },
    {
        .syntax = "$EUVS?",
        .parameters = NULL,
        .description = "Read UV index",
        .callback = &_CLI_euvs_callback
    },
    {
        .syntax = "$TIME=",
        .parameters = "<timeout[s]>",
        .description = "Get GPS time",
        .callback = &_CLI_time_callback
    },
    {
        .syntax = "$GPS=",
        .parameters = "<timeout[s]>",
        .description = "Get GPS position",
        .callback = &_CLI_gps_callback
    },
#ifdef SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE
    {
        .syntax = "$SO",
        .parameters = NULL,
        .description = "Sigfox send control keep-alive",
        .callback = &_CLI_so_callback
    },
#endif
    {
        .syntax = "$SB=",
        .parameters = "<data[bit]>,(<bidir_flag[bit]>)",
        .description = "Sigfox send bit",
        .callback = &_CLI_sb_callback
    },
    {
        .syntax = "$SF=",
        .parameters = "<data[hex]>,(<bidir_flag[bit]>)",
        .description = "Sigfox send frame",
        .callback = &_CLI_sf_callback
    },
    {
        .syntax = "$TM=",
        .parameters = "<bit_rate_index[dec]>,<test_mode_reference[dec]>",
        .description = "Sigfox RFP test mode",
        .callback = _CLI_tm_callback
    },
    {
        .syntax = "$CW=",
        .parameters = "<frequency[hz]>,<enable[bit]>,(<output_power[dbm]>)",
        .description = "Continuous wave",
        .callback = &_CLI_cw_callback
    },
#ifdef SIGFOX_EP_BIDIRECTIONAL
    {
        .syntax = "$RSSI=",
        .parameters = "<frequency[hz]>,<duration[s]>",
        .description = "Continuous RSSI measurement",
        .callback = &_CLI_rssi_callback
    }
#endif
};

static CLI_context_t cli_ctx = {
    .at_process_flag = 0,
    .at_parser_ptr = NULL
};

/*** CLI local functions ***/

/*******************************************************************/
#define _CLI_check_driver_status(driver_status, driver_success, driver_error_base) { \
    /* Check value */ \
    if (driver_status != driver_success) { \
        /* Stack error */ \
        ERROR_stack_add((ERROR_code_t) (driver_error_base + driver_status)); \
        /* Exit with execution error */ \
        status = AT_ERROR_COMMAND_EXECUTION; \
        goto errors; \
    } \
}

/*******************************************************************/
static void _CLI_at_process_callback(void) {
    // Set local flag.
    cli_ctx.at_process_flag = 1;
}

/*******************************************************************/
static AT_status_t _CLI_z_callback(void) {
    // Local variables.
    AT_status_t status = AT_SUCCESS;
    // Execute command.
    PWR_software_reset();
    return status;
}

/*******************************************************************/
static AT_status_t _CLI_rcc_callback(void) {
    // Local variables.
    AT_status_t status = AT_SUCCESS;
    RCC_status_t rcc_status = RCC_SUCCESS;
    char_t* rcc_clock_name[RCC_CLOCK_LAST - 1] = { "SYS", "HSI", "MSI", "HSE", "PLL", "LSI", "LSE" };
    uint8_t rcc_clock_index = 0;
    uint8_t clock_status = 0;
    uint32_t clock_frequency = 0;
    uint8_t idx = 0;
    // Calibrate clocks.
    rcc_status = RCC_calibrate_internal_clocks(NVIC_PRIORITY_CLOCK_CALIBRATION);
    _CLI_check_driver_status(rcc_status, RCC_SUCCESS, ERROR_BASE_RCC);
    // Clocks loop.
    for (idx = 0; idx < (RCC_CLOCK_LAST - 1); idx++) {
        // Convert to RCC clock index.
        rcc_clock_index = (RCC_CLOCK_NONE + 1 + idx);
        // Read status.
        rcc_status = RCC_get_status(rcc_clock_index, &clock_status);
        _CLI_check_driver_status(rcc_status, RCC_SUCCESS, ERROR_BASE_RCC);
        // Read frequency.
        rcc_status = RCC_get_frequency_hz(rcc_clock_index, &clock_frequency);
        _CLI_check_driver_status(rcc_status, RCC_SUCCESS, ERROR_BASE_RCC);
        // Print data.
        AT_reply_add_string(rcc_clock_name[idx]);
        AT_reply_add_string((clock_status == 0) ? ":OFF:" : ":ON:");
        AT_reply_add_integer((int32_t) clock_frequency, STRING_FORMAT_DECIMAL, 0);
        AT_reply_add_string("Hz");
        AT_send_reply();
    }
errors:
    return status;
}

/*******************************************************************/
static AT_status_t _CLI_get_ep_id_callback(void) {
    // Local variables.
    AT_status_t status = AT_SUCCESS;
    NVM_status_t nvm_status = NVM_SUCCESS;
    uint8_t idx = 0;
    uint8_t id_byte = 0;
    // Retrieve device ID in NVM.
    for (idx = 0; idx < SIGFOX_EP_ID_SIZE_BYTES; idx++) {
        nvm_status = NVM_read_byte((NVM_ADDRESS_SIGFOX_EP_ID + idx), &id_byte);
        _CLI_check_driver_status(nvm_status, NVM_SUCCESS, ERROR_BASE_NVM);
        AT_reply_add_integer(id_byte, STRING_FORMAT_HEXADECIMAL, 0);
    }
    AT_send_reply();
errors:
    return status;
}

/*******************************************************************/
static AT_status_t _CLI_set_ep_id_callback(void) {
    // Local variables.
    AT_status_t status = AT_SUCCESS;
    PARSER_status_t parser_status = PARSER_SUCCESS;
    NVM_status_t nvm_status = NVM_SUCCESS;
    uint8_t sigfox_ep_id[SIGFOX_EP_ID_SIZE_BYTES];
    uint32_t unused = 0;
    uint8_t idx = 0;
    // Read ID parameter.
    parser_status = PARSER_get_byte_array(cli_ctx.at_parser_ptr, STRING_CHAR_NULL, SIGFOX_EP_ID_SIZE_BYTES, 1, sigfox_ep_id, &unused);
    PARSER_exit_error(AT_ERROR_BASE_PARSER);
    // Write device ID in NVM.
    for (idx = 0; idx < SIGFOX_EP_ID_SIZE_BYTES; idx++) {
        nvm_status = NVM_write_byte((NVM_ADDRESS_SIGFOX_EP_ID + idx), sigfox_ep_id[idx]);
        _CLI_check_driver_status(nvm_status, NVM_SUCCESS, ERROR_BASE_NVM);
    }
errors:
    return status;
}

/*******************************************************************/
static AT_status_t _CLI_get_ep_key_callback(void) {
    // Local variables.
    AT_status_t status = AT_SUCCESS;
    NVM_status_t nvm_status = NVM_SUCCESS;
    uint8_t idx = 0;
    uint8_t key_byte = 0;
    // Retrieve device key in NVM.
    for (idx = 0; idx < SIGFOX_EP_KEY_SIZE_BYTES; idx++) {
        nvm_status = NVM_read_byte((NVM_ADDRESS_SIGFOX_EP_KEY + idx), &key_byte);
        _CLI_check_driver_status(nvm_status, NVM_SUCCESS, ERROR_BASE_NVM);
        AT_reply_add_integer(key_byte, STRING_FORMAT_HEXADECIMAL, 0);
    }
    AT_send_reply();
errors:
    return status;
}

/*******************************************************************/
static AT_status_t _CLI_set_ep_key_callback(void) {
    // Local variables.
    AT_status_t status = AT_SUCCESS;
    PARSER_status_t parser_status = PARSER_SUCCESS;
    NVM_status_t nvm_status = NVM_SUCCESS;
    uint8_t sigfox_ep_key[SIGFOX_EP_KEY_SIZE_BYTES];
    uint32_t unused = 0;
    uint8_t idx = 0;
    // Read key parameter.
    parser_status = PARSER_get_byte_array(cli_ctx.at_parser_ptr, STRING_CHAR_NULL, SIGFOX_EP_KEY_SIZE_BYTES, 1, sigfox_ep_key, &unused);
    PARSER_exit_error(AT_ERROR_BASE_PARSER);
    // Write device ID in NVM.
    for (idx = 0; idx < SIGFOX_EP_KEY_SIZE_BYTES; idx++) {
        nvm_status = NVM_write_byte((NVM_ADDRESS_SIGFOX_EP_KEY + idx), sigfox_ep_key[idx]);
        _CLI_check_driver_status(nvm_status, NVM_SUCCESS, ERROR_BASE_NVM);
    }
errors:
    return status;
}

/*******************************************************************/
static AT_status_t _CLI_adc_callback(void) {
    // Local variables.
    AT_status_t status = AT_SUCCESS;
    ANALOG_status_t analog_status = ANALOG_SUCCESS;
    int32_t generic_s32 = 0;
    // Turn analog front-end on.
    POWER_enable(POWER_REQUESTER_ID_CLI, POWER_DOMAIN_ANALOG, LPTIM_DELAY_MODE_ACTIVE);
    // MCU voltage.
    analog_status = ANALOG_convert_channel(ANALOG_CHANNEL_VMCU_MV, &generic_s32);
    _CLI_check_driver_status(analog_status, ANALOG_SUCCESS, ERROR_BASE_ANALOG);
    AT_reply_add_string("Vmcu=");
    AT_reply_add_integer(generic_s32, STRING_FORMAT_DECIMAL, 0);
    AT_reply_add_string("mV");
    AT_send_reply();
    // MCU temperature.
    analog_status = ANALOG_convert_channel(ANALOG_CHANNEL_TMCU_DEGREES, &generic_s32);
    _CLI_check_driver_status(analog_status, ANALOG_SUCCESS, ERROR_BASE_ANALOG);
    AT_reply_add_string("Tmcu=");
    AT_reply_add_integer(generic_s32, STRING_FORMAT_DECIMAL, 0);
    AT_reply_add_string("dC");
    AT_send_reply();
    // Source voltage.
    AT_reply_add_string("Vpv=");
    analog_status = ANALOG_convert_channel(ANALOG_CHANNEL_VPV_MV, &generic_s32);
    _CLI_check_driver_status(analog_status, ANALOG_SUCCESS, ERROR_BASE_ANALOG);
    AT_reply_add_integer(generic_s32, STRING_FORMAT_DECIMAL, 0);
    AT_reply_add_string("mV");
    AT_send_reply();
    // Supercap voltage.
    AT_reply_add_string("Vcap=");
    analog_status = ANALOG_convert_channel(ANALOG_CHANNEL_VCAP_MV, &generic_s32);
    _CLI_check_driver_status(analog_status, ANALOG_SUCCESS, ERROR_BASE_ANALOG);
    AT_reply_add_integer(generic_s32, STRING_FORMAT_DECIMAL, 0);
    AT_reply_add_string("mV");
    AT_send_reply();
    // Light.
    AT_reply_add_string("Light=");
    analog_status = ANALOG_convert_channel(ANALOG_CHANNEL_LDR_PERCENT, &generic_s32);
    _CLI_check_driver_status(analog_status, ANALOG_SUCCESS, ERROR_BASE_ANALOG);
    AT_reply_add_integer(generic_s32, STRING_FORMAT_DECIMAL, 0);
    AT_reply_add_string("%");
    AT_send_reply();
errors:
    POWER_disable(POWER_REQUESTER_ID_CLI, POWER_DOMAIN_ANALOG);
    return status;
}

/*******************************************************************/
static AT_status_t _CLI_iths_callback(void) {
    // Local variables.
    AT_status_t status = AT_SUCCESS;
    SHT3X_status_t sht3x_status = SHT3X_SUCCESS;
    int32_t temperature_tenth_degrees = 0;
    char_t temperature_str[CLI_TEMPERATURE_STRING_SIZE] = { STRING_CHAR_NULL };
    int32_t humidity_percent = 0;
    // Turn digital sensors on.
    POWER_enable(POWER_REQUESTER_ID_CLI, POWER_DOMAIN_SENSORS, LPTIM_DELAY_MODE_SLEEP);
    // Perform measurements.
    sht3x_status = SHT3X_get_temperature_humidity(I2C_ADDRESS_SHT30_INTERNAL, &temperature_tenth_degrees, &humidity_percent);
    _CLI_check_driver_status(sht3x_status, SHT3X_SUCCESS, ERROR_BASE_SHT30_INTERNAL);
    // Read and print data.
    // Temperature.
    STRING_integer_to_floating_decimal_string(temperature_tenth_degrees, 1, (CLI_TEMPERATURE_STRING_SIZE - 1), (char_t*) temperature_str);
    AT_reply_add_string("Tpcb=");
    AT_reply_add_string(temperature_str);
    AT_reply_add_string("dC");
    AT_send_reply();
    // Humidity.
    AT_reply_add_string("Hpcb=");
    AT_reply_add_integer(humidity_percent, STRING_FORMAT_DECIMAL, 0);
    AT_reply_add_string("%");
    AT_send_reply();
errors:
    POWER_disable(POWER_REQUESTER_ID_CLI, POWER_DOMAIN_SENSORS);
    return status;
}

#ifdef HW2_0
/*******************************************************************/
static AT_status_t _CLI_eths_callback(void) {
    // Local variables.
    AT_status_t status = AT_SUCCESS;
    SHT3X_status_t sht3x_status = SHT3X_SUCCESS;
    int32_t temperature_tenth_degrees = 0;
    char_t temperature_str[CLI_TEMPERATURE_STRING_SIZE] = { STRING_CHAR_NULL };
    int32_t humidity_percent = 0;
    // Turn digital sensors on.
    POWER_enable(POWER_REQUESTER_ID_CLI, POWER_DOMAIN_SENSORS, LPTIM_DELAY_MODE_SLEEP);
    // Perform measurements.
    sht3x_status = SHT3X_get_temperature_humidity(I2C_ADDRESS_SHT30_EXTERNAL, &temperature_tenth_degrees, &humidity_percent);
    _CLI_check_driver_status(sht3x_status, SHT3X_SUCCESS, ERROR_BASE_SHT30_EXTERNAL);
    // Read and print data.
    // Temperature.
    STRING_integer_to_floating_decimal_string(temperature_tenth_degrees, 1, (CLI_TEMPERATURE_STRING_SIZE - 1), (char_t*) temperature_str);
    AT_reply_add_string("Tamb=");
    AT_reply_add_string(temperature_str);
    AT_reply_add_string("dC");
    AT_send_reply();
    // Humidity.
    AT_reply_add_string("Hamb=");
    AT_reply_add_integer(humidity_percent, STRING_FORMAT_DECIMAL, 0);
    AT_reply_add_string("%");
    AT_send_reply();
errors:
    POWER_disable(POWER_REQUESTER_ID_CLI, POWER_DOMAIN_SENSORS);
    return status;
}
#endif

/*******************************************************************/
static AT_status_t _CLI_epts_callback(void) {
    // Local variables.
    AT_status_t status = AT_SUCCESS;
    DPS310_status_t dps310_status = DPS310_SUCCESS;
    int32_t pressure_pa = 0;
    int32_t temperature_tenth_degrees = 0;
    char_t temperature_str[CLI_TEMPERATURE_STRING_SIZE] = { STRING_CHAR_NULL };
    // Turn digital sensors on.
    POWER_enable(POWER_REQUESTER_ID_CLI, POWER_DOMAIN_SENSORS, LPTIM_DELAY_MODE_SLEEP);
    // Perform measurements.
    dps310_status = DPS310_get_pressure_temperature(I2C_ADDRESS_DPS310, &pressure_pa, &temperature_tenth_degrees);
    _CLI_check_driver_status(dps310_status, DPS310_SUCCESS, ERROR_BASE_DPS310);
    // Read and print data.
    // Pressure.
    AT_reply_add_string("Pabs=");
    AT_reply_add_integer(pressure_pa, STRING_FORMAT_DECIMAL, 0);
    AT_reply_add_string("Pa");
    AT_send_reply();
    // Temperature.
    STRING_integer_to_floating_decimal_string(temperature_tenth_degrees, 1, (CLI_TEMPERATURE_STRING_SIZE - 1), (char_t*) temperature_str);
    AT_reply_add_string("Tamb=");
    AT_reply_add_string(temperature_str);
    AT_reply_add_string("dC");
    AT_send_reply();
errors:
    POWER_disable(POWER_REQUESTER_ID_CLI, POWER_DOMAIN_SENSORS);
    return status;
}

/*******************************************************************/
static AT_status_t _CLI_euvs_callback(void) {
    // Local variables.
    AT_status_t status = AT_SUCCESS;
    SI1133_status_t si1133_status = SI1133_SUCCESS;
    int32_t uv_index = 0;
    // Turn digital sensors on.
    POWER_enable(POWER_REQUESTER_ID_CLI, POWER_DOMAIN_SENSORS, LPTIM_DELAY_MODE_SLEEP);
    // Perform measurements.
    si1133_status = SI1133_get_uv_index(I2C_ADDRESS_SI1133, &uv_index);
    _CLI_check_driver_status(si1133_status, SI1133_SUCCESS, ERROR_BASE_SI1133);
    // Read and print data.
    AT_reply_add_string("UVI=");
    AT_reply_add_integer(uv_index, STRING_FORMAT_DECIMAL, 0);
    AT_send_reply();
errors:
    POWER_disable(POWER_REQUESTER_ID_CLI, POWER_DOMAIN_SENSORS);
    return status;
}

/*******************************************************************/
static AT_status_t _CLI_time_callback(void) {
    // Local variables.
    AT_status_t status = AT_SUCCESS;
    PARSER_status_t parser_status = PARSER_SUCCESS;
    GPS_status_t gps_status = GPS_SUCCESS;
    GPS_acquisition_status_t acquisition_status = GPS_ACQUISITION_ERROR_LAST;
    GPS_time_t gps_time;
    int32_t timeout_seconds = 0;
    uint32_t fix_duration_seconds = 0;
    // Read timeout parameter.
    parser_status = PARSER_get_parameter(cli_ctx.at_parser_ptr, STRING_FORMAT_DECIMAL, STRING_CHAR_NULL, &timeout_seconds);
    PARSER_exit_error(AT_ERROR_BASE_PARSER);
    // Turn GPS on.
    POWER_enable(POWER_REQUESTER_ID_CLI, POWER_DOMAIN_GPS, LPTIM_DELAY_MODE_SLEEP);
    // Perform time acquisition.
    gps_status = GPS_get_time(&gps_time, (uint32_t) timeout_seconds, &fix_duration_seconds, &acquisition_status);
    _CLI_check_driver_status(gps_status, GPS_SUCCESS, ERROR_BASE_GPS);
    // Check status.
    if (acquisition_status == GPS_ACQUISITION_SUCCESS) {
        // Year.
        AT_reply_add_integer((int32_t) (gps_time.year), STRING_FORMAT_DECIMAL, 0);
        AT_reply_add_string("-");
        // Month.
        if ((gps_time.month) < 10) {
            AT_reply_add_integer(0, STRING_FORMAT_DECIMAL, 0);
        }
        AT_reply_add_integer((int32_t) (gps_time.month), STRING_FORMAT_DECIMAL, 0);
        AT_reply_add_string("-");
        // Day.
        if ((gps_time.date) < 10) {
            AT_reply_add_integer(0, STRING_FORMAT_DECIMAL, 0);
        }
        AT_reply_add_integer((int32_t) (gps_time.date), STRING_FORMAT_DECIMAL, 0);
        AT_reply_add_string(":");
        // Hours.
        if ((gps_time.hours) < 10) {
            AT_reply_add_integer(0, STRING_FORMAT_DECIMAL, 0);
        }
        AT_reply_add_integer((int32_t) (gps_time.hours), STRING_FORMAT_DECIMAL, 0);
        AT_reply_add_string(".");
        // Minutes.
        if ((gps_time.minutes) < 10) {
            AT_reply_add_integer(0, STRING_FORMAT_DECIMAL, 0);
        }
        AT_reply_add_integer((int32_t) (gps_time.minutes), STRING_FORMAT_DECIMAL, 0);
        AT_reply_add_string(".");
        // Seconds.
        if ((gps_time.seconds) < 10) {
            AT_reply_add_integer(0, STRING_FORMAT_DECIMAL, 0);
        }
        AT_reply_add_integer((int32_t) (gps_time.seconds), STRING_FORMAT_DECIMAL, 0);
        AT_reply_add_string(":");
    }
    else {
        AT_reply_add_string("TIMEOUT:");
    }
    AT_reply_add_integer((int32_t) fix_duration_seconds, STRING_FORMAT_DECIMAL, 0);
    AT_reply_add_string("s");
    AT_send_reply();
errors:
    POWER_disable(POWER_REQUESTER_ID_CLI, POWER_DOMAIN_GPS);
    return status;
}

/*******************************************************************/
static AT_status_t _CLI_gps_callback(void) {
    // Local variables.
    AT_status_t status = AT_SUCCESS;
    PARSER_status_t parser_status = PARSER_SUCCESS;
    GPS_status_t gps_status = GPS_SUCCESS;
    GPS_position_t gps_position;
    GPS_acquisition_status_t acquisition_status = GPS_ACQUISITION_ERROR_LAST;
    int32_t timeout_seconds = 0;
    uint32_t fix_duration_seconds = 0;
    // Read timeout parameter.
    parser_status = PARSER_get_parameter(cli_ctx.at_parser_ptr, STRING_FORMAT_DECIMAL, STRING_CHAR_NULL, &timeout_seconds);
    PARSER_exit_error(AT_ERROR_BASE_PARSER);
    // Turn GPS on.
    POWER_enable(POWER_REQUESTER_ID_CLI, POWER_DOMAIN_GPS, LPTIM_DELAY_MODE_SLEEP);
    // Perform time acquisition.
    gps_status = GPS_get_position(&gps_position, (uint32_t) timeout_seconds, &fix_duration_seconds, &acquisition_status);
    _CLI_check_driver_status(gps_status, GPS_SUCCESS, ERROR_BASE_GPS);
    // Check status.
    if (acquisition_status == GPS_ACQUISITION_SUCCESS) {
        // Latitude.
        AT_reply_add_integer((gps_position.lat_degrees), STRING_FORMAT_DECIMAL, 0);
        AT_reply_add_string("d");
        AT_reply_add_integer((gps_position.lat_minutes), STRING_FORMAT_DECIMAL, 0);
        AT_reply_add_string("'");
        AT_reply_add_integer((gps_position.lat_seconds), STRING_FORMAT_DECIMAL, 0);
        AT_reply_add_string("''");
        AT_reply_add_string(((gps_position.lat_north_flag) == 0) ? "S" : "N");
        // Longitude.
        AT_reply_add_string(":");
        AT_reply_add_integer((gps_position.long_degrees), STRING_FORMAT_DECIMAL, 0);
        AT_reply_add_string("d");
        AT_reply_add_integer((gps_position.long_minutes), STRING_FORMAT_DECIMAL, 0);
        AT_reply_add_string("'");
        AT_reply_add_integer((gps_position.long_seconds), STRING_FORMAT_DECIMAL, 0);
        AT_reply_add_string("''");
        AT_reply_add_string(((gps_position.long_east_flag) == 0) ? "W" : "E");
        // Altitude.
        AT_reply_add_string(":");
        AT_reply_add_integer((gps_position.altitude), STRING_FORMAT_DECIMAL, 0);
        AT_reply_add_string("m:");
    }
    else {
        AT_reply_add_string("TIMEOUT:");
    }
    AT_reply_add_integer((int32_t) fix_duration_seconds, STRING_FORMAT_DECIMAL, 0);
    AT_reply_add_string("s");
    AT_send_reply();
errors:
    POWER_disable(POWER_REQUESTER_ID_CLI, POWER_DOMAIN_GPS);
    return status;
}

#ifdef SIGFOX_EP_BIDIRECTIONAL
/*******************************************************************/
static void _CLI_print_dl_payload(sfx_u8* dl_payload, sfx_u8 dl_payload_size, sfx_s16 rssi_dbm) {
    // Local variables.
    uint8_t idx = 0;
    // Print DL payload.
    AT_reply_add_string("+RX=");
    for (idx = 0; idx < dl_payload_size; idx++) {
        AT_reply_add_integer(dl_payload[idx], STRING_FORMAT_HEXADECIMAL, 0);
    }
    AT_reply_add_string(":");
    AT_reply_add_integer(rssi_dbm, STRING_FORMAT_DECIMAL, 0);
    AT_reply_add_string("dBm");
    AT_send_reply();
}
#endif

#ifdef SIGFOX_EP_BIDIRECTIONAL
/*******************************************************************/
static AT_status_t _CLI_read_print_dl_payload(void) {
    // Local variables.
    AT_status_t status = AT_SUCCESS;
    SIGFOX_EP_API_status_t sigfox_ep_api_status = SIGFOX_EP_API_SUCCESS;
    SIGFOX_EP_API_message_status_t message_status;
    sfx_u8 dl_payload[SIGFOX_DL_PAYLOAD_SIZE_BYTES];
    sfx_s16 dl_rssi_dbm = 0;
    // Get message status.
    message_status = SIGFOX_EP_API_get_message_status();
    // Check downlink status.
    if (message_status.field.dl_frame == 0) {
        AT_reply_add_string("+RX=timeout");
        AT_send_reply();
    }
    else {
        // Read downlink payload.
        sigfox_ep_api_status = SIGFOX_EP_API_get_dl_payload(dl_payload, SIGFOX_DL_PAYLOAD_SIZE_BYTES, &dl_rssi_dbm);
        _CLI_check_driver_status(sigfox_ep_api_status, SIGFOX_EP_API_SUCCESS, (ERROR_BASE_SIGFOX_EP_LIB + (SIGFOX_ERROR_SOURCE_SIGFOX_EP_API * ERROR_BASE_STEP)));
        // Print downlink payload.
        _CLI_print_dl_payload(dl_payload, SIGFOX_DL_PAYLOAD_SIZE_BYTES, dl_rssi_dbm);
    }
errors:
    return status;
}
#endif

#ifdef SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE
/*******************************************************************/
static AT_status_t _CLI_so_callback(void) {
    // Local variables.
    AT_status_t status = AT_SUCCESS;
    SIGFOX_EP_API_status_t sigfox_ep_api_status = SIGFOX_EP_API_SUCCESS;
    SIGFOX_EP_API_config_t lib_config;
    SIGFOX_EP_API_control_message_t control_message;
    // Library configuration.
    lib_config.rc = &SIGFOX_RC1;
    // Default control message parameters.
    control_message.type = SIGFOX_CONTROL_MESSAGE_TYPE_KEEP_ALIVE;
    control_message.common_parameters.number_of_frames = 3;
    control_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_100BPS;
    // Open library.
    sigfox_ep_api_status = SIGFOX_EP_API_open(&lib_config);
    _CLI_check_driver_status(sigfox_ep_api_status, SIGFOX_EP_API_SUCCESS, (ERROR_BASE_SIGFOX_EP_LIB + (SIGFOX_ERROR_SOURCE_SIGFOX_EP_API * ERROR_BASE_STEP)));
    // Send application message.
    sigfox_ep_api_status = SIGFOX_EP_API_send_control_message(&control_message);
    _CLI_check_driver_status(sigfox_ep_api_status, SIGFOX_EP_API_SUCCESS, (ERROR_BASE_SIGFOX_EP_LIB + (SIGFOX_ERROR_SOURCE_SIGFOX_EP_API * ERROR_BASE_STEP)));
    // Close library.
    sigfox_ep_api_status = SIGFOX_EP_API_close();
    _CLI_check_driver_status(sigfox_ep_api_status, SIGFOX_EP_API_SUCCESS, (ERROR_BASE_SIGFOX_EP_LIB + (SIGFOX_ERROR_SOURCE_SIGFOX_EP_API * ERROR_BASE_STEP)));
    goto end;
errors:
    SIGFOX_EP_API_close();
end:
    return status;
}
#endif

/*******************************************************************/
static AT_status_t _CLI_sb_callback(void) {
    // Local variables.
    AT_status_t status = AT_SUCCESS;
    PARSER_status_t parser_status = PARSER_SUCCESS;
    SIGFOX_EP_API_status_t sigfox_ep_api_status = SIGFOX_EP_API_SUCCESS;
    SIGFOX_EP_API_config_t lib_config;
    SIGFOX_EP_API_application_message_t application_message;
    int32_t ul_bit = 0;
    int32_t bidir_flag = 0;
    // Library configuration.
    lib_config.rc = &SIGFOX_RC1;
    // Default application message parameters.
    application_message.common_parameters.number_of_frames = 3;
    application_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_100BPS;
    application_message.ul_payload = SIGFOX_NULL;
    application_message.ul_payload_size_bytes = 0;
    // First try with 2 parameters.
    parser_status = PARSER_get_parameter(cli_ctx.at_parser_ptr, STRING_FORMAT_BOOLEAN, CLI_CHAR_SEPARATOR, &ul_bit);
    if (parser_status == PARSER_SUCCESS) {
        // Try parsing downlink request parameter.
        parser_status = PARSER_get_parameter(cli_ctx.at_parser_ptr, STRING_FORMAT_BOOLEAN, STRING_CHAR_NULL, &bidir_flag);
        PARSER_exit_error(AT_ERROR_BASE_PARSER);
        // Update parameters.
        application_message.type = (SIGFOX_APPLICATION_MESSAGE_TYPE_BIT0 + ul_bit);
#ifdef SIGFOX_EP_BIDIRECTIONAL
        application_message.bidirectional_flag = bidir_flag;
#endif
    }
    else {
        // Try with 1 parameter.
        parser_status = PARSER_get_parameter(cli_ctx.at_parser_ptr, STRING_FORMAT_BOOLEAN, STRING_CHAR_NULL, &ul_bit);
        PARSER_exit_error(AT_ERROR_BASE_PARSER);
        // Update parameters.
        application_message.type = (SIGFOX_APPLICATION_MESSAGE_TYPE_BIT0 + ul_bit);
#ifdef SIGFOX_EP_BIDIRECTIONAL
        application_message.bidirectional_flag = 0;
#endif
    }
    // Open library.
    sigfox_ep_api_status = SIGFOX_EP_API_open(&lib_config);
    _CLI_check_driver_status(sigfox_ep_api_status, SIGFOX_EP_API_SUCCESS, (ERROR_BASE_SIGFOX_EP_LIB + (SIGFOX_ERROR_SOURCE_SIGFOX_EP_API * ERROR_BASE_STEP)));
    // Send application message.
    sigfox_ep_api_status = SIGFOX_EP_API_send_application_message(&application_message);
    _CLI_check_driver_status(sigfox_ep_api_status, SIGFOX_EP_API_SUCCESS, (ERROR_BASE_SIGFOX_EP_LIB + (SIGFOX_ERROR_SOURCE_SIGFOX_EP_API * ERROR_BASE_STEP)));
#ifdef SIGFOX_EP_BIDIRECTIONAL
    // Read and print DL payload if needed.
    if ((application_message.bidirectional_flag) == SIGFOX_TRUE) {
        status = _CLI_read_print_dl_payload();
        if (status != AT_SUCCESS) goto errors;
    }
#endif
    // Close library.
    sigfox_ep_api_status = SIGFOX_EP_API_close();
    _CLI_check_driver_status(sigfox_ep_api_status, SIGFOX_EP_API_SUCCESS, (ERROR_BASE_SIGFOX_EP_LIB + (SIGFOX_ERROR_SOURCE_SIGFOX_EP_API * ERROR_BASE_STEP)));
    goto end;
errors:
    SIGFOX_EP_API_close();
end:
    return status;
}

/*******************************************************************/
static AT_status_t _CLI_sf_callback(void) {
    // Local variables.
    AT_status_t status = AT_SUCCESS;
    PARSER_status_t parser_status = PARSER_SUCCESS;
    SIGFOX_EP_API_status_t sigfox_ep_api_status = SIGFOX_EP_API_SUCCESS;
    SIGFOX_EP_API_config_t lib_config;
    SIGFOX_EP_API_application_message_t application_message;
    sfx_u8 data[SIGFOX_UL_PAYLOAD_MAX_SIZE_BYTES];
    uint32_t extracted_size = 0;
    int32_t bidir_flag = 0;
    // Library configuration.
    lib_config.rc = &SIGFOX_RC1;
    // Default application message parameters.
    application_message.common_parameters.number_of_frames = 3;
    application_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_100BPS;
    application_message.type = SIGFOX_APPLICATION_MESSAGE_TYPE_BYTE_ARRAY;
#ifdef SIGFOX_EP_BIDIRECTIONAL
    application_message.bidirectional_flag = 0;
#endif
    application_message.ul_payload = SIGFOX_NULL;
    application_message.ul_payload_size_bytes = 0;
    // First try with 2 parameters.
    parser_status = PARSER_get_byte_array(cli_ctx.at_parser_ptr, CLI_CHAR_SEPARATOR, SIGFOX_UL_PAYLOAD_MAX_SIZE_BYTES, 0, data, &extracted_size);
    if (parser_status == PARSER_SUCCESS) {
        // Try parsing downlink request parameter.
        parser_status = PARSER_get_parameter(cli_ctx.at_parser_ptr, STRING_FORMAT_BOOLEAN, STRING_CHAR_NULL, &bidir_flag);
        PARSER_exit_error(AT_ERROR_BASE_PARSER);
        // Update parameters.
        application_message.ul_payload = (sfx_u8*) data;
        application_message.ul_payload_size_bytes = extracted_size;
#ifdef SIGFOX_EP_BIDIRECTIONAL
        application_message.bidirectional_flag = bidir_flag;
#endif
    }
    else {
        // Try with 1 parameter.
        parser_status = PARSER_get_byte_array(cli_ctx.at_parser_ptr, STRING_CHAR_NULL, SIGFOX_UL_PAYLOAD_MAX_SIZE_BYTES, 0, data, &extracted_size);
        PARSER_exit_error(AT_ERROR_BASE_PARSER);
        // Update parameters.
        application_message.ul_payload = (sfx_u8*) data;
        application_message.ul_payload_size_bytes = extracted_size;
    }
    // Open library.
    sigfox_ep_api_status = SIGFOX_EP_API_open(&lib_config);
    _CLI_check_driver_status(sigfox_ep_api_status, SIGFOX_EP_API_SUCCESS, (ERROR_BASE_SIGFOX_EP_LIB + (SIGFOX_ERROR_SOURCE_SIGFOX_EP_API * ERROR_BASE_STEP)));
    // Send application message.
    sigfox_ep_api_status = SIGFOX_EP_API_send_application_message(&application_message);
    _CLI_check_driver_status(sigfox_ep_api_status, SIGFOX_EP_API_SUCCESS, (ERROR_BASE_SIGFOX_EP_LIB + (SIGFOX_ERROR_SOURCE_SIGFOX_EP_API * ERROR_BASE_STEP)));
#ifdef SIGFOX_EP_BIDIRECTIONAL
    // Read and print DL payload if needed.
    if ((application_message.bidirectional_flag) == SIGFOX_TRUE) {
        status = _CLI_read_print_dl_payload();
        if (status != AT_SUCCESS) goto errors;
    }
#endif
    // Close library.
    sigfox_ep_api_status = SIGFOX_EP_API_close();
    _CLI_check_driver_status(sigfox_ep_api_status, SIGFOX_EP_API_SUCCESS, (ERROR_BASE_SIGFOX_EP_LIB + (SIGFOX_ERROR_SOURCE_SIGFOX_EP_API * ERROR_BASE_STEP)));
    goto end;
errors:
    SIGFOX_EP_API_close();
end:
    return status;
}

/*******************************************************************/
static AT_status_t _CLI_tm_callback(void) {
    // Local variables.
    AT_status_t status = AT_SUCCESS;
    PARSER_status_t parser_status = PARSER_SUCCESS;
    SIGFOX_EP_ADDON_RFP_API_status_t sigfox_ep_addon_rfp_status = SIGFOX_EP_ADDON_RFP_API_SUCCESS;
    SIGFOX_EP_ADDON_RFP_API_config_t addon_config;
    SIGFOX_EP_ADDON_RFP_API_test_mode_t test_mode;
    int32_t bit_rate_index = 0;
    int32_t test_mode_reference = 0;
    // Read bit rate.
    parser_status = PARSER_get_parameter(cli_ctx.at_parser_ptr, STRING_FORMAT_DECIMAL, CLI_CHAR_SEPARATOR, &bit_rate_index);
    PARSER_exit_error(AT_ERROR_BASE_PARSER);
    // Read test mode parameter.
    parser_status = PARSER_get_parameter(cli_ctx.at_parser_ptr, STRING_FORMAT_DECIMAL, STRING_CHAR_NULL, &test_mode_reference);
    PARSER_exit_error(AT_ERROR_BASE_PARSER);
    // Addon configuration.
    addon_config.rc = &SIGFOX_RC1;
    // Test mode parameters.
    test_mode.test_mode_reference = (SIGFOX_EP_ADDON_RFP_API_test_mode_reference_t) test_mode_reference;
    test_mode.ul_bit_rate = (SIGFOX_ul_bit_rate_t) bit_rate_index;
#ifdef SIGFOX_EP_BIDIRECTIONAL
    test_mode.downlink_cplt_cb = &_CLI_print_dl_payload;
#endif
    // Open addon.
    sigfox_ep_addon_rfp_status = SIGFOX_EP_ADDON_RFP_API_open(&addon_config);
    _CLI_check_driver_status(sigfox_ep_addon_rfp_status, SIGFOX_EP_ADDON_RFP_API_SUCCESS, ERROR_BASE_SIGFOX_EP_ADDON_RFP);
    // Call test mode function.
    sigfox_ep_addon_rfp_status = SIGFOX_EP_ADDON_RFP_API_test_mode(&test_mode);
    _CLI_check_driver_status(sigfox_ep_addon_rfp_status, SIGFOX_EP_ADDON_RFP_API_SUCCESS, ERROR_BASE_SIGFOX_EP_ADDON_RFP);
    // Close addon.
    sigfox_ep_addon_rfp_status = SIGFOX_EP_ADDON_RFP_API_close();
    _CLI_check_driver_status(sigfox_ep_addon_rfp_status, SIGFOX_EP_ADDON_RFP_API_SUCCESS, ERROR_BASE_SIGFOX_EP_ADDON_RFP);
    goto end;
errors:
    SIGFOX_EP_ADDON_RFP_API_close();
end:
    return status;
}

/*******************************************************************/
static AT_status_t _CLI_cw_callback(void) {
    // Local variables.
    AT_status_t status = AT_SUCCESS;
    PARSER_status_t parser_status = PARSER_SUCCESS;
    RF_API_status_t rf_api_status = RF_API_SUCCESS;
    RF_API_radio_parameters_t radio_params;
    int32_t enable = 0;
    int32_t frequency_hz = 0;
    int32_t power_dbm = 0;
    // Set common radio parameters.
    radio_params.rf_mode = RF_API_MODE_TX;
    radio_params.modulation = RF_API_MODULATION_NONE;
    radio_params.bit_rate_bps = 0;
#ifdef SIGFOX_EP_BIDIRECTIONAL
    radio_params.deviation_hz = 0;
#endif
    // Read frequency parameter.
    parser_status = PARSER_get_parameter(cli_ctx.at_parser_ptr, STRING_FORMAT_DECIMAL, CLI_CHAR_SEPARATOR, &frequency_hz);
    PARSER_exit_error(AT_ERROR_BASE_PARSER);
    // Update radio configuration.
    radio_params.frequency_hz = (sfx_u32) frequency_hz;
    // First try with 3 parameters.
    parser_status = PARSER_get_parameter(cli_ctx.at_parser_ptr, STRING_FORMAT_BOOLEAN, CLI_CHAR_SEPARATOR, &enable);
    if (parser_status == PARSER_SUCCESS) {
        // There is a third parameter, try to parse power.
        parser_status = PARSER_get_parameter(cli_ctx.at_parser_ptr, STRING_FORMAT_DECIMAL, STRING_CHAR_NULL, &power_dbm);
        PARSER_exit_error(AT_ERROR_BASE_PARSER);
        // Update radio configuration.
        radio_params.tx_power_dbm_eirp = (sfx_s8) power_dbm;
    }
    else {
        // Power is not given, try to parse enable as last parameter.
        parser_status = PARSER_get_parameter(cli_ctx.at_parser_ptr, STRING_FORMAT_BOOLEAN, STRING_CHAR_NULL, &enable);
        PARSER_exit_error(AT_ERROR_BASE_PARSER);
        // Update radio configuration.
        radio_params.tx_power_dbm_eirp = SIGFOX_EP_TX_POWER_DBM_EIRP;
    }
    // Stop CW.
    rf_api_status = RF_API_de_init();
    _CLI_check_driver_status(rf_api_status, RF_API_SUCCESS, (ERROR_BASE_SIGFOX_EP_LIB + (SIGFOX_ERROR_SOURCE_RF_API * ERROR_BASE_STEP)));
    rf_api_status = RF_API_sleep();
    _CLI_check_driver_status(rf_api_status, RF_API_SUCCESS, (ERROR_BASE_SIGFOX_EP_LIB + (SIGFOX_ERROR_SOURCE_RF_API * ERROR_BASE_STEP)));
    // Restart if required.
    if (enable != 0) {
        // Init radio.
        rf_api_status = RF_API_wake_up();
        _CLI_check_driver_status(rf_api_status, RF_API_SUCCESS, (ERROR_BASE_SIGFOX_EP_LIB + (SIGFOX_ERROR_SOURCE_RF_API * ERROR_BASE_STEP)));
        rf_api_status = RF_API_init(&radio_params);
        _CLI_check_driver_status(rf_api_status, RF_API_SUCCESS, (ERROR_BASE_SIGFOX_EP_LIB + (SIGFOX_ERROR_SOURCE_RF_API * ERROR_BASE_STEP)));
        // Start CW.
        rf_api_status = RF_API_start_continuous_wave();
        _CLI_check_driver_status(rf_api_status, RF_API_SUCCESS, (ERROR_BASE_SIGFOX_EP_LIB + (SIGFOX_ERROR_SOURCE_RF_API * ERROR_BASE_STEP)));
    }
    goto end;
errors:
    RF_API_de_init();
    RF_API_sleep();
end:
    return status;
}

#ifdef SIGFOX_EP_BIDIRECTIONAL
/*******************************************************************/
static AT_status_t _CLI_rssi_callback(void) {
    // Local variables.
    AT_status_t status = AT_SUCCESS;
    PARSER_status_t parser_status = PARSER_SUCCESS;
    RF_API_status_t rf_api_status = RF_API_SUCCESS;
    SX1232_status_t sx1232_status = SX1232_SUCCESS;
    RFE_status_t rfe_status = RFE_SUCCESS;
    LPTIM_status_t lptim_status = LPTIM_SUCCESS;
    RF_API_radio_parameters_t radio_params;
    int32_t frequency_hz = 0;
    int32_t duration_seconds = 0;
    int16_t rssi_dbm = 0;
    uint32_t report_loop = 0;
    // Read frequency parameter.
    parser_status = PARSER_get_parameter(cli_ctx.at_parser_ptr, STRING_FORMAT_DECIMAL, CLI_CHAR_SEPARATOR, &frequency_hz);
    PARSER_exit_error(AT_ERROR_BASE_PARSER);
    // Read duration parameters.
    parser_status = PARSER_get_parameter(cli_ctx.at_parser_ptr, STRING_FORMAT_DECIMAL, STRING_CHAR_NULL, &duration_seconds);
    PARSER_exit_error(AT_ERROR_BASE_PARSER);
    // Radio configuration.
    radio_params.rf_mode = RF_API_MODE_RX;
    radio_params.frequency_hz = (sfx_u32) frequency_hz;
    radio_params.modulation = RF_API_MODULATION_NONE;
    radio_params.bit_rate_bps = 0;
    radio_params.tx_power_dbm_eirp = SIGFOX_EP_TX_POWER_DBM_EIRP;
    radio_params.deviation_hz = 0;
    // Init radio.
    rf_api_status = RF_API_wake_up();
    _CLI_check_driver_status(rf_api_status, RF_API_SUCCESS, (ERROR_BASE_SIGFOX_EP_LIB + (SIGFOX_ERROR_SOURCE_RF_API * ERROR_BASE_STEP)));
    rf_api_status = RF_API_init(&radio_params);
    _CLI_check_driver_status(rf_api_status, RF_API_SUCCESS, (ERROR_BASE_SIGFOX_EP_LIB + (SIGFOX_ERROR_SOURCE_RF_API * ERROR_BASE_STEP)));
    // Start continuous listening.
    sx1232_status = SX1232_start_rx();
    _CLI_check_driver_status(sx1232_status, SX1232_SUCCESS, ERROR_BASE_SX1232);
    // Measurement loop.
    while (report_loop < ((uint32_t) ((duration_seconds * 1000) / CLI_RSSI_REPORT_PERIOD_MS))) {
        // Read RSSI.
        rfe_status = RFE_get_rssi(&rssi_dbm);
        _CLI_check_driver_status(rfe_status, RFE_SUCCESS, ERROR_BASE_RFE);
        // Print RSSI.
        AT_reply_add_integer(rssi_dbm, STRING_FORMAT_DECIMAL, 0);
        AT_reply_add_string("dBm");
        AT_send_reply();
        // Report delay.
        lptim_status = LPTIM_delay_milliseconds(CLI_RSSI_REPORT_PERIOD_MS, LPTIM_DELAY_MODE_ACTIVE);
        _CLI_check_driver_status(lptim_status, LPTIM_SUCCESS, ERROR_BASE_LPTIM);
        report_loop++;
        // Reload watchdog.
        IWDG_reload();
    }
    // Turn radio off.
    rf_api_status = RF_API_de_init();
    _CLI_check_driver_status(rf_api_status, RF_API_SUCCESS, (ERROR_BASE_SIGFOX_EP_LIB + (SIGFOX_ERROR_SOURCE_RF_API * ERROR_BASE_STEP)));
    rf_api_status = RF_API_sleep();
    _CLI_check_driver_status(rf_api_status, RF_API_SUCCESS, (ERROR_BASE_SIGFOX_EP_LIB + (SIGFOX_ERROR_SOURCE_RF_API * ERROR_BASE_STEP)));
    goto end;
errors:
    RF_API_de_init();
    RF_API_sleep();
end:
    return status;
}
#endif

/*** CLI functions ***/

/*******************************************************************/
CLI_status_t CLI_init(void) {
    // Local variables.
    CLI_status_t status = CLI_SUCCESS;
    AT_status_t at_status = AT_SUCCESS;
    AT_configuration_t at_config;
    uint8_t idx = 0;
    // Init context.
    cli_ctx.at_process_flag = 0;
    cli_ctx.at_parser_ptr = NULL;
    // Init AT driver.
    at_config.terminal_instance = TERMINAL_INSTANCE_CLI;
    at_config.process_callback = &_CLI_at_process_callback;
    at_status = AT_init(&at_config, &(cli_ctx.at_parser_ptr));
    AT_exit_error(CLI_ERROR_BASE_AT);
    // Register commands.
    for (idx = 0; idx < (sizeof(CLI_COMMANDS_LIST) / sizeof(AT_command_t)); idx++) {
        at_status = AT_register_command(&(CLI_COMMANDS_LIST[idx]));
        AT_exit_error(CLI_ERROR_BASE_AT);
    }
errors:
    return status;
}

/*******************************************************************/
CLI_status_t CLI_de_init(void) {
    // Local variables.
    CLI_status_t status = CLI_SUCCESS;
    AT_status_t at_status = AT_SUCCESS;
    uint8_t idx = 0;
    // Unregister commands.
    for (idx = 0; idx < (sizeof(CLI_COMMANDS_LIST) / sizeof(AT_command_t)); idx++) {
        at_status = AT_unregister_command(&(CLI_COMMANDS_LIST[idx]));
        AT_stack_error(ERROR_BASE_CLI + CLI_ERROR_BASE_AT);
    }
    // Release AT driver.
    at_status = AT_de_init();
    AT_stack_error(ERROR_BASE_CLI + CLI_ERROR_BASE_AT);
    return status;
}

/*******************************************************************/
CLI_status_t CLI_process(void) {
    // Local variables.
    CLI_status_t status = CLI_SUCCESS;
    AT_status_t at_status = AT_SUCCESS;
    // Check process flag.
    if (cli_ctx.at_process_flag != 0) {
        // Clear flag.
        cli_ctx.at_process_flag = 0;
        // Process AT driver.
        at_status = AT_process();
        AT_exit_error(CLI_ERROR_BASE_AT);
    }
errors:
    return status;
}

#endif /* SPSWS_MODE_CLI */
