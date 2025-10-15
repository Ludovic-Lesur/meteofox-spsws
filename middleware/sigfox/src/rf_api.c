/*!*****************************************************************
 * \file    rf_api.c
 * \brief   Radio drivers.
 *******************************************************************
 * \copyright
 *
 * Copyright (c) 2022, UnaBiz SAS
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1 Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  2 Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  3 Neither the name of UnaBiz SAS nor the names of its contributors may be
 *    used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************/

#include "manuf/rf_api.h"

#ifndef SIGFOX_EP_DISABLE_FLAGS_FILE
#include "sigfox_ep_flags.h"
#endif
#include "sigfox_types.h"
#include "sigfox_error.h"

#include "error.h"
#include "error_base.h"
#include "exti.h"
#include "mcu_mapping.h"
#include "iwdg.h"
#include "manuf/mcu_api.h"
#include "nvic_priority.h"
#include "power.h"
#include "pwr.h"
#include "rfe.h"
#include "sx1232.h"
#include "types.h"

/*** RF API local macros ***/

#define RF_API_SLOW_BR_SYMBOL_PROFILE_SIZE_BYTES    40
#define RF_API_FAST_BR_SYMBOL_PROFILE_SIZE_BYTES    20

/*** RF API local structures ***/

/*******************************************************************/
typedef enum {
    // Driver errors.
    RF_API_ERROR_NULL_PARAMETER = (RF_API_SUCCESS + 1),
    RF_API_ERROR_BUFFER_SIZE,
    RF_API_ERROR_STATE,
    RF_API_ERROR_MODULATION,
    RF_API_ERROR_MODE,
    RF_API_ERROR_LATENCY_TYPE,
    // Low level drivers errors.
    RF_API_ERROR_DRIVER_TIMER_MODULATION,
    RF_API_ERROR_DRIVER_MCU_API,
    RF_API_ERROR_DRIVER_SX1232,
    RF_API_ERROR_DRIVER_RFE
} RF_API_custom_status_t;

/*******************************************************************/
typedef enum {
    RF_API_STATE_READY = 0,
    RF_API_STATE_TX_START,
    RF_API_STATE_TX_RAMP_UP,
    RF_API_STATE_TX_BITSTREAM,
    RF_API_STATE_TX_RAMP_DOWN,
    RF_API_STATE_TX_END,
#ifdef SIGFOX_EP_BIDIRECTIONAL
    RF_API_STATE_RX_START,
    RF_API_STATE_RX,
#endif
    RF_API_STATE_LAST
} RF_API_state_t;

/*******************************************************************/
typedef union {
    sfx_u8 all;
    struct {
#ifdef SIGFOX_EP_BIDIRECTIONAL
        unsigned gpio_irq_flag :1;
        unsigned gpio_irq_enable :1;
#endif
        unsigned timer_irq_flag :1;
        unsigned timer_irq_enable :1;
    } __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
} RF_API_flags_t;

/*******************************************************************/
typedef struct {
    // Common.
    RF_API_state_t state;
    volatile RF_API_flags_t flags;
    // TX.
    sfx_u8 tx_bitstream[SIGFOX_UL_BITSTREAM_SIZE_BYTES];
    sfx_u8 tx_bitstream_size_bytes;
    sfx_u8* tx_ramp_amplitude_profile;
    sfx_u8* tx_bit0_amplitude_profile;
    sfx_u8 tx_symbol_profile_size_bytes;
    sfx_u8 tx_byte_idx;
    sfx_u8 tx_bit_idx;
    sfx_u8 tx_symbol_profile_idx;
    sfx_u32 tx_modulation_timer_period_ns;
#ifdef SIGFOX_EP_BIDIRECTIONAL
    // RX.
    sfx_u8 dl_phy_content[SIGFOX_DL_PHY_CONTENT_SIZE_BYTES];
    sfx_u8 dl_rssi_read_flag;
    sfx_s16 dl_rssi_dbm;
#endif
} RF_API_context_t;

/*** RF API local global variables ***/

static const sfx_u8 RF_API_SLOW_BR_RAMP_AMPLITUDE_PROFILE[RF_API_SLOW_BR_SYMBOL_PROFILE_SIZE_BYTES] = {
    0, 9, 16, 23, 29, 36, 43, 49, 55, 60, 65, 70, 74, 78, 81, 84, 86, 88, 89, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90
};
static const sfx_u8 RF_API_SLOW_BR_BIT0_AMPLITUDE_PROFILE[RF_API_SLOW_BR_SYMBOL_PROFILE_SIZE_BYTES] = {
    90, 89, 88, 86, 84, 81, 78, 74, 70, 65, 60, 55, 49, 43, 36, 29, 23, 16, 9, 0, 0, 9, 16, 23, 29, 36, 43, 49, 55, 60, 65, 70, 74, 78, 81, 84, 86, 88, 89, 90
};
static const sfx_u8 RF_API_FAST_BR_RAMP_AMPLITUDE_PROFILE[RF_API_FAST_BR_SYMBOL_PROFILE_SIZE_BYTES] = {
    0, 9, 16, 23, 29, 36, 43, 49, 55, 60, 65, 70, 74, 78, 81, 84, 86, 88, 89, 90
};
static const sfx_u8 RF_API_FAST_BR_BIT0_AMPLITUDE_PROFILE[RF_API_FAST_BR_SYMBOL_PROFILE_SIZE_BYTES] = {
    90, 86, 82, 73, 66, 55, 39, 23, 9, 0, 0, 9, 23, 39, 55, 66, 73, 82, 86, 90
};
#ifdef SIGFOX_EP_BIDIRECTIONAL
static const sfx_u8 RF_API_DL_FT[SIGFOX_DL_FT_SIZE_BYTES] = SIGFOX_DL_FT;
#endif
#if (defined SIGFOX_EP_TIMER_REQUIRED) && (defined SIGFOX_EP_LATENCY_COMPENSATION)
static sfx_u32 RF_API_LATENCY_MS[RF_API_LATENCY_LAST] = {
    POWER_ON_DELAY_MS_RADIO_TCXO, // Wake-up.
    (POWER_ON_DELAY_MS_RADIO + SX1232_OSCILLATOR_DELAY_MS + 1), // TX init (power on delay + 750us).
    0, // Send start (depends on bit rate and will be computed during init function).
    0, // Send stop (depends on bit rate and will be computed during init function).
    0, // TX de-init (215us).
    0, // Sleep.
#ifdef SIGFOX_EP_BIDIRECTIONAL
    (POWER_ON_DELAY_MS_RADIO + SX1232_OSCILLATOR_DELAY_MS + SX1232_IMAGE_CALIBRATION_DELAY_MS + 2), // RX init (power on delay + calibration + 1.06ms).
    SX1232_START_RX_DELAY_MS, // Receive start.
    10, // Receive stop (10ms).
    0, // RX de-init (215us).
#endif
};
#endif
static RF_API_context_t rf_api_ctx;

/*** RF API local functions ***/

/*******************************************************************/
static void _RF_API_modulation_timer_irq_callback(void) {
    // Set flag.
    rf_api_ctx.flags.timer_irq_flag = rf_api_ctx.flags.timer_irq_enable;
}

#ifdef SIGFOX_EP_BIDIRECTIONAL
/*******************************************************************/
static void _RF_API_sx1232_gpio_irq_callback(void) {
    // Set flag.
    rf_api_ctx.flags.gpio_irq_flag = rf_api_ctx.flags.gpio_irq_enable;
}
#endif

/*******************************************************************/
static RF_API_status_t _RF_API_internal_process(void) {
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
    SX1232_status_t sx1232_status = SX1232_SUCCESS;
    sfx_u8 symbol_profile_idx = 0;
#ifdef SIGFOX_EP_BIDIRECTIONAL
    RFE_status_t rfe_status = RFE_SUCCESS;
    sfx_u8 sx1232_payload_ready_irq = 0;
#endif
    // Perform state machine.
    switch (rf_api_ctx.state) {
    case RF_API_STATE_READY:
        // Nothing to do.
        break;
    case RF_API_STATE_TX_START:
        // Start radio.
        SX1232_set_pa_power_value(0x00);
        sx1232_status = SX1232_start_tx();
        SX1232_stack_exit_error(ERROR_BASE_SX1232, (RF_API_status_t) RF_API_ERROR_DRIVER_SX1232);
        // Enable interrupt.
        rf_api_ctx.flags.timer_irq_enable = 1;
        // Update state.
        rf_api_ctx.state = RF_API_STATE_TX_RAMP_UP;
        break;
    case RF_API_STATE_TX_RAMP_UP:
        // Load ramp profile.
        SX1232_set_pa_power_value(rf_api_ctx.tx_ramp_amplitude_profile[rf_api_ctx.tx_symbol_profile_idx]);
        // Increment symbol profile index.
        rf_api_ctx.tx_symbol_profile_idx++;
        if (rf_api_ctx.tx_symbol_profile_idx >= rf_api_ctx.tx_symbol_profile_size_bytes) {
            rf_api_ctx.tx_symbol_profile_idx = 0;
            // Update state.
            rf_api_ctx.state = RF_API_STATE_TX_BITSTREAM;
        }
        break;
    case RF_API_STATE_TX_BITSTREAM:
        // Check bit.
        if ((rf_api_ctx.tx_bitstream[rf_api_ctx.tx_byte_idx] & (1 << (7 - rf_api_ctx.tx_bit_idx))) == 0) {
            // Invert phase at the middle of the symbol profile.
            if (rf_api_ctx.tx_symbol_profile_idx == (rf_api_ctx.tx_symbol_profile_size_bytes >> 1)) {
                GPIO_SX1232_DIO2_HIGH();
            }
            else {
                GPIO_SX1232_DIO2_LOW();
            }
            // Amplitude shaping.
            symbol_profile_idx = rf_api_ctx.tx_symbol_profile_idx;
        }
        else {
            // Constant CW.
            symbol_profile_idx = 0;
        }
        SX1232_set_pa_power_value(rf_api_ctx.tx_bit0_amplitude_profile[symbol_profile_idx]);
        // Increment symbol profile index.
        rf_api_ctx.tx_symbol_profile_idx++;
        if (rf_api_ctx.tx_symbol_profile_idx >= rf_api_ctx.tx_symbol_profile_size_bytes) {
            rf_api_ctx.tx_symbol_profile_idx = 0;
            // Increment bit index.
            rf_api_ctx.tx_bit_idx++;
            if (rf_api_ctx.tx_bit_idx >= 8) {
                rf_api_ctx.tx_bit_idx = 0;
                // Increment byte index.
                rf_api_ctx.tx_byte_idx++;
                // Check end of bitstream.
                if (rf_api_ctx.tx_byte_idx >= (rf_api_ctx.tx_bitstream_size_bytes)) {
                    rf_api_ctx.tx_byte_idx = 0;
                    // Update state.
                    rf_api_ctx.state = RF_API_STATE_TX_RAMP_DOWN;
                }
            }
        }
        break;
    case RF_API_STATE_TX_RAMP_DOWN:
        // Load ramp profile.
        SX1232_set_pa_power_value(rf_api_ctx.tx_ramp_amplitude_profile[rf_api_ctx.tx_symbol_profile_size_bytes - rf_api_ctx.tx_symbol_profile_idx - 1]);
        // Increment symbol profile index.
        rf_api_ctx.tx_symbol_profile_idx++;
        if (rf_api_ctx.tx_symbol_profile_idx >= rf_api_ctx.tx_symbol_profile_size_bytes) {
            rf_api_ctx.tx_symbol_profile_idx = 0;
            // Update state.
            rf_api_ctx.state = RF_API_STATE_TX_END;
        }
        break;
    case RF_API_STATE_TX_END:
        // Disable interrupt.
        rf_api_ctx.flags.timer_irq_enable = 0;
        // Stop radio.
        SX1232_set_pa_power_value(0x00);
        SX1232_set_mode(SX1232_MODE_STANDBY);
        // Update state.
        rf_api_ctx.state = RF_API_STATE_READY;
        break;
#ifdef SIGFOX_EP_BIDIRECTIONAL
    case RF_API_STATE_RX_START:
        // Start radio.
        sx1232_status = SX1232_start_rx();
        SX1232_stack_exit_error(ERROR_BASE_SX1232, (RF_API_status_t) RF_API_ERROR_DRIVER_SX1232);
        // Enable interrupt.
        rf_api_ctx.flags.gpio_irq_enable = 1;
        // Update state.
        rf_api_ctx.state = RF_API_STATE_RX;
        break;
    case RF_API_STATE_RX:
        // Read payload ready flag.
        sx1232_status = SX1232_get_irq_flag(SX1232_IRQ_INDEX_PAYLOAD_READY, &sx1232_payload_ready_irq);
        SX1232_stack_exit_error(ERROR_BASE_SX1232, (RF_API_status_t) RF_API_ERROR_DRIVER_SX1232);
        // Check flag.
        if (sx1232_payload_ready_irq != 0) {
            // Read RSSI.
            rfe_status = RFE_get_rssi(&rf_api_ctx.dl_rssi_dbm);
            RFE_stack_exit_error(ERROR_BASE_RFE, (RF_API_status_t) RF_API_ERROR_DRIVER_RFE);
            // Stop radio.
            sx1232_status = SX1232_set_mode(SX1232_MODE_STANDBY);
            SX1232_stack_exit_error(ERROR_BASE_SX1232, (RF_API_status_t) RF_API_ERROR_DRIVER_SX1232);
            // Read FIFO
            sx1232_status = SX1232_read_fifo((sfx_u8*) rf_api_ctx.dl_phy_content, SIGFOX_DL_PHY_CONTENT_SIZE_BYTES);
            SX1232_stack_exit_error(ERROR_BASE_SX1232, (RF_API_status_t) RF_API_ERROR_DRIVER_SX1232);
            // Disable interrupt.
            rf_api_ctx.flags.gpio_irq_enable = 0;
            // Update state.
            rf_api_ctx.state = RF_API_STATE_READY;
        }
        break;
#endif
    default:
        SIGFOX_EXIT_ERROR((RF_API_status_t) RF_API_ERROR_STATE);
        break;
    }
errors:
    SIGFOX_RETURN();
}

/*** RF API functions ***/

#if (defined SIGFOX_EP_ASYNCHRONOUS) || (defined SIGFOX_EP_LOW_LEVEL_OPEN_CLOSE)
/*******************************************************************/
RF_API_status_t RF_API_open(RF_API_config_t* rf_api_config) {
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
    SIGFOX_UNUSED(rf_api_config);
    SIGFOX_RETURN();
}
#endif

#ifdef SIGFOX_EP_LOW_LEVEL_OPEN_CLOSE
/*******************************************************************/
RF_API_status_t RF_API_close(void) {
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
    SIGFOX_RETURN();
}
#endif

#ifdef SIGFOX_EP_ASYNCHRONOUS
/*******************************************************************/
RF_API_status_t RF_API_process(void) {
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
    SIGFOX_RETURN();
}
#endif

/*******************************************************************/
RF_API_status_t RF_API_wake_up(void) {
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
    // Turn radio TCXO on.
    POWER_enable(POWER_REQUESTER_ID_RF_API, POWER_DOMAIN_RADIO_TCXO, LPTIM_DELAY_MODE_SLEEP);
    SIGFOX_RETURN();
}

/*******************************************************************/
RF_API_status_t RF_API_sleep(void) {
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
    // Turn radio TCXO off.
    POWER_disable(POWER_REQUESTER_ID_RF_API, POWER_DOMAIN_RADIO_TCXO);
    SIGFOX_RETURN();
}

/*******************************************************************/
RF_API_status_t RF_API_init(RF_API_radio_parameters_t* radio_parameters) {
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
    TIM_status_t tim_status = TIM_SUCCESS;
    SX1232_status_t sx1232_status = SX1232_SUCCESS;
    RFE_status_t rfe_status = RFE_SUCCESS;
    SX1232_modulation_t modulation = SX1232_MODULATION_LAST;
    SX1232_modulation_shaping_t modulation_shaping = SX1232_MODULATION_SHAPING_LAST;
    SX1232_data_mode_t data_mode = SX1232_DATA_MODE_LAST;
    sfx_u32 bitrate_bps = 0;
    sfx_u32 deviation_hz = 0;
    sfx_u32 frequency_hz = 0;
#ifdef SIGFOX_EP_PARAMETERS_CHECK
    // Check parameter.
    if (radio_parameters == SIGFOX_NULL) {
        SIGFOX_EXIT_ERROR((RF_API_status_t) RF_API_ERROR_NULL_PARAMETER);
    }
#endif
    // Turn radio on.
    POWER_enable(POWER_REQUESTER_ID_RF_API, POWER_DOMAIN_RADIO, LPTIM_DELAY_MODE_SLEEP);
    // Init transceiver.
    sx1232_status = SX1232_set_mode(SX1232_MODE_SLEEP);
    SX1232_stack_exit_error(ERROR_BASE_SX1232, (RF_API_status_t) RF_API_ERROR_DRIVER_SX1232);
    // Oscillator.
    sx1232_status = SX1232_set_oscillator(SX1232_OSCILLATOR_TCXO);
    SX1232_stack_exit_error(ERROR_BASE_SX1232, (RF_API_status_t) RF_API_ERROR_DRIVER_SX1232);
    // Modulation parameters.
    switch (radio_parameters->modulation) {
    case RF_API_MODULATION_NONE:
        modulation = SX1232_MODULATION_OOK;
        modulation_shaping = SX1232_MODULATION_SHAPING_NONE;
        bitrate_bps = (radio_parameters->bit_rate_bps);
#ifdef SIGFOX_EP_BIDIRECTIONAL
        deviation_hz = (radio_parameters->deviation_hz);
#endif
        frequency_hz = (radio_parameters->frequency_hz);
        data_mode = SX1232_DATA_MODE_CONTINUOUS;
        // Set CW output power.
        sx1232_status = SX1232_set_rf_output_power(radio_parameters->tx_power_dbm_eirp);
        SX1232_stack_exit_error(ERROR_BASE_SX1232, (RF_API_status_t) RF_API_ERROR_DRIVER_SX1232);
        break;
    case RF_API_MODULATION_DBPSK:
        // Update tables according to bit rate.
        if ((radio_parameters->bit_rate_bps) > SIGFOX_UL_BIT_RATE_BPS_LIST[SIGFOX_UL_BIT_RATE_100BPS]) {
            // Use fast bit rate tables.
            rf_api_ctx.tx_ramp_amplitude_profile = (sfx_u8*) RF_API_FAST_BR_RAMP_AMPLITUDE_PROFILE;
            rf_api_ctx.tx_bit0_amplitude_profile = (sfx_u8*) RF_API_FAST_BR_BIT0_AMPLITUDE_PROFILE;
            rf_api_ctx.tx_symbol_profile_size_bytes = RF_API_FAST_BR_SYMBOL_PROFILE_SIZE_BYTES;
        }
        else {
            // Use slow bit rate tables.
            rf_api_ctx.tx_ramp_amplitude_profile = (sfx_u8*) RF_API_SLOW_BR_RAMP_AMPLITUDE_PROFILE;
            rf_api_ctx.tx_bit0_amplitude_profile = (sfx_u8*) RF_API_SLOW_BR_BIT0_AMPLITUDE_PROFILE;
            rf_api_ctx.tx_symbol_profile_size_bytes = RF_API_SLOW_BR_SYMBOL_PROFILE_SIZE_BYTES;
        }
        modulation = SX1232_MODULATION_FSK;
        modulation_shaping = SX1232_MODULATION_SHAPING_NONE;
        bitrate_bps = 0; // Unused directly.
        deviation_hz = (((radio_parameters->bit_rate_bps) * rf_api_ctx.tx_symbol_profile_size_bytes) >> 2);
        frequency_hz = (radio_parameters->frequency_hz) + deviation_hz;
        data_mode = SX1232_DATA_MODE_CONTINUOUS;
        // Use manual control of the PA.
        sx1232_status = SX1232_enable_manual_pa_control();
        SX1232_stack_exit_error(ERROR_BASE_SX1232, (RF_API_status_t) RF_API_ERROR_DRIVER_SX1232);
        // Init symbol profile timer.
        rf_api_ctx.tx_modulation_timer_period_ns = (MATH_POWER_10[9]) / ((radio_parameters->bit_rate_bps) * rf_api_ctx.tx_symbol_profile_size_bytes);
        tim_status = TIM_STD_init(TIM_INSTANCE_RF_API, NVIC_PRIORITY_SIGFOX_MODULATION_TIMER);
        TIM_stack_exit_error(ERROR_BASE_TIM_RF_API, (RF_API_status_t) RF_API_ERROR_DRIVER_TIMER_MODULATION);
        break;
    case RF_API_MODULATION_GFSK:
        modulation = SX1232_MODULATION_FSK;
        modulation_shaping = SX1232_MODULATION_SHAPING_FSK_BT_1;
        bitrate_bps = (radio_parameters->bit_rate_bps);
#ifdef SIGFOX_EP_BIDIRECTIONAL
        deviation_hz = (radio_parameters->deviation_hz);
#endif
        frequency_hz = (radio_parameters->frequency_hz);
        data_mode = SX1232_DATA_MODE_PACKET;
        break;
    default:
        SIGFOX_EXIT_ERROR((RF_API_status_t) RF_API_ERROR_MODULATION);
        break;
    }
    // Frequency.
    sx1232_status = SX1232_set_rf_frequency(frequency_hz);
    SX1232_stack_exit_error(ERROR_BASE_SX1232, (RF_API_status_t) RF_API_ERROR_DRIVER_SX1232);
    // Modulation.
    sx1232_status = SX1232_set_modulation(modulation, modulation_shaping);
    SX1232_stack_exit_error(ERROR_BASE_SX1232, (RF_API_status_t) RF_API_ERROR_DRIVER_SX1232);
    // Bit rate.
    if (bitrate_bps != 0) {
        sx1232_status = SX1232_set_bitrate(bitrate_bps);
        SX1232_stack_exit_error(ERROR_BASE_SX1232, (RF_API_status_t) RF_API_ERROR_DRIVER_SX1232);
    }
    // Deviation.
    if (deviation_hz != 0) {
        sx1232_status = SX1232_set_fsk_deviation(deviation_hz);
        SX1232_stack_exit_error(ERROR_BASE_SX1232, (RF_API_status_t) RF_API_ERROR_DRIVER_SX1232);
    }
    // Data mode.
    sx1232_status = SX1232_set_data_mode(data_mode);
    SX1232_stack_exit_error(ERROR_BASE_SX1232, (RF_API_status_t) RF_API_ERROR_DRIVER_SX1232);
    // Configure specific registers.
    switch (radio_parameters->rf_mode) {
    case RF_API_MODE_TX:
        // Specific uplink settings.
        sx1232_status = SX1232_set_pll_mode(SX1232_PLL_MODE_LOW_PHASE_NOISE);
        SX1232_stack_exit_error(ERROR_BASE_SX1232, (RF_API_status_t) RF_API_ERROR_DRIVER_SX1232);
#if (defined SIGFOX_EP_TIMER_REQUIRED) && (defined SIGFOX_EP_LATENCY_COMPENSATION)
        // Start latency = ramp-up.
        RF_API_LATENCY_MS[RF_API_LATENCY_SEND_START] = SX1232_START_TX_DELAY_MS + ((1000) / ((sfx_u32) (radio_parameters->bit_rate_bps)));
        // Stop latency = ramp-down.
        RF_API_LATENCY_MS[RF_API_LATENCY_SEND_STOP] = ((1000) / ((sfx_u32) (radio_parameters->bit_rate_bps)));
#endif
        // Init DIO2 for bitstream transmission.
        GPIO_configure(&GPIO_SX1232_DIO2, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
        // Switch to TX.
        sx1232_status = SX1232_set_rf_output_pin(SX1232_RF_OUTPUT_PIN_PABOOST);
        SX1232_stack_exit_error(ERROR_BASE_SX1232, (RF_API_status_t) RF_API_ERROR_DRIVER_SX1232);
        rfe_status = RFE_set_path(RFE_PATH_TX_BYPASS);
        RFE_stack_exit_error(ERROR_BASE_RFE, (RF_API_status_t) RF_API_ERROR_DRIVER_RFE);
        break;
#ifdef SIGFOX_EP_BIDIRECTIONAL
    case RF_API_MODE_RX:
        // Specific downlink settings.
        sx1232_status = SX1232_set_rx_bandwidth(SX1232_RXBW_MANTISSA_24, SX1232_RXBW_EXPONENT_7);
        SX1232_stack_exit_error(ERROR_BASE_SX1232, (RF_API_status_t) RF_API_ERROR_DRIVER_SX1232);
        sx1232_status = SX1232_set_lna_configuration(SX1232_LNA_MODE_BOOST, SX1232_LNA_GAIN_ATTENUATION_0DB, 1);
        SX1232_stack_exit_error(ERROR_BASE_SX1232, (RF_API_status_t) RF_API_ERROR_DRIVER_SX1232);
        sx1232_status = SX1232_set_preamble_detector(1, 0);
        SX1232_stack_exit_error(ERROR_BASE_SX1232, (RF_API_status_t) RF_API_ERROR_DRIVER_SX1232);
        sx1232_status = SX1232_set_sync_word((sfx_u8*) RF_API_DL_FT, SIGFOX_DL_FT_SIZE_BYTES);
        SX1232_stack_exit_error(ERROR_BASE_SX1232, (RF_API_status_t) RF_API_ERROR_DRIVER_SX1232);
        sx1232_status = SX1232_set_data_size(SIGFOX_DL_PHY_CONTENT_SIZE_BYTES);
        SX1232_stack_exit_error(ERROR_BASE_SX1232, (RF_API_status_t) RF_API_ERROR_DRIVER_SX1232);
        // Image calibration.
        sx1232_status = SX1232_calibrate_image();
        SX1232_stack_exit_error(ERROR_BASE_SX1232, (RF_API_status_t) RF_API_ERROR_DRIVER_SX1232);
        // Init DIO0 to detect payload ready interrupt.
        sx1232_status = SX1232_set_dio_mapping(SX1232_DIO0, SX1232_DIO_MAPPING0);
        SX1232_stack_exit_error(ERROR_BASE_SX1232, (RF_API_status_t) RF_API_ERROR_DRIVER_SX1232);
        EXTI_configure_gpio(&GPIO_SX1232_DIO0, GPIO_PULL_NONE, EXTI_TRIGGER_RISING_EDGE, &_RF_API_sx1232_gpio_irq_callback, NVIC_PRIORITY_SIGFOX_DOWNLINK_GPIO);
        // Switch to RX.
        rfe_status = RFE_set_path(RFE_PATH_RX_LNA);
        RFE_stack_exit_error(ERROR_BASE_RFE, (RF_API_status_t) RF_API_ERROR_DRIVER_RFE);
        break;
#endif
    default:
        SIGFOX_EXIT_ERROR((RF_API_status_t) RF_API_ERROR_MODE);
        break;
    }
errors:
    SIGFOX_RETURN();
}

/*******************************************************************/
RF_API_status_t RF_API_de_init(void) {
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
    TIM_status_t tim_status = TIM_SUCCESS;
    RFE_status_t rfe_status = RFE_SUCCESS;
    // Release DIO2.
    GPIO_write(&GPIO_SX1232_DIO2, 0);
#ifdef SIGFOX_EP_BIDIRECTIONAL
    // Release DIO0.
    EXTI_release_gpio(&GPIO_SX1232_DIO0, GPIO_MODE_OUTPUT);
#endif
    // Release symbol profile timer.
    tim_status = TIM_STD_de_init(TIM_INSTANCE_RF_API);
    // Check status.
    if (tim_status != TIM_SUCCESS) {
        TIM_stack_error(ERROR_BASE_TIM_RF_API);
        status = (RF_API_status_t) RF_API_ERROR_DRIVER_TIMER_MODULATION;
    }
    // Disable front-end.
    rfe_status = RFE_set_path(RFE_PATH_NONE);
    // Check status.
    if (rfe_status != RFE_SUCCESS) {
        RFE_stack_error(ERROR_BASE_RFE);
        status = (RF_API_status_t) RF_API_ERROR_DRIVER_RFE;
    }
    POWER_disable(POWER_REQUESTER_ID_RF_API, POWER_DOMAIN_RADIO);
    SIGFOX_RETURN();
}

/*******************************************************************/
RF_API_status_t RF_API_send(RF_API_tx_data_t* tx_data) {
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
    TIM_status_t tim_status = TIM_SUCCESS;
    sfx_u8 idx = 0;
#ifdef SIGFOX_EP_PARAMETERS_CHECK
    // Check parameter.
    if (tx_data == SIGFOX_NULL) {
        SIGFOX_EXIT_ERROR((RF_API_status_t) RF_API_ERROR_NULL_PARAMETER);
    }
#endif
    // Store TX data.
    rf_api_ctx.tx_bitstream_size_bytes = (tx_data->bitstream_size_bytes);
    for (idx = 0; idx < (rf_api_ctx.tx_bitstream_size_bytes); idx++) {
        rf_api_ctx.tx_bitstream[idx] = (tx_data->bitstream)[idx];
    }
    // Init state.
    rf_api_ctx.tx_bit_idx = 0;
    rf_api_ctx.tx_byte_idx = 0;
    rf_api_ctx.tx_symbol_profile_idx = 0;
    rf_api_ctx.state = RF_API_STATE_TX_START;
    rf_api_ctx.flags.all = 0;
    // Trigger TX.
    status = _RF_API_internal_process();
    SIGFOX_CHECK_STATUS(RF_API_SUCCESS);
    // Start timer.
    tim_status = TIM_STD_start(TIM_INSTANCE_RF_API, rf_api_ctx.tx_modulation_timer_period_ns, TIM_UNIT_NS, &_RF_API_modulation_timer_irq_callback);
    TIM_stack_exit_error(ERROR_BASE_TIM_RF_API, (RF_API_status_t) RF_API_ERROR_DRIVER_TIMER_MODULATION);
    // Wait for transmission to complete.
    while (rf_api_ctx.state != RF_API_STATE_READY) {
        // Wait for GPIO interrupt.
        while (rf_api_ctx.flags.timer_irq_flag == 0);
        // Clear flag.
        rf_api_ctx.flags.timer_irq_flag = 0;
        // Call process function.
        _RF_API_internal_process();
    }
    tim_status = TIM_STD_stop(TIM_INSTANCE_RF_API);
    TIM_stack_exit_error(ERROR_BASE_TIM_RF_API, (RF_API_status_t) RF_API_ERROR_DRIVER_TIMER_MODULATION);
errors:
    SIGFOX_RETURN();
}

#ifdef SIGFOX_EP_BIDIRECTIONAL
/*******************************************************************/
RF_API_status_t RF_API_receive(RF_API_rx_data_t* rx_data) {
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
    MCU_API_status_t mcu_api_status = MCU_API_SUCCESS;
    SX1232_status_t sx1232_status = SX1232_SUCCESS;
    sfx_bool dl_timeout = SIGFOX_FALSE;
#ifdef SIGFOX_EP_PARAMETERS_CHECK
    // Check parameter.
    if (rx_data == SIGFOX_NULL) {
        SIGFOX_EXIT_ERROR((RF_API_status_t) RF_API_ERROR_NULL_PARAMETER);
    }
#endif
    // Enable GPIO interrupt.
    EXTI_clear_gpio_flag(&GPIO_SX1232_DIO0);
    EXTI_enable_gpio_interrupt(&GPIO_SX1232_DIO0);
    // Reset flag.
    (rx_data->data_received) = SIGFOX_FALSE;
    // Init state.
    rf_api_ctx.state = RF_API_STATE_RX_START;
    rf_api_ctx.flags.all = 0;
    // Trigger RX.
    status = _RF_API_internal_process();
    SIGFOX_CHECK_STATUS(RF_API_SUCCESS);
    // Wait for reception to complete.
    while (rf_api_ctx.state != RF_API_STATE_READY) {
        // Wait for GPIO interrupt.
        while (rf_api_ctx.flags.gpio_irq_flag == 0) {
            // Enter sleep mode.
            PWR_enter_sleep_mode(PWR_SLEEP_MODE_NORMAL);
            IWDG_reload();
            // Check timeout.
            mcu_api_status = MCU_API_timer_status(MCU_API_TIMER_INSTANCE_T_RX, &dl_timeout);
            MCU_API_check_status((RF_API_status_t) RF_API_ERROR_DRIVER_MCU_API);
            // Exit if timeout.
            if (dl_timeout == SIGFOX_TRUE) {
                // Stop radio.
                sx1232_status = SX1232_set_mode(SX1232_MODE_STANDBY);
                SX1232_stack_exit_error(ERROR_BASE_SX1232, (RF_API_status_t) RF_API_ERROR_DRIVER_SX1232);
                // Exit loop.
                goto errors;
            }
        }
        // Clear flag.
        rf_api_ctx.flags.gpio_irq_flag = 0;
        // Call process function.
        status = _RF_API_internal_process();
        SIGFOX_CHECK_STATUS(RF_API_SUCCESS);
    }
    // Update status flag.
    (rx_data->data_received) = SIGFOX_TRUE;
errors:
    // Disable GPIO interrupt.
    EXTI_disable_gpio_interrupt(&GPIO_SX1232_DIO0);
    SIGFOX_RETURN();
}
#endif

#ifdef SIGFOX_EP_BIDIRECTIONAL
/*******************************************************************/
RF_API_status_t RF_API_get_dl_phy_content_and_rssi(sfx_u8* dl_phy_content, sfx_u8 dl_phy_content_size, sfx_s16* dl_rssi_dbm) {
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
    sfx_u8 idx = 0;
#ifdef SIGFOX_EP_PARAMETERS_CHECK
    // Check parameters.
    if ((dl_phy_content == SIGFOX_NULL) || (dl_rssi_dbm == SIGFOX_NULL)) {
        SIGFOX_EXIT_ERROR((RF_API_status_t) RF_API_ERROR_NULL_PARAMETER);
    }
    if (dl_phy_content_size > SIGFOX_DL_PHY_CONTENT_SIZE_BYTES) {
        SIGFOX_EXIT_ERROR((RF_API_status_t) RF_API_ERROR_BUFFER_SIZE);
    }
#endif
    // Fill data.
    for (idx = 0; idx < dl_phy_content_size; idx++) {
        dl_phy_content[idx] = rf_api_ctx.dl_phy_content[idx];
    }
    (*dl_rssi_dbm) = (sfx_s16) rf_api_ctx.dl_rssi_dbm;
#ifdef SIGFOX_EP_PARAMETERS_CHECK
errors:
#endif
    SIGFOX_RETURN();
}
#endif

#if (defined SIGFOX_EP_REGULATORY) && (defined SIGFOX_EP_SPECTRUM_ACCESS_LBT)
/*******************************************************************/
RF_API_status_t RF_API_carrier_sense(RF_API_carrier_sense_parameters_t *carrier_sense_params) {
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
    SIGFOX_UNUSED(carrier_sense_params);
    SIGFOX_RETURN();
}
#endif

#if (defined SIGFOX_EP_TIMER_REQUIRED) && (defined SIGFOX_EP_LATENCY_COMPENSATION)
/*******************************************************************/
RF_API_status_t RF_API_get_latency(RF_API_latency_t latency_type, sfx_u32* latency_ms) {
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
#ifdef SIGFOX_EP_PARAMETERS_CHECK
    // Check parameter.
    if (latency_ms == SIGFOX_NULL) {
        SIGFOX_EXIT_ERROR((RF_API_status_t) RF_API_ERROR_NULL_PARAMETER);
    }
    if (latency_type >= RF_API_LATENCY_LAST) {
        SIGFOX_EXIT_ERROR((RF_API_status_t) RF_API_ERROR_LATENCY_TYPE);
    }
#endif
    // Set latency.
    (*latency_ms) = RF_API_LATENCY_MS[latency_type];
#ifdef SIGFOX_EP_PARAMETERS_CHECK
errors:
#endif
    SIGFOX_RETURN();
}
#endif

#ifdef SIGFOX_EP_CERTIFICATION
/*******************************************************************/
RF_API_status_t RF_API_start_continuous_wave(void) {
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
    SX1232_status_t sx1232_status = SX1232_SUCCESS;
    // Start radio.
    sx1232_status = SX1232_start_tx();
    SX1232_stack_exit_error(ERROR_BASE_SX1232, (RF_API_status_t) RF_API_ERROR_DRIVER_SX1232);
    // Start OOK data signal.
    GPIO_write(&GPIO_SX1232_DIO2, 1);
errors:
    SIGFOX_RETURN();
}
#endif

#ifdef SIGFOX_EP_VERBOSE
/*******************************************************************/
RF_API_status_t RF_API_get_version(sfx_u8 **version, sfx_u8 *version_size_char) {
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
    SIGFOX_UNUSED(version);
    SIGFOX_UNUSED(version_size_char);
    SIGFOX_RETURN();
}
#endif

#ifdef SIGFOX_EP_ERROR_CODES
/*******************************************************************/
void RF_API_error(void) {
    // Force all front-end off.
    RFE_set_path(RFE_PATH_NONE);
    POWER_disable(POWER_REQUESTER_ID_RF_API, POWER_DOMAIN_RADIO_TCXO);
    POWER_disable(POWER_REQUESTER_ID_RF_API, POWER_DOMAIN_RADIO);
}
#endif
