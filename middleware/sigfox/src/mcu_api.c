/*!*****************************************************************
 * \file    mcu_api.c
 * \brief   MCU drivers.
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

#include "manuf/mcu_api.h"

#ifndef SIGFOX_EP_DISABLE_FLAGS_FILE
#include "sigfox_ep_flags.h"
#endif
#include "sigfox_types.h"
#include "sigfox_error.h"

#include "aes.h"
#include "analog.h"
#include "error.h"
#include "error_base.h"
#include "mcu_mapping.h"
#include "nvic_priority.h"
#include "nvm.h"
#include "nvm_address.h"
#include "power.h"
#include "tim.h"
#include "types.h"

/*** MCU API local structures ***/

typedef enum {
    // Driver errors.
    MCU_API_ERROR_NULL_PARAMETER = (MCU_API_SUCCESS + 1),
    MCU_API_ERROR_EP_KEY,
    MCU_API_ERROR_LATENCY_TYPE,
    // Low level drivers errors.
    MCU_API_ERROR_DRIVER_ANALOG,
    MCU_API_ERROR_DRIVER_AES,
    MCU_API_ERROR_DRIVER_NVM,
    MCU_API_ERROR_DRIVER_TIM
} MCU_API_custom_status_t;

/*** MCU API local global variables ***/

#if (defined SIGFOX_EP_TIMER_REQUIRED) && (defined SIGFOX_EP_LATENCY_COMPENSATION) && (defined SIGFOX_EP_BIDIRECTIONAL)
static sfx_u32 MCU_API_LATENCY_MS[MCU_API_LATENCY_LAST] = {
    (POWER_ON_DELAY_MS_ANALOG + ADC_INIT_DELAY_MS) // Get voltage and temperature function.
};
#endif

/*** MCU API functions ***/

#if (defined SIGFOX_EP_ASYNCHRONOUS) || (defined SIGFOX_EP_LOW_LEVEL_OPEN_CLOSE)
/*******************************************************************/
MCU_API_status_t MCU_API_open(MCU_API_config_t* mcu_api_config) {
    // Local variables.
    MCU_API_status_t status = MCU_API_SUCCESS;
    TIM_status_t tim_status = TIM_SUCCESS;
    // Ignore unused parameters.
    SIGFOX_UNUSED(mcu_api_config);
    // Init timer.
    tim_status = TIM_MCH_init(TIM_INSTANCE_MCU_API, NVIC_PRIORITY_SIGFOX_TIMER);
    TIM_stack_exit_error(ERROR_BASE_TIM_MCU_API, (MCU_API_status_t) MCU_API_ERROR_DRIVER_TIM);
errors:
    SIGFOX_RETURN();
}
#endif

#ifdef SIGFOX_EP_LOW_LEVEL_OPEN_CLOSE
/*******************************************************************/
MCU_API_status_t MCU_API_close(void) {
    // Local variables.
    MCU_API_status_t status = MCU_API_SUCCESS;
    TIM_status_t tim_status = TIM_SUCCESS;
    // Release timer.
    tim_status = TIM_MCH_de_init(TIM_INSTANCE_MCU_API);
    // Check status.
    if (tim_status != TIM_SUCCESS) {
        TIM_stack_error(ERROR_BASE_TIM_MCU_API);
        status = (MCU_API_status_t) MCU_API_ERROR_DRIVER_TIM;
    }
    SIGFOX_RETURN();
}
#endif

#ifdef SIGFOX_EP_ASYNCHRONOUS
/*******************************************************************/
MCU_API_status_t MCU_API_process(void) {
    // Local variables.
    MCU_API_status_t status = MCU_API_SUCCESS;
    SIGFOX_RETURN();
}
#endif

#ifdef SIGFOX_EP_TIMER_REQUIRED
/*******************************************************************/
MCU_API_status_t MCU_API_timer_start(MCU_API_timer_t* timer) {
    // Local variables.
    MCU_API_status_t status = MCU_API_SUCCESS;
    TIM_status_t tim_status = TIM_SUCCESS;
    // Note: low power sleep mode is useless since the MCU is clocked by the TCXO.
    TIM_waiting_mode_t tim_waiting_mode = TIM_WAITING_MODE_SLEEP;
    // Check parameter.
    if (timer == SIGFOX_NULL) {
        SIGFOX_EXIT_ERROR((MCU_API_status_t) MCU_API_ERROR_NULL_PARAMETER);
    }
#ifdef SIGFOX_EP_BIDIRECTIONAL
    // Update waiting mode according to timer reason.
    if ((timer->reason) == MCU_API_TIMER_REASON_T_RX) {
        // T_RX completion is directly checked with the raw timer status within the RF_API_receive() function.
        // All other timers completion are checked with the MCU_API_timer_wait_cplt() function, using sleep waiting mode.
        tim_waiting_mode = TIM_WAITING_MODE_ACTIVE;
    }
#endif
    // Start timer.
    tim_status = TIM_MCH_start_channel(TIM_INSTANCE_MCU_API, (TIM_channel_t) (timer->instance), (timer->duration_ms), tim_waiting_mode);
    TIM_stack_exit_error(ERROR_BASE_TIM_MCU_API, (MCU_API_status_t) MCU_API_ERROR_DRIVER_TIM);
errors:
    SIGFOX_RETURN();
}
#endif

#ifdef SIGFOX_EP_TIMER_REQUIRED
/*******************************************************************/
MCU_API_status_t MCU_API_timer_stop(MCU_API_timer_instance_t timer_instance) {
    // Local variables.
    MCU_API_status_t status = MCU_API_SUCCESS;
    TIM_status_t tim_status = TIM_SUCCESS;
    // Stop timer.
    tim_status = TIM_MCH_stop_channel(TIM_INSTANCE_MCU_API, (TIM_channel_t) timer_instance);
    TIM_stack_exit_error(ERROR_BASE_TIM_MCU_API, (MCU_API_status_t) MCU_API_ERROR_DRIVER_TIM);
errors:
    SIGFOX_RETURN();
}
#endif

#if (defined SIGFOX_EP_TIMER_REQUIRED) && !(defined SIGFOX_EP_ASYNCHRONOUS)
/*******************************************************************/
MCU_API_status_t MCU_API_timer_status(MCU_API_timer_instance_t timer_instance, sfx_bool* timer_has_elapsed) {
    // Local variables.
    MCU_API_status_t status = MCU_API_SUCCESS;
    TIM_status_t tim_status = TIM_SUCCESS;
    // Read status.
    tim_status = TIM_MCH_get_channel_status(TIM_INSTANCE_MCU_API, (TIM_channel_t) timer_instance, timer_has_elapsed);
    TIM_stack_exit_error(ERROR_BASE_TIM_MCU_API, (MCU_API_status_t) MCU_API_ERROR_DRIVER_TIM);
errors:
    SIGFOX_RETURN();
}
#endif

#if (defined SIGFOX_EP_TIMER_REQUIRED) && !(defined SIGFOX_EP_ASYNCHRONOUS)
/*******************************************************************/
MCU_API_status_t MCU_API_timer_wait_cplt(MCU_API_timer_instance_t timer_instance) {
    // Local variables.
    MCU_API_status_t status = MCU_API_SUCCESS;
    TIM_status_t tim_status = TIM_SUCCESS;
    // Wait for timer completion.
    tim_status = TIM_MCH_wait_channel_completion(TIM_INSTANCE_MCU_API, (TIM_channel_t) timer_instance);
    TIM_stack_exit_error(ERROR_BASE_TIM_MCU_API, (MCU_API_status_t) MCU_API_ERROR_DRIVER_TIM);
errors:
    SIGFOX_RETURN();
}
#endif

#ifdef SIGFOX_EP_AES_HW
/*******************************************************************/
MCU_API_status_t MCU_API_aes_128_cbc_encrypt(MCU_API_encryption_data_t* aes_data) {
    // Local variables.
    MCU_API_status_t status = MCU_API_SUCCESS;
    NVM_status_t nvm_status = NVM_SUCCESS;
    AES_status_t aes_status = AES_SUCCESS;
    uint8_t idx = 0;
    uint8_t local_key[SIGFOX_EP_KEY_SIZE_BYTES];
    // Get right key.
#ifdef SIGFOX_EP_PUBLIC_KEY_CAPABLE
    switch (aes_data -> key) {
    case SIGFOX_EP_KEY_PRIVATE:
        // Retrieve private key from NVM.
        for (idx = 0; idx < SIGFOX_EP_KEY_SIZE_BYTES; idx++) {
            nvm_status = NVM_read_byte((NVM_ADDRESS_SIGFOX_EP_KEY + idx), &(local_key[idx]));
            NVM_stack_exit_error(ERROR_BASE_NVM, (MCU_API_status_t) MCU_API_ERROR_DRIVER_NVM);
        }
        break;
    case SIGFOX_EP_KEY_PUBLIC:
        // Use public key.
        for (idx = 0; idx < SIGFOX_EP_KEY_SIZE_BYTES; idx++) {
            local_key[idx] = SIGFOX_EP_PUBLIC_KEY[idx];
        }
        break;
    default:
        SIGFOX_EXIT_ERROR((MCU_API_status_t) MCU_API_ERROR_EP_KEY);
        break;
    }
#else
    // Retrieve private key from NVM.
    for (idx = 0; idx < SIGFOX_EP_KEY_SIZE_BYTES; idx++) {
        nvm_status = NVM_read_byte((NVM_ADDRESS_SIGFOX_EP_KEY + idx), &(local_key[idx]));
        NVM_stack_exit_error(ERROR_BASE_NVM, (MCU_API_status_t) MCU_API_ERROR_DRIVER_NVM);
    }
#endif
    // Init peripheral.
    AES_init();
    // Perform AES.
    aes_status = AES_encrypt((aes_data->data), (aes_data->data), local_key);
    AES_stack_exit_error(ERROR_BASE_AES, (MCU_API_status_t) MCU_API_ERROR_DRIVER_AES);
errors:
    // Release peripheral.
    AES_de_init();
    SIGFOX_RETURN();
}
#endif

#ifdef SIGFOX_EP_CRC_HW
/*******************************************************************/
MCU_API_status_t MCU_API_compute_crc16(sfx_u8 *data, sfx_u8 data_size, sfx_u16 polynom, sfx_u16 *crc) {
    // Local variables.
    MCU_API_status_t status = MCU_API_SUCCESS;
    SIGFOX_UNUSED(data);
    SIGFOX_UNUSED(data_size);
    SIGFOX_UNUSED(polynom);
    SIGFOX_UNUSED(crc);
    SIGFOX_RETURN();
}
#endif

#if (defined SIGFOX_EP_CRC_HW) && (defined SIGFOX_EP_BIDIRECTIONAL)
/*******************************************************************/
MCU_API_status_t MCU_API_compute_crc8(sfx_u8 *data, sfx_u8 data_size, sfx_u16 polynom, sfx_u8 *crc) {
    // Local variables.
    MCU_API_status_t status = MCU_API_SUCCESS;
    SIGFOX_UNUSED(data);
    SIGFOX_UNUSED(data_size);
    SIGFOX_UNUSED(polynom);
    SIGFOX_UNUSED(crc);
    SIGFOX_RETURN();
}
#endif

/*******************************************************************/
MCU_API_status_t MCU_API_get_ep_id(sfx_u8* ep_id, sfx_u8 ep_id_size_bytes) {
    // Local variables.
    MCU_API_status_t status = MCU_API_SUCCESS;
    NVM_status_t nvm_status = NVM_SUCCESS;
    uint8_t idx = 0;
    // Get device ID.
    for (idx = 0; idx < ep_id_size_bytes; idx++) {
        nvm_status = NVM_read_byte((NVM_ADDRESS_SIGFOX_EP_ID + idx), &(ep_id[idx]));
        NVM_stack_exit_error(ERROR_BASE_NVM, (MCU_API_status_t) MCU_API_ERROR_DRIVER_NVM);
    }
errors:
    SIGFOX_RETURN();
}

#ifndef SIGFOX_EP_AES_HW
/*******************************************************************/
MCU_API_status_t MCU_API_get_ep_key(sfx_u8 *ep_key, sfx_u8 ep_key_size_bytes) {
    // Local variables.
    MCU_API_status_t status = MCU_API_SUCCESS;
    SIGFOX_UNUSED(ep_key);
    SIGFOX_UNUSED(ep_key_size_bytes);
    SIGFOX_RETURN();
}
#endif

/*******************************************************************/
MCU_API_status_t MCU_API_get_nvm(sfx_u8* nvm_data, sfx_u8 nvm_data_size_bytes) {
    // Local variables.
    MCU_API_status_t status = MCU_API_SUCCESS;
    NVM_status_t nvm_status = NVM_SUCCESS;
    uint8_t idx = 0;
    // Read data.
    for (idx = 0; idx < nvm_data_size_bytes; idx++) {
        nvm_status = NVM_read_byte((NVM_ADDRESS_SIGFOX_EP_LIB_DATA + idx), &(nvm_data[idx]));
        NVM_stack_exit_error(ERROR_BASE_NVM, (MCU_API_status_t) MCU_API_ERROR_DRIVER_NVM);
    }
errors:
    SIGFOX_RETURN();
}

/*******************************************************************/
MCU_API_status_t MCU_API_set_nvm(sfx_u8* nvm_data, sfx_u8 nvm_data_size_bytes) {
    // Local variables.
    MCU_API_status_t status = MCU_API_SUCCESS;
    NVM_status_t nvm_status = NVM_SUCCESS;
    uint8_t idx = 0;
    // Write data.
    for (idx = 0; idx < nvm_data_size_bytes; idx++) {
        nvm_status = NVM_write_byte((NVM_ADDRESS_SIGFOX_EP_LIB_DATA + idx), nvm_data[idx]);
        NVM_stack_exit_error(ERROR_BASE_NVM, (MCU_API_status_t) MCU_API_ERROR_DRIVER_NVM);
    }
errors:
    SIGFOX_RETURN();
}

#if (defined SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE) || (defined SIGFOX_EP_BIDIRECTIONAL)
/*******************************************************************/
MCU_API_status_t MCU_API_get_voltage_temperature(sfx_u16* voltage_idle_mv, sfx_u16* voltage_tx_mv, sfx_s16* temperature_tenth_degrees) {
    // Local variables.
    MCU_API_status_t status = MCU_API_SUCCESS;
    ANALOG_status_t analog_status = ANALOG_SUCCESS;
    int32_t data = 0;
    // Perform analog measurements.
    POWER_enable(POWER_REQUESTER_ID_MCU_API, POWER_DOMAIN_ANALOG, LPTIM_DELAY_MODE_SLEEP);
    // Get MCU supply voltage.
    analog_status = ANALOG_convert_channel(ANALOG_CHANNEL_MCU_VOLTAGE_MV, &data);
    ANALOG_stack_exit_error(ERROR_BASE_ANALOG, (MCU_API_status_t) MCU_API_ERROR_DRIVER_ANALOG);
    (*voltage_idle_mv) = (sfx_u16) data;
    (*voltage_tx_mv) = (sfx_u16) data;
    // Get MCU internal temperature.
    analog_status = ANALOG_convert_channel(ANALOG_CHANNEL_MCU_TEMPERATURE_DEGREES, &data);
    ANALOG_stack_exit_error(ERROR_BASE_ANALOG, (MCU_API_status_t) MCU_API_ERROR_DRIVER_ANALOG);
    (*temperature_tenth_degrees) = ((sfx_s16) data) * 10;
errors:
    POWER_disable(POWER_REQUESTER_ID_MCU_API, POWER_DOMAIN_ANALOG);
    SIGFOX_RETURN();
}
#endif

#ifdef SIGFOX_EP_VERBOSE
/*******************************************************************/
MCU_API_status_t MCU_API_get_initial_pac(sfx_u8 *initial_pac, sfx_u8 initial_pac_size_bytes) {
    // Local variables.
    MCU_API_status_t status = MCU_API_SUCCESS;
    SIGFOX_UNUSED(initial_pac);
    SIGFOX_UNUSED(initial_pac_size_bytes);
    SIGFOX_RETURN();
}
#endif

#if (defined SIGFOX_EP_TIMER_REQUIRED) && (defined SIGFOX_EP_LATENCY_COMPENSATION) && (defined SIGFOX_EP_BIDIRECTIONAL)
/*******************************************************************/
MCU_API_status_t MCU_API_get_latency(MCU_API_latency_t latency_type, sfx_u32* latency_ms) {
    // Local variables.
    MCU_API_status_t status = MCU_API_SUCCESS;
    // Check parameter.
    if (latency_type >= MCU_API_LATENCY_LAST) {
        SIGFOX_EXIT_ERROR((MCU_API_status_t) MCU_API_ERROR_LATENCY_TYPE);
    }
    // Set latency.
    (*latency_ms) = MCU_API_LATENCY_MS[latency_type];
errors:
    SIGFOX_RETURN();
}
#endif

#ifdef SIGFOX_EP_VERBOSE
/*******************************************************************/
MCU_API_status_t MCU_API_get_version(sfx_u8 **version, sfx_u8 *version_size_char) {
    // Local variables.
    MCU_API_status_t status = MCU_API_SUCCESS;
    SIGFOX_UNUSED(version);
    SIGFOX_UNUSED(version_size_char);
    SIGFOX_RETURN();
}
#endif

#ifdef SIGFOX_EP_ERROR_CODES
/*******************************************************************/
void MCU_API_error(void) {
#if (defined SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE) || (defined SIGFOX_EP_BIDIRECTIONAL)
    // Force all power off.
    POWER_disable(POWER_REQUESTER_ID_MCU_API, POWER_DOMAIN_ANALOG);
#endif
}
#endif
