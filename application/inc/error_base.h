/*
 * error_base.h
 *
 *  Created on: 12 mar. 2022
 *      Author: Ludo
 */

#ifndef __ERROR_BASE_H__
#define __ERROR_BASE_H__

// Peripherals.
#include "aes.h"
#include "iwdg.h"
#include "lptim.h"
#include "nvm.h"
#include "rcc.h"
#include "rtc.h"
#include "tim.h"
// Utils.
#include "error.h"
#include "maths.h"
#include "terminal.h"
// Components.
#include "dps310.h"
#include "max111xx.h"
#include "neom8x.h"
#include "sen15901.h"
#include "sht3x.h"
#include "si1133.h"
#include "sx1232.h"
#include "ultimeter.h"
// Middleware.
#include "analog.h"
#include "cli.h"
#include "gps.h"
#include "power.h"
#include "rfe.h"
#include "sigfox_error.h"

/*** ERROR BASE structures ***/

/*!******************************************************************
 * \enum ERROR_base_t
 * \brief Board error bases.
 *******************************************************************/
typedef enum {
    SUCCESS = 0,
    ERROR_DL_OP_CODE,
    // Peripherals.
    ERROR_BASE_AES = ERROR_BASE_STEP,
    ERROR_BASE_IWDG = (ERROR_BASE_AES + AES_ERROR_BASE_LAST),
    ERROR_BASE_LPTIM = (ERROR_BASE_IWDG + IWDG_ERROR_BASE_LAST),
    ERROR_BASE_NVM = (ERROR_BASE_LPTIM + LPTIM_ERROR_BASE_LAST),
    ERROR_BASE_RCC = (ERROR_BASE_NVM + NVM_ERROR_BASE_LAST),
    ERROR_BASE_RTC = (ERROR_BASE_RCC + RCC_ERROR_BASE_LAST),
    ERROR_BASE_TIM_MCU_API = (ERROR_BASE_RTC + RTC_ERROR_BASE_LAST),
    ERROR_BASE_TIM_RF_API = (ERROR_BASE_TIM_MCU_API + TIM_ERROR_BASE_LAST),
    // Utils.
    ERROR_BASE_MATH = (ERROR_BASE_TIM_RF_API + TIM_ERROR_BASE_LAST),
    ERROR_BASE_TERMINAL_AT = (ERROR_BASE_MATH + MATH_ERROR_BASE_LAST),
    // Components.
    ERROR_BASE_DPS310 = (ERROR_BASE_TERMINAL_AT + TERMINAL_ERROR_BASE_LAST),
    ERROR_BASE_MAX11136 = (ERROR_BASE_DPS310 + DPS310_ERROR_BASE_LAST),
    ERROR_BASE_NEOM8N = (ERROR_BASE_MAX11136 + MAX111XX_ERROR_BASE_LAST),
    ERROR_BASE_SEN15901 = (ERROR_BASE_NEOM8N + NEOM8X_ERROR_BASE_LAST),
    ERROR_BASE_SHT3X = (ERROR_BASE_SEN15901 + SEN15901_ERROR_BASE_LAST),
    ERROR_BASE_SHT30_INTERNAL = (ERROR_BASE_SHT3X + SHT3X_ERROR_BASE_LAST),
    ERROR_BASE_SHT30_EXTERNAL = (ERROR_BASE_SHT30_INTERNAL + SHT3X_ERROR_BASE_LAST),
    ERROR_BASE_SI1133 = (ERROR_BASE_SHT30_EXTERNAL + SHT3X_ERROR_BASE_LAST),
    ERROR_BASE_SX1232 = (ERROR_BASE_SI1133 + SI1133_ERROR_BASE_LAST),
    ERROR_BASE_ULTIMETER = (ERROR_BASE_SX1232 + SX1232_ERROR_BASE_LAST),
    // Middleware.
    ERROR_BASE_ANALOG = (ERROR_BASE_ULTIMETER + ULTIMETER_ERROR_BASE_LAST),
    ERROR_BASE_CLI = (ERROR_BASE_ANALOG + ANALOG_ERROR_BASE_LAST),
    ERROR_BASE_GPS = (ERROR_BASE_CLI + CLI_ERROR_BASE_LAST),
    ERROR_BASE_POWER = (ERROR_BASE_GPS + GPS_ERROR_BASE_LAST),
    ERROR_BASE_RFE = (ERROR_BASE_POWER + POWER_ERROR_BASE_LAST),
    ERROR_BASE_SIGFOX_EP_LIB = (ERROR_BASE_RFE + RFE_ERROR_BASE_LAST),
    ERROR_BASE_SIGFOX_EP_ADDON_RFP = (ERROR_BASE_SIGFOX_EP_LIB + (SIGFOX_ERROR_SOURCE_LAST * ERROR_BASE_STEP)),
    // Last base value.
    ERROR_BASE_LAST = (ERROR_BASE_SIGFOX_EP_ADDON_RFP + ERROR_BASE_STEP)
} ERROR_base_t;

#endif /* __ERROR_BASE_H__ */
