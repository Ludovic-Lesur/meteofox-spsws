/*
 * ultimeter_driver_flags.h
 *
 *  Created on: 13 may 2025
 *      Author: Ludo
 */

#ifndef __ULTIMETER_DRIVER_FLAGS_H__
#define __ULTIMETER_DRIVER_FLAGS_H__

#include "lptim.h"
#include "spsws_flags.h"

/*** ULTIMETER driver compilation flags ***/

#if (!(defined SPSWS_WIND_RAINFALL_MEASUREMENTS) || !(defined SPSWS_WIND_VANE_ULTIMETER))
#define ULTIMETER_DRIVER_DISABLE
#endif

#define ULTIMETER_DRIVER_TIMER_ERROR_BASE_LAST  LPTIM_ERROR_BASE_LAST

#define ULTIMETER_DRIVER_TIMER_STEP_US          488

#endif /* __ULTIMETER_DRIVER_FLAGS_H__ */
