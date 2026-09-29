/*
 * si1133_driver_flags.h
 *
 *  Created on: 31 aug. 2024
 *      Author: Ludo
 */

#ifndef __SI1133_DRIVER_FLAGS_H__
#define __SI1133_DRIVER_FLAGS_H__

#include "i2c.h"
#include "lptim.h"

/*** SI1133 driver compilation flags ***/

#define SI1133_DRIVER_I2C_ERROR_BASE_LAST               I2C_ERROR_BASE_LAST
#define SI1133_DRIVER_DELAY_ERROR_BASE_LAST             LPTIM_ERROR_BASE_LAST

#define SI1133_DRIVER_CALIBRATION_MODE                  0

#define SI1133_DRIVER_CALIBRATION_LUX_GAIN_NUMERATOR    1
#define SI1133_DRIVER_CALIBRATION_LUX_GAIN_DENOMINATOR  1

#define SI1133_DRIVER_CALIBRATION_UVI_GAIN_NUMERATOR    1
#define SI1133_DRIVER_CALIBRATION_UVI_GAIN_DENOMINATOR  1

#endif /* __SI1133_DRIVER_FLAGS_H__ */
