/*
 * mode.h
 *
 *  Created on: 22 dec. 2018
 *      Author: Ludo
 */

#ifndef MODE_H
#define MODE_H

#include "sigfox_api.h"

/*** Wheather station mode ***/

//#define ATM 		// AT command mode.
#define IM 			// Intermittent mode.
//#define CM 		// Continuous mode.

/*** Debug mode ***/

//#define DEBUG		// Use LED and programming pins for debug purpose if defined.

/*** Error management ***/

#if ((defined ATM && defined IM) || \
	 (defined ATM && defined CM) || \
	 (defined IM && defined CM))
#error "Only 1 weather station mode must be selected."
#endif

#endif /* MODE_H */
