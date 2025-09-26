/*
 * nvm_address.h
 *
 *  Created on: 11 aug. 2024
 *      Author: Ludo
 */

#ifndef __NVM_ADDRESS_H__
#define __NVM_ADDRESS_H__

#include "sigfox_types.h"

/*!******************************************************************
 * \enum NVM_address_t
 * \brief NVM address mapping.
 *******************************************************************/
typedef enum {
    // Sigfox library.
    NVM_ADDRESS_SIGFOX_EP_ID = 0,
    NVM_ADDRESS_SIGFOX_EP_KEY = (NVM_ADDRESS_SIGFOX_EP_ID + SIGFOX_EP_ID_SIZE_BYTES),
    NVM_ADDRESS_SIGFOX_EP_LIB_DATA = (NVM_ADDRESS_SIGFOX_EP_KEY + SIGFOX_EP_KEY_SIZE_BYTES),
    // Last wake-up timestamp.
    NVM_ADDRESS_LAST_WAKE_UP_YEAR = (NVM_ADDRESS_SIGFOX_EP_LIB_DATA + SIGFOX_NVM_DATA_SIZE_BYTES),
    NVM_ADDRESS_LAST_WAKE_UP_MONTH = (NVM_ADDRESS_LAST_WAKE_UP_YEAR + 2),
    NVM_ADDRESS_LAST_WAKE_UP_DATE,
    NVM_ADDRESS_LAST_WAKE_UP_HOUR,
    // Last geolocation timestamp and status.
    NVM_ADDRESS_LAST_GEOLOC_YEAR,
    NVM_ADDRESS_LAST_GEOLOC_MONTH = (NVM_ADDRESS_LAST_GEOLOC_YEAR + 2),
    NVM_ADDRESS_LAST_GEOLOC_DATE,
    NVM_ADDRESS_LAST_GEOLOC_STATUS,
    // Last downlink timestamp and status.
    NVM_ADDRESS_LAST_DOWNLINK_YEAR,
    NVM_ADDRESS_LAST_DOWNLINK_MONTH = (NVM_ADDRESS_LAST_DOWNLINK_YEAR + 2),
    NVM_ADDRESS_LAST_DOWNLINK_DATE,
    NVM_ADDRESS_LAST_DOWNLINK_STATUS
} NVM_address_t;

#endif /* __NVM_ADDRESS_H__ */
