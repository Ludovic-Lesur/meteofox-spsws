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
    NVM_ADDRESS_SIGFOX_EP_ID = 0,
    NVM_ADDRESS_SIGFOX_EP_KEY = (NVM_ADDRESS_SIGFOX_EP_ID + SIGFOX_EP_ID_SIZE_BYTES),
    NVM_ADDRESS_SIGFOX_EP_LIB_DATA = (NVM_ADDRESS_SIGFOX_EP_KEY + SIGFOX_EP_KEY_SIZE_BYTES),
    NVM_ADDRESS_PREVIOUS_WAKE_UP_YEAR = (NVM_ADDRESS_SIGFOX_EP_LIB_DATA + SIGFOX_NVM_DATA_SIZE_BYTES),
    NVM_ADDRESS_PREVIOUS_WAKE_UP_MONTH = (NVM_ADDRESS_PREVIOUS_WAKE_UP_YEAR + 2),
    NVM_ADDRESS_PREVIOUS_WAKE_UP_DATE,
    NVM_ADDRESS_PREVIOUS_WAKE_UP_HOUR,
    NVM_ADDRESS_PREVIOUS_GEOLOC_YEAR,
    NVM_ADDRESS_PREVIOUS_GEOLOC_MONTH = (NVM_ADDRESS_PREVIOUS_GEOLOC_YEAR + 2),
    NVM_ADDRESS_PREVIOUS_GEOLOC_DATE,
} NVM_address_t;

#endif /* __NVM_ADDRESS_H__ */
