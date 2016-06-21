/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 */


/**@file
 *
 * @defgroup ble_sdk_lib_advertising Advertising Module
 * @{
 * @ingroup  ble_sdk_lib
 * @brief    Module for handling connectable BLE advertising.
 *
 * @details  The Advertising Module handles connectable advertising for your application. It can 
 *           be configured with advertising modes to suit most typical use cases.
 *           Your main application can react to changes in advertising modes
 *           if an event handler is provided.
 *
 * @note     The Advertising Module supports only applications with a single peripheral link.
 *
 * The application must propagate BLE stack events to this module by calling
 * @ref ble_advertising_on_ble_evt() and system events by calling
 * @ref ble_advertising_on_sys_evt().
 *
 */

#ifndef W_ESS_ADVERTISING_H__
#define W_ESS_ADVERTISING_H__

#include <stdint.h>
#include "ble_gattc.h"
#include "ble.h"
#include "nrf_error.h"
#include "ble_advdata.h"

/**@brief Advertising modes.
*/
typedef enum
{
    ESS_ADV_MODE_IDLE,
    ESS_ADV_MODE_1,
    ESS_ADV_MODE_2,
    ESS_ADV_MODE_3,
} ess_adv_mode_t;

/**@brief Advertising events.
 *
 * @details These events are propagated to the main application if a handler was provided during
 *          initialization of the Advertising Module. Events for modes that are not used can be
 *          ignored. Similarly, BLE_ADV_EVT_WHITELIST_REQUEST and BLE_ADV_EVT_PEER_ADDR_REQUEST
 *          can be ignored if whitelist and direct advertising is not used.
 */
typedef enum
{
    ESS_ADV_EVT_IDLE,
    ESS_ADV_EVT_MODE_1,
    ESS_ADV_EVT_MODE_2,
    ESS_ADV_EVT_MODE_3,
    ESS_ADV_EVT_WHITELIST_REQUEST,   /**< Request a whitelist from the main application. For whitelist advertising to work, the whitelist must be set when this event occurs. */
    ESS_ADV_EVT_PEER_ADDR_REQUEST,   /**< Request a peer address from the main application. For directed advertising to work, the peer address must be set when this event occurs. */
} ess_adv_evt_t;

/**@brief Options for the different advertisement modes.
 *
 * @details This structure is used to enable or disable advertising modes and to configure time-out
 *          periods and advertising intervals.
 */
typedef struct
{
    uint32_t ess_adv_unbonded_fast_interval;
    uint32_t ess_adv_unbonded_fast_timeout;
    uint32_t ess_adv_unbonded_slow_interval;
    uint32_t ess_adv_unbonded_slow_timeout;
    uint32_t ess_adv_bonded_wl_on_interval;
    uint32_t ess_adv_bonded_wl_on_timeout;
    uint32_t ess_adv_bonded_wl_off_fast_interval;
    uint32_t ess_adv_bonded_wl_off_fast_timeout;
    uint32_t ess_adv_bonded_wl_off_slow_interval;
    uint32_t ess_adv_bonded_wl_off_slow_timeout;
} ess_adv_modes_config_t;


/**@brief BLE advertising event handler type. */
typedef void (*ess_advertising_evt_handler_t) (ess_adv_evt_t const adv_evt);

/**@brief BLE advertising error handler type. */
typedef void (*ess_advertising_error_handler_t) (uint32_t nrf_error);

/**@brief Initialization parameters for the Advertising Module.
 * @details This structure is used to pass advertising options, advertising data, and an event handler to the Advertising Module during initialization. */
typedef struct
{
    ess_adv_modes_config_t        options;     /**< Parameters for advertising modes.*/
    ble_advdata_t                 advdata;     /**< Advertising data. */
    ess_advertising_evt_handler_t evt_handler; /**< Event handler. */
} ess_adv_init_t;


/**@brief Function for handling BLE events.
 *
 * @details This function must be called from the BLE stack event dispatcher for
 *          the module to handle BLE events that are relevant for the Advertising Module.
 *
 * @param[in] p_ble_evt BLE stack event.
 */
void ess_advertising_on_ble_evt(const ble_evt_t * const p_ble_evt);


/**@brief Function for handling system events.
 *
 * @details This function must be called to handle system events that are relevant
 *          for the Advertising Module. Specifically, the advertising module can not use the
 *          softdevice as long as there are pending writes to the flash memory. This
 *          event handler is designed to delay advertising until there is no flash operation.
 *
 * @param[in] sys_evt  System event.
 */
void ess_advertising_on_sys_evt(uint32_t sys_evt);


/**@brief Function for initializing the Advertising Module.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 *          The supplied advertising data is copied to a local structure and is manipulated
 *          depending on what advertising modes are started in @ref ble_advertising_start.
 *
 * @param[in] p_advdata     Advertising data: name, appearance, discovery flags, and more.
 * @param[in] p_srdata      Scan response data: Supplement to advertising data.
 * @param[in] p_config      Select which advertising modes and intervals will be utilized.
 * @param[in] evt_handler   Event handler that will be called upon advertising events.
 * @param[in] error_handler Error handler that will propogate internal errors to the main applications.
 *
 * @retval NRF_SUCCESS If initialization was successful. Otherwise, an error code is returned.
 */
uint32_t ess_advertising_init(ble_advdata_t const                 * p_advdata,
                              ble_advdata_t const                 * p_srdata,
                              ess_adv_modes_config_t const        * p_config,
                              ess_advertising_evt_handler_t const   evt_handler,
                              ess_advertising_error_handler_t const error_handler);


/**@brief Function for starting advertising.
 *
 * @details You can start advertising in any of the advertising modes that you enabled
 *          during initialization.
 *
 * @param[in] advertising_mode  Advertising mode.
 *
 * @retval @ref NRF_SUCCESS On success, else an error code indicating reason for failure.
 * @retval @ref NRF_ERROR_INVALID_STATE                             
 */
uint32_t ess_advertising_start(void);

/**@brief Function for setting the peer address.
 *
 * @details The peer address must be set by the application upon receiving a
 *          @ref BLE_ADV_EVT_PEER_ADDR_REQUEST event. Without the peer address, the directed
 *          advertising mode will not be run.
 *
 * @param[in] p_peer_addr  Pointer to a peer address.
 *
 * @retval @ref NRF_SUCCESS Successfully stored the peer address pointer in the advertising module.
 * @retval @ref NRF_ERROR_INVALID_STATE If a reply was not expected.
 */
//uint32_t ble_advertising_peer_addr_reply(ble_gap_addr_t * p_peer_addr);


/**@brief Function for setting a whitelist.
 *
 * @details The whitelist must be set by the application upon receiving a
 *          @ref BLE_ADV_EVT_WHITELIST_REQUEST event. Without the whitelist, the whitelist
 *          advertising for fast and slow modes will not be run.
 *
 * @param[in] p_whitelist  Pointer to a whitelist.
 *
 * @retval @ref NRF_SUCCESS Successfully stored pointers to the whitelist into the advertising module.
 * @retval @ref NRF_ERROR_INVALID_STATE If a reply was not expected.
 */
uint32_t ess_advertising_whitelist_reply(ble_gap_whitelist_t * p_whitelist);


/**@brief Function for disabling whitelist advertising.
 *
 * @details This function temporarily disables whitelist advertising.
 *          Calling this function resets the current time-out countdown.
 *
 * @retval @ref NRF_SUCCESS On success, else an error message propogated from the Softdevice.
 */
//uint32_t ble_advertising_restart_without_whitelist(void);
/** @} */

#endif // BLE_ADVERTISING_H__

/** @} */
