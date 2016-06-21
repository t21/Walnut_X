/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_app_gls_main main.c
 * @{
 * @ingroup ble_sdk_app_gls
 * @brief Glucose Meter service Sample Application
 *
 * This file contains the source code for a sample application using the Glucose Meter service
 * (and also Battery and Device Information services). This application uses the
 * @ref srvlib_conn_params module.
 */

/**
 * TODO:
 * - Configure board
 * - Remove use of bsp
 * - 
 *
 */
 
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
//#include "nrf_gpio.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
//#include "ble_advertising.h"
#include "ble_dis.h"
#include "ble_bas.h"
#include "ble_conn_params.h"
#include "ble_radio_notification.h"
#include "boards.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "peer_manager.h"
//#include "app_button.h"
//#include "pstorage.h"
#include "app_uart.h"
#include "app_button.h"
//#include "bsp.h"
//#include "bsp_btn_ble.h"
#include "nrf_log.h"
#include "fds.h"
#include "fstorage.h"
#include "ble_conn_state.h"
//#include "app_trace.h"
#include "service_if.h"
#include "app_twi.h"
#include "app_util_platform.h"
#include "nrf_delay.h"

#include "sensors.h"
#include "ble_ess_climate.h"
#include "w_ess_advertising.h"

#define CENTRAL_LINK_COUNT              0                                          /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                          /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define APP_LOG NRF_LOG

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                          /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define MODEL_NUMBER                   "nRF51"                                     /**< Model Number string. Will be passed to Device Information Service. */
#define MANUFACTURER_ID                0x55AA55AA55                                /**< DUMMY Manufacturer ID. Will be passed to Device Information Service. You shall use the ID for your Company*/
#define ORG_UNIQUE_ID                  0xEEBBEE                                    /**< DUMMY Organisation Unique ID. Will be passed to Device Information Service. You shall use the Organisation Unique ID relevant for your Company */
#define DEVICE_NAME                      "WalnutX"                               /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                "Sigma Connectivity AB"                      /**< Manufacturer. Will be passed to Device Information Service. */

#define APP_ADV_UNBONDED_FAST_INTERVAL   MSEC_TO_UNITS(60, UNIT_0_625_MS)
#define APP_ADV_UNBONDED_FAST_TIMEOUT    30                                         // Should be 30s according to BT spec
#define APP_ADV_UNBONDED_SLOW_INTERVAL   MSEC_TO_UNITS(1200, UNIT_0_625_MS)
#define APP_ADV_UNBONDED_SLOW_TIMEOUT    270
#define APP_ADV_BONDED_WL_ON_INTERVAL    MSEC_TO_UNITS(60, UNIT_0_625_MS)
#define APP_ADV_BONDED_WL_ON_TIMEOUT     10                                         // Should be 30s according to BT spec
#define APP_ADV_BONDED_WL_OFF_FAST_INTERVAL     MSEC_TO_UNITS(60, UNIT_0_625_MS)
#define APP_ADV_BONDED_WL_OFF_FAST_TIMEOUT      20                                         // Should be 30s according to BT spec
#define APP_ADV_BONDED_WL_OFF_SLOW_INTERVAL     MSEC_TO_UNITS(1200, UNIT_0_625_MS)
#define APP_ADV_BONDED_WL_OFF_SLOW_TIMEOUT      2

#define APP_TIMER_PRESCALER            0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE        4                                           /**< Size of timer operation queues. */

#define SECURITY_REQUEST_DELAY         APP_TIMER_TICKS(400, APP_TIMER_PRESCALER)  /**< Delay after connection until Security Request is sent, if necessary (ticks). */

#define BATTERY_LEVEL_MEAS_INTERVAL    APP_TIMER_TICKS(60000, APP_TIMER_PRESCALER) /**< Battery level measurement interval (ticks). */
#define MIN_BATTERY_LEVEL              81                                          /**< Minimum battery level as returned by the simulated measurement function. */
#define MAX_BATTERY_LEVEL              100                                         /**< Maximum battery level as returned by the simulated measurement function. */
#define BATTERY_LEVEL_INCREMENT        1                                           /**< Value by which the battery level is incremented/decremented for each call to the simulated measurement function. */

#define MIN_CONN_INTERVAL              MSEC_TO_UNITS(10, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (10 ms). */
#define MAX_CONN_INTERVAL              MSEC_TO_UNITS(100, UNIT_1_25_MS)            /**< Maximum acceptable connection interval (100 ms) */
#define SLAVE_LATENCY                  9                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT               MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAM_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                 1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                 1                                           /**< Man In The Middle protection required (applicable when display module is detected). */
#define SEC_PARAM_LESC                 0                                           /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS             0                                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES      BLE_GAP_IO_CAPS_DISPLAY_ONLY                /**< Display I/O capabilities. */
#define SEC_PARAM_OOB                  0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE         16                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE         16                                          /**< Maximum encryption key size. */

//#define PASSKEY_TXT                    "Passkey:"                                  /**< Message to be displayed together with the pass-key. */
#define PASSKEY_TXT_LENGTH             8                                           /**< Length of message to be displayed together with the pass-key. */
#define PASSKEY_LENGTH                 6                                           /**< Length of pass-key received by the stack for display. */

#define DEAD_BEEF                      0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define APP_FEATURE_NOT_SUPPORTED      BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */

// TODO: Check if this is a good value
#define TWI_MAX_PENDING_TRANSACTIONS    5

#define BLE_UUID_ENVIRONMENTAL_SENSING_SERVICE  0x181A

#define APP_PRIORITY_LOW 3
#define INITIAL_TX_POWER_LEVEL 4
#define UICR_PIN_ADDRESS 0x10001084UL

#define BUTTON_DEBOUNCE_MS 100

// GPIOs
#define BUTTON1 17

typedef enum {
    W_ADV_STATE_IDLE,
    W_ADV_STATE_ADVERTISING,
} w_adv_state_t;


static uint16_t                        m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */
static ble_bas_t                       m_bas;                                      /**< Structure used to identify the battery service. */

APP_TIMER_DEF(m_battery_timer_id);                                                /**< Battery timer. */
APP_TIMER_DEF(m_sec_req_timer_id);                                                /**< Security Request timer. */

static ble_uuid_t m_adv_uuids[] = {{BLE_UUID_ENVIRONMENTAL_SENSING_SERVICE, BLE_UUID_TYPE_BLE}}; /**< Universally unique service identifiers. */
pm_peer_id_t peer_to_be_deleted = PM_PEER_ID_INVALID;
static app_twi_t m_app_twi = APP_TWI_INSTANCE(0);
static ess_adv_modes_config_t m_adv_modes_config;
static w_adv_state_t m_adv_state;

static void advertising_init(void);
static void advertising_start(void);


#ifdef  __GNUC__
    // This is required if one wants to use floating-point values in 'printf'
    // (by default this feature is not linked together with newlib-nano).
    // Please note, however, that this adds about 13 kB code footprint...
    asm (".global _printf_float");
#endif


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;

    switch(p_evt->evt_id)
    {
        case PM_EVT_BONDED_PEER_CONNECTED:
        {
            NRF_LOG_PRINTF_DEBUG("Connected to previously bonded device\r\n");
            // Start Security Request timer.
            err_code = app_timer_start(m_sec_req_timer_id, SECURITY_REQUEST_DELAY, NULL);
            APP_ERROR_CHECK(err_code);
            err_code = pm_peer_rank_highest(p_evt->peer_id);
            if (err_code != NRF_ERROR_BUSY)
            {
                    APP_ERROR_CHECK(err_code);
            }
        }break;//PM_EVT_BONDED_PEER_CONNECTED

        case PM_EVT_CONN_SEC_START:
            break;//PM_EVT_CONN_SEC_START

        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
            /*Check if the link is authenticated (meaning at least MITM)*/
            pm_conn_sec_status_t conn_sec_status;
            err_code = pm_conn_sec_status_get(p_evt->conn_handle, &conn_sec_status);
            APP_ERROR_CHECK(err_code);
            if (!conn_sec_status.mitm_protected)
            {
                APP_LOG("Collector did not use MITM, disconnecting\r\n");
                /*The peer did not use MITM, disconnect*/
                err_code = pm_peer_id_get(m_conn_handle, &peer_to_be_deleted);
                APP_ERROR_CHECK(err_code);
                err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);
//                m_state = W_ADV_STATE_UNBONDED;
            }    
            else
            {
                NRF_LOG_PRINTF_DEBUG("Link secured. Role: %d. conn_handle: %d, Procedure: %d\r\n",
                               ble_conn_state_role(p_evt->conn_handle),
                               p_evt->conn_handle,
                               p_evt->params.conn_sec_succeeded.procedure);
                err_code = pm_peer_rank_highest(p_evt->peer_id);
                if (err_code != NRF_ERROR_BUSY)
                {
                        APP_ERROR_CHECK(err_code);
                }
//                m_state = W_ADV_STATE_BONDED;
            }
        }break;//PM_EVT_CONN_SEC_SUCCEEDED

        case PM_EVT_CONN_SEC_FAILED:
        {
            /** In some cases, when securing fails, it can be restarted directly. Sometimes it can
             *  be restarted, but only after changing some Security Parameters. Sometimes, it cannot
             *  be restarted until the link is disconnected and reconnected. Sometimes it is
             *  impossible, to secure the link, or the peer device does not support it. How to
             *  handle this error is highly application dependent. */
            APP_LOG("link secure failed! ");        
            switch (p_evt->params.conn_sec_failed.error)
            {
                case PM_CONN_SEC_ERROR_PIN_OR_KEY_MISSING:
                    NRF_LOG_DEBUG("error: PM_CONN_SEC_ERROR_PIN_OR_KEY_MISSING");
                    break;//PM_CONN_SEC_ERROR_PIN_OR_KEY_MISSING

                case PM_CONN_SEC_ERROR_MIC_FAILURE:
                    NRF_LOG_DEBUG("error: PM_CONN_SEC_ERROR_MIC_FAILURE");
                    break;//PM_CONN_SEC_ERROR_MIC_FAILURE

                case PM_CONN_SEC_ERROR_DISCONNECT :
                    NRF_LOG_DEBUG("error: PM_CONN_SEC_ERROR_DISCONNECT ");
                    break;//PM_CONN_SEC_ERROR_DISCONNECT

                case PM_CONN_SEC_ERROR_SMP_TIMEOUT:
                    NRF_LOG_DEBUG("error: PM_CONN_SEC_ERROR_SMP_TIMEOUT");
                    break;//PM_CONN_SEC_ERROR_SMP_TIMEOUT

                default:
                    NRF_LOG_DEBUG("unknown error");
                    break;
            }
            APP_LOG("\r\nDisconnecting\r\n");
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
        }break;//PM_EVT_CONN_SEC_FAILED

        case PM_EVT_CONN_SEC_CONFIG_REQ:
        {
            // Reject pairing request from an already bonded peer.
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
        }break;//PM_EVT_CONN_SEC_CONFIG_REQ

        case PM_EVT_STORAGE_FULL:
        {
            // Run garbage collection on the flash.
            err_code = fds_gc();
            if (err_code == FDS_ERR_BUSY || err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
            {
                // Retry.
            }
            else
            {
                APP_ERROR_CHECK(err_code);
            }
        }break;//PM_EVT_STORAGE_FULL

        case PM_EVT_ERROR_UNEXPECTED:
            // Assert.
            APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
            break;//PM_EVT_ERROR_UNEXPECTED

        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
            break;//PM_EVT_PEER_DATA_UPDATE_SUCCEEDED

        case PM_EVT_PEER_DATA_UPDATE_FAILED:
            // Assert.
            APP_ERROR_CHECK_BOOL(false);
            break;//PM_EVT_PEER_DATA_UPDATE_FAILED

        case PM_EVT_PEER_DELETE_SUCCEEDED:
            break;//PM_EVT_PEER_DELETE_SUCCEEDED

        case PM_EVT_PEER_DELETE_FAILED:
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
            break;//PM_EVT_PEER_DELETE_FAILED

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            NRF_LOG_PRINTF_DEBUG("%s: PM_EVT_PEERS_DELETE_SUCCEEDED\r\n", __func__);
//            m_erasing_bonds = false;
//            advertising_init();
//            advertising_start();
            break;

        case PM_EVT_PEERS_DELETE_FAILED:
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
            break;//PM_EVT_PEERS_DELETE_FAILED

        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
            break;//PM_EVT_LOCAL_DB_CACHE_APPLIED

        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
            // The local database has likely changed, send service changed indications.
            pm_local_database_has_changed();
            break;//PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED

        case PM_EVT_SERVICE_CHANGED_IND_SENT:
            break;//PM_EVT_SERVICE_CHANGED_IND_SENT

        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
            break;//PM_EVT_SERVICE_CHANGED_IND_CONFIRMED

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for handling Service errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
//static void service_error_handler(uint32_t nrf_error)
//{
//    APP_ERROR_HANDLER(nrf_error);
//}


/**@brief Function for performing battery measurement and updating the Battery Level characteristic
 *        in Battery Service.
 */
static void battery_level_update(void)
{
    uint32_t err_code;
    uint8_t  battery_level;

//    battery_level = (uint8_t)sensorsim_measure(&m_battery_sim_state, &m_battery_sim_cfg);
    battery_level = 37;

    err_code = ble_bas_battery_level_update(&m_bas, battery_level);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_PACKETS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }

    
//    if (m_adv_state == W_ADV_STATE_IDLE) {
//        advertising_init();
//        advertising_start();
//    }
}


/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    battery_level_update();
}


/**@brief Function for handling the Security Request timer timeout.
 *
 * @details This function will be called each time the Security Request timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void sec_req_timeout_handler(void * p_context)
{
    uint32_t             err_code;

    if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        // Initiate bonding.
        NRF_LOG_DEBUG("Start encryption\r\n");
        err_code = pm_conn_secure(m_conn_handle, false);
        if (err_code != NRF_ERROR_INVALID_STATE)
        {
            APP_ERROR_CHECK(err_code);
        }
    }
}


/**@brief Function for GPIO initialization.
 *
 * @details Initializes the GPIOs used by the application.
 */
static void gpio_init(void)
{
    nrf_gpio_cfg_output(LED_1);
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    uint32_t err_code;

    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create timers.
    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);

    // Create Security Request timer.
    err_code = app_timer_create(&m_sec_req_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                sec_req_timeout_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_UNKNOWN);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_tx_power_set(INITIAL_TX_POWER_LEVEL);
    APP_ERROR_CHECK(err_code);

    ble_opt_t ble_opt;
    uint8_t passkey[6] = { 0 };
    
    // Fetch pin from UICR
    uint8_t val = (*((uint32_t *)UICR_PIN_ADDRESS)) & 0x000000FF;
    passkey[0] = (val == 0xFF) ? 0x31 : val;
    
    val = ((*((uint32_t *)UICR_PIN_ADDRESS)) & 0x0000FF00) >> 8;
    passkey[1] = (val == 0xFF) ? 0x32 : val;
    
    val = ((*((uint32_t *)UICR_PIN_ADDRESS)) & 0x00FF0000) >> 16;
    passkey[2] = (val == 0xFF) ? 0x33 : val;
    
    val = ((*((uint32_t *)UICR_PIN_ADDRESS)) & 0xFF000000) >> 24;
    passkey[3] = (val == 0xFF) ? 0x34 : val;
    
    val = (*((uint32_t *)(UICR_PIN_ADDRESS+4))) & 0x000000FF;
    passkey[4] = (val == 0xFF) ? 0x35 : val;
    
    val = ((*((uint32_t *)(UICR_PIN_ADDRESS+4))) & 0x0000FF00) >> 8;
    passkey[5] = (val == 0xFF) ? 0x36 : val;
    
    ble_opt.gap_opt.passkey.p_passkey = passkey;
    err_code = sd_ble_opt_set(BLE_GAP_OPT_PASSKEY, &ble_opt);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Glucose, Battery and Device Information services.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_dis_init_t dis_init;
    ble_bas_init_t bas_init;

    // Initialize Battery Service.
    memset(&bas_init, 0, sizeof(bas_init));

    // Here the sec level for the Battery Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init.battery_level_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_report_read_perm);

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, MANUFACTURER_NAME);

    ble_srv_ascii_to_utf8(&dis_init.serial_num_str, MODEL_NUMBER);

    ble_dis_sys_id_t system_id;
    system_id.manufacturer_id            = MANUFACTURER_ID;
    system_id.organizationally_unique_id = ORG_UNIQUE_ID;
    dis_init.p_sys_id                    = &system_id;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting application timers.
 */
static void application_timers_start(void)
{
    uint32_t err_code;

    // Start application timers.
    err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Connection Parameter events.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail configuration parameter, but instead we use the
 *                event handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAM_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code;

#ifdef DEBUG    
    nrf_delay_ms(100);
#endif
    
    // Stop all sensor timers
    sensors_timers_stop();
    
    // TODO: Put all sensors in deep sleep
    // TODO: Stop all relevant timers?
    
    // Prepare wakeup buttons.
    err_code = app_button_disable();
    APP_ERROR_CHECK(err_code);

    // Configure buttons with sense level low as wakeup source.
    // Should already have been done, but just to make sure
    nrf_gpio_cfg_sense_input(BUTTON1, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);

    // Wait 1s for things to finish
    nrf_delay_ms(1000);
    
    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for stopping advertising.
 */
//static void advertising_stop(void)
//{
//    uint32_t err_code;

//    err_code = sd_ble_gap_adv_stop();
//    APP_ERROR_CHECK(err_code);

////    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
////    APP_ERROR_CHECK(err_code);
//}


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ess_adv_evt_t ess_adv_evt)
{
    switch (ess_adv_evt)
    {
        case ESS_ADV_EVT_MODE_1:
            NRF_LOG_PRINTF("%s - ESS_ADV_EVT_MODE_1\r\n", __func__);
            break;

        case ESS_ADV_EVT_MODE_2:
            NRF_LOG_PRINTF("%s - ESS_ADV_EVT_MODE_2\r\n", __func__);
            break;

        case ESS_ADV_EVT_MODE_3:
            NRF_LOG_PRINTF("%s - ESS_ADV_EVT_MODE_3\r\n", __func__);
            m_adv_state = W_ADV_STATE_IDLE;
            break;
       
        case ESS_ADV_EVT_IDLE:
            NRF_LOG_PRINTF("%s - ESS_ADV_EVT_IDLE\r\n", __func__);
            NRF_LOG("[APP]: Going to sleep.\r\n\r\n\r\n");
            sleep_mode_enter();
            break;

        case ESS_ADV_EVT_WHITELIST_REQUEST:
        {
            NRF_LOG_PRINTF("%s - ESS_ADV_EVT_WHITELIST_REQUEST\r\n", __func__);
            ret_code_t err_code;
            // Storage for the whitelist.
            ble_gap_irk_t * irks[8];
            ble_gap_addr_t * addrs[8];
            ble_gap_whitelist_t whitelist = {.pp_irks = irks, .pp_addrs = addrs};
            // Construct a list of peer IDs.
            pm_peer_id_t peer_ids[8];
            pm_peer_id_t peer_id = pm_next_peer_id_get(PM_PEER_ID_INVALID);
            uint32_t n_peer_ids = 0;
            while((peer_id != PM_PEER_ID_INVALID) && (n_peer_ids < 8))
            {
                peer_ids[n_peer_ids++] = peer_id;
                peer_id = pm_next_peer_id_get(peer_id);
            }
            // Create the whitelist.
            err_code = pm_whitelist_create(peer_ids, n_peer_ids, &whitelist);
            APP_ERROR_CHECK(err_code);
            // Apply the whitelist.
            err_code = ess_advertising_whitelist_reply(&whitelist);
            APP_ERROR_CHECK(err_code);
            break;
        }

        case ESS_ADV_EVT_PEER_ADDR_REQUEST:
        {
            NRF_LOG_PRINTF("%s - ESS_ADV_EVT_PEER_ADDR_REQUEST\r\n", __func__);
//            ble_gap_addr_t peer_address;

//            // Only Give peer address if we have a handle to the bonded peer.
//            if(m_bonded_peer_handle.appl_id != DM_INVALID_ID)
//            {

//                err_code = dm_peer_addr_get(&m_bonded_peer_handle, &peer_address);
//                if (err_code != (NRF_ERROR_NOT_FOUND | DEVICE_MANAGER_ERR_BASE))
//                {
//                    APP_ERROR_CHECK(err_code);

//                    err_code = ble_advertising_peer_addr_reply(&peer_address);
//                    APP_ERROR_CHECK(err_code);
//                }

//            }
            break;
        }
        
        default:
            break;
    }
}


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code = NRF_SUCCESS;                    //lint -save -e438 // Last value assigned to variable 'err_code' not used

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:
        {
            NRF_LOG_PRINTF("[APP]: %s: BLE_GAP_EVT_DISCONNECTED\r\n", __func__);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            /*check if the last connected peer had not used MITM, if so, delete its bond information*/
            if (peer_to_be_deleted != PM_PEER_ID_INVALID)
            {
                ret_code_t ret_val = pm_peer_delete(peer_to_be_deleted);
                APP_ERROR_CHECK(ret_val);
                APP_LOG("Collector's bond deleted\r\n");
                peer_to_be_deleted = PM_PEER_ID_INVALID;
            }
//            advertising_init(); // TODO: Check if this is correct, shouldn't be needed
            nrf_gpio_pin_clear(LED_1);
        }break;//BLE_GAP_EVT_DISCONNECTED

        case BLE_GAP_EVT_CONNECTED:
        {
            NRF_LOG_PRINTF("[APP]: %s: BLE_GAP_EVT_CONNECTED\r\n", __func__);
            peer_to_be_deleted = PM_PEER_ID_INVALID;
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            // Start Security Request timer.
            err_code = app_timer_start(m_sec_req_timer_id, SECURITY_REQUEST_DELAY, NULL);
            APP_ERROR_CHECK(err_code);
        }break;//BLE_GAP_EVT_CONNECTED

        case BLE_GATTS_EVT_TIMEOUT:
            NRF_LOG_PRINTF("[APP]: %s: BLE_GATTS_EVT_TIMEOUT\r\n", __func__);
            // Disconnect on GATT Server timeout events.
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;//BLE_GATTS_EVT_TIMEOUT

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            NRF_LOG_DEBUG("BLE_GAP_EVT_SEC_PARAMS_REQUEST\r\n");
        break;//BLE_GAP_EVT_SEC_PARAMS_REQUEST

        case BLE_GAP_EVT_PASSKEY_DISPLAY:
        {
            NRF_LOG_PRINTF("[APP]: %s: BLE_GAP_EVT_PASSKEY_DISPLAY\r\n", __func__);
            char passkey[PASSKEY_LENGTH+1];
            memcpy(passkey,p_ble_evt->evt.gap_evt.params.passkey_display.passkey,PASSKEY_LENGTH);
            passkey[PASSKEY_LENGTH] = 0;
            // Don't send delayed Security Request if security procedure is already in progress.
            err_code = app_timer_stop(m_sec_req_timer_id);
            APP_ERROR_CHECK(err_code);

            NRF_LOG_PRINTF("Passkey: %s\r\n",passkey);
        }break;//BLE_GAP_EVT_PASSKEY_DISPLAY

        case BLE_EVT_USER_MEM_REQUEST:
            NRF_LOG_PRINTF("[APP]: %s: BLE_EVT_USER_MEM_REQUEST\r\n", __func__);
            err_code = sd_ble_user_mem_reply(m_conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break;//BLE_EVT_USER_MEM_REQUEST

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            NRF_LOG_PRINTF("[APP]: %s: BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST\r\n", __func__);
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            if(p_ble_evt->evt.gatts_evt.params.authorize_request.type
               != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((p_ble_evt->evt.gatts_evt.params.authorize_request.request.write.op
                     == BLE_GATTS_OP_PREP_WRITE_REQ)
                    || (p_ble_evt->evt.gatts_evt.params.authorize_request.request.write.op
                     == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW)
                    || (p_ble_evt->evt.gatts_evt.params.authorize_request.request.write.op
                     == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (p_ble_evt->evt.gatts_evt.params.authorize_request.type
                        == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(m_conn_handle,&auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        }break;//BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_state_on_ble_evt(p_ble_evt);
    pm_on_ble_evt(p_ble_evt);
    ble_bas_on_ble_evt(&m_bas, p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
    ess_advertising_on_ble_evt(p_ble_evt);
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in] sys_evt  System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    fs_sys_event_handler(sys_evt);
    ess_advertising_on_sys_evt(sys_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
    
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
    
    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);
    
    // Enable BLE stack.
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);    
}


/**@brief Function for the Peer Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Peer Manager.
 */
static void peer_manager_init(bool erase_bonds)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    if (erase_bonds)
    {
        err_code = pm_peers_delete();
        APP_ERROR_CHECK(err_code);
    }

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond              = SEC_PARAM_BOND;
    sec_param.mitm              = SEC_PARAM_MITM;
    sec_param.lesc              = SEC_PARAM_LESC;
    sec_param.keypress          = SEC_PARAM_KEYPRESS;
    sec_param.io_caps           = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob               = SEC_PARAM_OOB;
    sec_param.min_key_size      = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size      = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc     = 1;
    sec_param.kdist_own.id      = 1;
    sec_param.kdist_peer.enc    = 1;
    sec_param.kdist_peer.id     = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanresp;

    m_adv_modes_config.ess_adv_unbonded_fast_interval = APP_ADV_UNBONDED_FAST_INTERVAL;
    m_adv_modes_config.ess_adv_unbonded_fast_timeout = APP_ADV_UNBONDED_FAST_TIMEOUT;
    m_adv_modes_config.ess_adv_unbonded_slow_interval = APP_ADV_UNBONDED_SLOW_INTERVAL;
    m_adv_modes_config.ess_adv_unbonded_slow_timeout = APP_ADV_UNBONDED_SLOW_TIMEOUT;
    m_adv_modes_config.ess_adv_bonded_wl_on_interval = APP_ADV_BONDED_WL_ON_INTERVAL;
    m_adv_modes_config.ess_adv_bonded_wl_on_timeout = APP_ADV_BONDED_WL_ON_TIMEOUT;
    m_adv_modes_config.ess_adv_bonded_wl_off_fast_interval = APP_ADV_BONDED_WL_OFF_FAST_INTERVAL;
    m_adv_modes_config.ess_adv_bonded_wl_off_fast_timeout = APP_ADV_BONDED_WL_OFF_FAST_TIMEOUT;
    m_adv_modes_config.ess_adv_bonded_wl_off_slow_interval = APP_ADV_BONDED_WL_OFF_SLOW_INTERVAL;
    m_adv_modes_config.ess_adv_bonded_wl_off_slow_timeout = APP_ADV_BONDED_WL_OFF_SLOW_TIMEOUT;
    
    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
//    advdata.include_appearance      = true;
    advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;;
    advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = m_adv_uuids;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&scanresp, 0, sizeof(scanresp));
    
    scanresp.include_ble_device_addr = true;
    
    err_code = ess_advertising_init(&advdata, &scanresp, &m_adv_modes_config, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the TWI interface (with transaction manager).
 */
static void twi_init(void)
{
    uint32_t err_code;

    nrf_drv_twi_config_t const config = {
       .scl                = 15,
       .sda                = 13,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_LOW
    };
    
//    APP_TWI_INIT(&m_app_twi, NULL, TWI_MAX_PENDING_TRANSACTIONS, err_code);
    APP_TWI_INIT(&m_app_twi, &config, TWI_MAX_PENDING_TRANSACTIONS, err_code);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling a button event.
 *
 * @param[in]   pin_no         Pin that had an event happen.
 * @param[in]   button_event   APP_BUTTON_PUSH or APP_BUTTON_RELEASE.
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_event)
{
//    uint32_t err_code;

//    if ((button_event == APP_BUTTON_PUSH) && (pin_no == BUTTON_1))
//    {
//        switch (m_button_state)
//        {
//        case BUTTON_STATE_NONE:
//            // Starta timer 10s
//            err_code = app_timer_start(m_button_single_shot_timer_id, BUTTON_LONG_PRESS, NULL);
//            APP_ERROR_CHECK(err_code);
//            m_button_state = BUTTON_STATE_FIRST_PRESS;
//            break;
//        case BUTTON_STATE_FIRST_PRESS:
//            // Do nothing, should not happen
//            break;
//        case BUTTON_STATE_SECOND_PRESS:
//            // Set erase state, actual erase will take place on button release
//            m_button_state = BUTTON_STATE_ERASE;
//            break;
//        case BUTTON_STATE_ERASE:
//            // Do nothing, erase and reset will be done on button release
//            break;
//        }
//    }
//    else if ((button_event == APP_BUTTON_RELEASE) && (pin_no == BUTTON_1))
//    {
//        switch (m_button_state)
//        {
//        case BUTTON_STATE_NONE:
//            // Do nothing, should not happen <- you never know
//            break;
//        case BUTTON_STATE_FIRST_PRESS:
//            // Stop timer
//            err_code = app_timer_stop(m_button_single_shot_timer_id);
//            APP_ERROR_CHECK(err_code);
//            m_button_state = BUTTON_STATE_NONE;
//            break;
//        case BUTTON_STATE_SECOND_PRESS:
//            // Do nothing
//            break;
//        case BUTTON_STATE_ERASE:
//            // Stop blink timer
//            err_code = app_timer_stop(m_blink_timer_id);
//            APP_ERROR_CHECK(err_code);
//            nrf_gpio_pin_clear(LED_1);
//            // Erase all and restart
//            w_conf_storage_erase_all();
//            // TODO: Erase bonding data as well
//            m_flash_mode = FLASH_ERASE_ALL;
//            break;
//        }
//    }
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(void)
{
    uint32_t err_code;
//    bsp_event_t startup_event;

//    uint32_t err_code = bsp_init(BSP_INIT_BUTTONS,
//                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
//                                 bsp_event_handler);
////    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
////                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
////                                 bsp_event_handler);
//    APP_ERROR_CHECK(err_code);

//    err_code = bsp_btn_ble_init(NULL, &startup_event);
//    APP_ERROR_CHECK(err_code);

//    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
    
//    err_code = bsp_button_is_pressed(0, p_erase_bonds);
//    APP_ERROR_CHECK(err_code);
    
    
    static const app_button_cfg_t app_buttons[] =
    {
        {BUTTON1, APP_BUTTON_ACTIVE_LOW, NRF_GPIO_PIN_PULLUP, button_event_handler},
    };
    
    err_code = app_button_init((app_button_cfg_t *)app_buttons, 
                                sizeof(&app_buttons) / sizeof(&app_buttons[0]), 
                                APP_TIMER_TICKS(BUTTON_DEBOUNCE_MS, APP_TIMER_PRESCALER));
    APP_ERROR_CHECK(err_code);
                                
    err_code = app_button_enable();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for checking if the erase bond button press sequence is detected.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void detect_erase_bond_pressed(bool * p_erase_bonds)
{
    uint32_t err_code;
    bool button;
    
    *p_erase_bonds = false;
    
    err_code = app_button_is_pushed(0, &button);
    APP_ERROR_CHECK(err_code);
    uint8_t count = 0;
    while (button) {
        nrf_delay_ms(100);
        count++;
        if (count > 20) {
            *p_erase_bonds = true;
            NRF_LOG_DEBUG("Erasing bonds button press detected!\r\n");
            break;
        }
        err_code = app_button_is_pushed(0, &button);
        APP_ERROR_CHECK(err_code);
    }

    while (button) {
        nrf_gpio_pin_clear(LED_1);
        nrf_delay_ms(200);
        nrf_gpio_pin_set(LED_1);
        nrf_delay_ms(200);
        err_code = app_button_is_pushed(0, &button);
        APP_ERROR_CHECK(err_code);
    }
    
}


//#ifdef DEBUG
static void radio_notification_blink(bool radio_active)
{
    if (radio_active) {
        nrf_gpio_pin_set(LED_1);
    } else {
        nrf_gpio_pin_clear(LED_1);
    }
}
//#endif


static void radio_notification_init()
{
//#ifdef DEBUG    
    uint32_t err_code;
    err_code = ble_radio_notification_init(APP_PRIORITY_LOW, NRF_RADIO_NOTIFICATION_DISTANCE_800US, radio_notification_blink);
    APP_ERROR_CHECK(err_code);
//#endif
}


static void sensors_callback(sensors_evt_t evt, sensors_data_t *data)
{
    switch (evt) {
        case SI7020_TEMP_MEAS_EVT:
            NRF_LOG_PRINTF("[APPL] Temperature measurement, %.2f.\r\n", data->temperature);
//            bluetooth_update_temperature(data->temperature);
            if (m_adv_state == W_ADV_STATE_IDLE) {
                advertising_init();
                advertising_start();
            }
            break;
        case SI7020_HUM_MEAS_EVT:
            NRF_LOG_PRINTF("[APP] T=%d.%02d°C, RH=%d.%02d%%\r\n", (uint32_t)data->temperature, 
                                                        (uint32_t)(100.0 * data->temperature) - 100 * (uint32_t)data->temperature, 
                                                        (uint32_t)data->humidity,
                                                        (uint32_t)(100.0 * data->humidity) - 100 * (uint32_t)data->humidity);
//            bluetooth_update_temperature(data->temperature);
//            bluetooth_update_humidity(data->humidity);
            if (m_adv_state == W_ADV_STATE_IDLE) {
                advertising_init();
                advertising_start();
            }
            break;
    }
}


/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    NRF_LOG("[APP]: Start Advertising\r\n");
    uint32_t err_code = ess_advertising_start();
    APP_ERROR_CHECK(err_code);
    m_adv_state = W_ADV_STATE_ADVERTISING;
}


/**@brief Function for application main entry.
 */
int main(void)
{
    uint32_t err_code;
    bool erase_bonds;

    // Initialize.
    gpio_init();
    // TODO: Measure battery
    nrf_gpio_pin_set(LED_1);
    err_code = NRF_LOG_INIT();
    APP_ERROR_CHECK(err_code);
        
    twi_init();
    sensors_init(&m_app_twi, sensors_callback);

    timers_init();
    buttons_leds_init();
//    erase_bonds = false;
    ble_stack_init();
    detect_erase_bond_pressed(&erase_bonds);
//    bool button;
//    err_code = app_button_is_pushed(0, &button);
//    APP_ERROR_CHECK(err_code);
//    uint8_t count = 0;
//    while (button) {
//        nrf_delay_ms(100);
//        count++;
//        if (count > 20) {
//            erase_bonds = true;
//            NRF_LOG_DEBUG("Erasing bonds!\r\n");
//            break;
//        }
//        err_code = app_button_is_pushed(0, &button);
//        APP_ERROR_CHECK(err_code);
//    }
//    while (button) {
//        nrf_gpio_pin_clear(LED_1);
//        nrf_delay_ms(200);
//        nrf_gpio_pin_set(LED_1);
//        nrf_delay_ms(200);
//        err_code = app_button_is_pushed(0, &button);
//        APP_ERROR_CHECK(err_code);
//    }
//    
    
    peer_manager_init(erase_bonds);
    gap_params_init();
    advertising_init();
    services_init();
    conn_params_init();
//    radio_notification_init();
    nrf_gpio_pin_clear(LED_1);

    // Start execution.
    application_timers_start();
    advertising_start();
//    sensors_measure();
    sensors_timers_init();
    sensors_timers_start();

    // Enter main loop.
    for (;;)
    {
        power_manage();
    }
}

/**
 * @}
 */
