/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/**
 * @brief BLE LED Button Service central and client application main file.
 *
 * This example can be a central for up to 8 peripherals.
 * The peripheral is called ble_app_blinky and can be found in the ble_peripheral
 * folder.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stdarg.h"
#include "nordic_common.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_db_discovery.h"
#include "ble_lbs_c.h"
#include "ble_thingy_uis_c.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "ble_agg_config_service.h"
#include "app_aggregator.h"
#include "app_uart.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#define APP_BLE_CONN_CFG_TAG      1                                     /**< A tag that refers to the BLE stack configuration we set with @ref sd_ble_cfg_set. Default tag is @ref APP_BLE_CONN_CFG_TAG. */
#define APP_BLE_OBSERVER_PRIO     1                                     /**< Application's BLE observer priority. You shouldn't need to modify this value. */

// Peripheral parameters
#define DEVICE_NAME                  "MultiLinkDemo"                    /**< Name of device. Will be included in the advertising data. */
#define AGG_CFG_SERVICE_UUID_TYPE    BLE_UUID_TYPE_VENDOR_BEGIN         /**< UUID type for the Nordic UART Service (vendor specific). */
#define MIN_PERIPHERAL_CON_INT       MSEC_TO_UNITS(20, UNIT_1_25_MS)    /**< Determines minimum connection interval in milliseconds. */
#define MAX_PERIPHERAL_CON_INT       MSEC_TO_UNITS(200, UNIT_1_25_MS)   /**< Determines maximum connection interval in milliseconds. */
#define PERIPHERAL_SLAVE_LATENCY     0                                  /**< Slave latency. */
#define PERIPHERAL_CONN_SUP_TIMEOUT  MSEC_TO_UNITS(4000, UNIT_10_MS)    /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */

#define PERIPHERAL_ADV_INTERVAL                100                      /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define PERIPHERAL_ADV_TIMEOUT_IN_SECONDS      180                      /**< The advertising timeout (in units of seconds). */

#define CENTRAL_SCANNING_LED        BSP_BOARD_LED_0
#define CENTRAL_CONNECTED_LED       BSP_BOARD_LED_1
#define LEDBUTTON_LED               BSP_BOARD_LED_2                     /**< LED to indicate a change of state of the the Button characteristic on the peer. */

#define LEDBUTTON_BUTTON            BSP_BUTTON_0                        /**< Button that will write to the LED characteristic of the peer. */
#define BUTTON_DETECTION_DELAY      APP_TIMER_TICKS(50)                 /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define SCAN_INTERVAL               0x00A0                              /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW                 0x0050                              /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_TIMEOUT                0x0000                              /**< Timout when scanning. 0x0000 disables timeout. */

#define MIN_CONNECTION_INTERVAL     MSEC_TO_UNITS(7.5, UNIT_1_25_MS)    /**< Determines minimum connection interval in milliseconds. */
#define MAX_CONNECTION_INTERVAL     MSEC_TO_UNITS(30, UNIT_1_25_MS)     /**< Determines maximum connection interval in milliseconds. */
#define SLAVE_LATENCY               0                                   /**< Determines slave latency in terms of connection events. */
#define SUPERVISION_TIMEOUT         MSEC_TO_UNITS(4000, UNIT_10_MS)     /**< Determines supervision time-out in units of 10 milliseconds. */

#define UUID16_SIZE                 2                                   /**< Size of a UUID, in bytes. */

#define THINGY_RSSI_CONNECT_LIMIT   -35

NRF_BLE_GATT_DEF(m_gatt);                                               /**< GATT module instance. */

BLE_AGG_CFG_SERVICE_DEF(m_agg_cfg_service);                             /**< BLE NUS service instance. */
BLE_ADVERTISING_DEF(m_advertising);                                     /**< Advertising module instance. */                                                                

BLE_LBS_C_ARRAY_DEF(m_lbs_c, NRF_SDH_BLE_CENTRAL_LINK_COUNT);           /**< LED Button client instances. */
BLE_THINGY_UIS_C_ARRAY_DEF(m_thingy_uis_c, NRF_SDH_BLE_CENTRAL_LINK_COUNT);
BLE_DB_DISCOVERY_ARRAY_DEF(m_db_disc, NRF_SDH_BLE_CENTRAL_LINK_COUNT);  /**< Database discovery module instances. */

static char const m_target_periph_name[] = "NT:";                       /**< Name of the device we try to connect to. This name is searched for in the scan report data*/

static volatile bool m_service_discovery_in_process = false;
static uint16_t   m_per_con_handle       = BLE_CONN_HANDLE_INVALID;
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_AGG_CFG_SERVICE_SERVICE, AGG_CFG_SERVICE_UUID_TYPE}
};

typedef enum {DEVTYPE_NONE, DEVTYPE_BLINKY, DEVTYPE_THINGY} device_type_t;
device_type_t    m_device_type_being_connected_to = DEVTYPE_NONE;
char             m_device_name_being_connected_to[30];

static ret_code_t led_status_send_to_all(uint8_t button_state, uint8_t r, uint8_t g, uint8_t b);
static ret_code_t led_status_send_by_mask(uint8_t button_state, uint8_t r, uint8_t g, uint8_t b, uint32_t mask);
static ret_code_t led_status_on_off_send_by_mask(bool on, uint32_t mask);
static ret_code_t post_connect_message(uint8_t conn_handle);

/**@brief Scan parameters requested for scanning and connection. */
static ble_gap_scan_params_t const m_scan_params =
{
    .active   = 0,
    .interval = SCAN_INTERVAL,
    .window   = SCAN_WINDOW,
    .timeout  = SCAN_TIMEOUT,
    #if (NRF_SD_BLE_API_VERSION <= 2)
        .selective   = 0,
        .p_whitelist = NULL,
    #endif
    #if (NRF_SD_BLE_API_VERSION >= 3)
        .use_whitelist  = 0,
        .adv_dir_report = 0,
    #endif
};

/**@brief Connection parameters requested for connection. */
static ble_gap_conn_params_t const m_connection_param =
{
    (uint16_t)MIN_CONNECTION_INTERVAL,
    (uint16_t)MAX_CONNECTION_INTERVAL,
    (uint16_t)SLAVE_LATENCY,
    (uint16_t)SUPERVISION_TIMEOUT
};


void uart_printf(const char *fmt, ...)
{
#if defined(__GNUC__)
    char buf[120], *p;
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    for (p = buf; *p; ++p)
    app_uart_put(*p);
    va_end(ap);
#elif defined(__CC_ARM)
  printf(fmt, ap);
#endif
 }


/**@brief Function to handle asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing ASSERT call.
 * @param[in] p_file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_PERIPHERAL_CON_INT;
    gap_conn_params.max_conn_interval = MAX_PERIPHERAL_CON_INT;
    gap_conn_params.slave_latency     = PERIPHERAL_SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = PERIPHERAL_CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
    
    ble_opt_t ble_opt;
    ble_opt.gap_opt.preferred_phys.tx_phys = BLE_GAP_PHY_2MBPS | BLE_GAP_PHY_1MBPS | BLE_GAP_PHY_CODED;
    ble_opt.gap_opt.preferred_phys.rx_phys = BLE_GAP_PHY_2MBPS | BLE_GAP_PHY_1MBPS | BLE_GAP_PHY_CODED;    
                                          
    sd_ble_opt_set(BLE_GAP_OPT_PREFERRED_PHYS_SET, &ble_opt);
}

enum {APPCMD_ERROR, APPCMD_SET_LED_ALL, APPCMD_SET_LED_ON_OFF_ALL, 
      APPCMD_POST_CONNECT_MESSAGE, APPCMD_DISCONNECT_PERIPHERALS};


static volatile uint32_t agg_cmd_received = 0;
static uint8_t           agg_cmd[32];

static void agg_cfg_service_data_handler(ble_agg_cfg_service_evt_t * p_evt)
{
    if (p_evt->type == BLE_AGG_CFG_SERVICE_EVT_RX_DATA)
    {
        while(agg_cmd_received != 0);
        //if(agg_cmd_received == 0)
        {
            agg_cmd_received = p_evt->params.rx_data.p_data[0];
            memcpy(agg_cmd, &p_evt->params.rx_data.p_data[1], p_evt->params.rx_data.length);
        }
        //else NRF_LOG_WARNING("AGG CMD OVERFLOW!!\r\n");
    }
}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void)
{
    bsp_board_leds_init();
}


/**
 * @brief Parses advertisement data, providing length and location of the field in case
 *        matching data is found.
 *
 * @param[in]  type       Type of data to be looked for in advertisement data.
 * @param[in]  p_advdata  Advertisement report length and pointer to report.
 * @param[out] p_typedata If data type requested is found in the data report, type data length and
 *                        pointer to data will be populated here.
 *
 * @retval NRF_SUCCESS if the data type is found in the report.
 * @retval NRF_ERROR_NOT_FOUND if the data type could not be found.
 */
static uint32_t adv_report_parse(uint8_t type, uint8_array_t * p_advdata, uint8_array_t * p_typedata)
{
    uint32_t  index = 0;
    uint8_t * p_data;

    p_data = p_advdata->p_data;

    while (index < p_advdata->size)
    {
        uint8_t field_length = p_data[index];
        uint8_t field_type   = p_data[index + 1];

        if (field_type == type)
        {
            p_typedata->p_data = &p_data[index + 2];
            p_typedata->size   = field_length - 1;
            return NRF_SUCCESS;
        }
        index += field_length + 1;
    }
    return NRF_ERROR_NOT_FOUND;
}


/**@brief Function to start scanning. */
static void scan_start(void)
{
    ret_code_t ret;

    (void) sd_ble_gap_scan_stop();

    NRF_LOG_INFO("Start scanning for device name %s.", (uint32_t)m_target_periph_name);
    ret = sd_ble_gap_scan_start(&m_scan_params);
    APP_ERROR_CHECK(ret);

    ret = bsp_indication_set(BSP_INDICATE_SCANNING);
    APP_ERROR_CHECK(ret);
}


/**@brief Handles events coming from the LED Button central module.
 *
 * @param[in] p_lbs_c     The instance of LBS_C that triggered the event.
 * @param[in] p_lbs_c_evt The LBS_C event.
 */
static void lbs_c_evt_handler(ble_lbs_c_t * p_lbs_c, ble_lbs_c_evt_t * p_lbs_c_evt)
{
    switch (p_lbs_c_evt->evt_type)
    {
        case BLE_LBS_C_EVT_DISCOVERY_COMPLETE:
        {
            ret_code_t err_code;

            NRF_LOG_INFO("LED Button service discovered on conn_handle 0x%x\r\n",
                         p_lbs_c_evt->conn_handle);

            // LED Button service discovered. Enable notification of Button.
            err_code = ble_lbs_c_button_notif_enable(p_lbs_c);
            APP_ERROR_CHECK(err_code);
            
            if(ble_conn_state_n_centrals() < NRF_SDH_BLE_CENTRAL_LINK_COUNT)
            {
                scan_start();
            }
            
            m_service_discovery_in_process = false;
        } break; // BLE_LBS_C_EVT_DISCOVERY_COMPLETE

        case BLE_LBS_C_EVT_BUTTON_NOTIFICATION:
        {
            NRF_LOG_INFO("Link 0x%x, Button state changed on peer to 0x%x",
                         p_lbs_c_evt->conn_handle,
                         p_lbs_c_evt->params.button.button_state);

            if (p_lbs_c_evt->params.button.button_state)
            {
                bsp_board_led_on(LEDBUTTON_LED);
            }
            else
            {
                bsp_board_led_off(LEDBUTTON_LED);
            }
            
            // Forward the data to the app aggregator module
            app_aggregator_on_blinky_data(p_lbs_c_evt->conn_handle, p_lbs_c_evt->params.button.button_state);
           
        } break; // BLE_LBS_C_EVT_BUTTON_NOTIFICATION

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Handles events coming from the Thingy UI central module.
 *
 * @param[in] p_thingy_uis_c     The instance of THINGY_UIS_C that triggered the event.
 * @param[in] p_thingy_uis_c_evt The THINGY_UIS_C event.
 */
static void thingy_uis_c_evt_handler(ble_thingy_uis_c_t * p_thingy_uis_c, ble_thingy_uis_c_evt_t * p_thingy_uis_c_evt)
{
    ret_code_t err_code;
    switch (p_thingy_uis_c_evt->evt_type)
    {
        case BLE_LBS_C_EVT_DISCOVERY_COMPLETE:
        {
            NRF_LOG_DEBUG("Thingy UI service discovered on conn_handle 0x%x\r\n", p_thingy_uis_c_evt->conn_handle);
            
            // Thingy UI service discovered. Enable notification of Button.
            err_code = ble_thingy_uis_c_button_notif_enable(p_thingy_uis_c);
            APP_ERROR_CHECK(err_code);
            
            if(ble_conn_state_n_centrals() < NRF_SDH_BLE_CENTRAL_LINK_COUNT)
            {
                scan_start();
            }
            
            m_service_discovery_in_process = false;
        } break; // BLE_LBS_C_EVT_DISCOVERY_COMPLETE

        case BLE_LBS_C_EVT_BUTTON_NOTIFICATION:
        {
            // Forward the data to the app aggregator module
            app_aggregator_on_blinky_data(p_thingy_uis_c_evt->conn_handle, p_thingy_uis_c_evt->params.button.button_state);
        } break; // BLE_LBS_C_EVT_BUTTON_NOTIFICATION

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for handling the advertising report BLE event.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_adv_report(ble_evt_t const * p_ble_evt)
{
    uint32_t      err_code;
    uint8_array_t adv_data;
    uint8_array_t dev_name;
    uint8_array_t service_uuid;

    if (m_device_type_being_connected_to == DEVTYPE_NONE)
    {

        // For readibility.
        ble_gap_evt_t  const * p_gap_evt  = &p_ble_evt->evt.gap_evt;
        ble_gap_addr_t const * peer_addr  = &p_gap_evt->params.adv_report.peer_addr;

        // Prepare advertisement report for parsing.
        adv_data.p_data = (uint8_t *)p_gap_evt->params.adv_report.data;
        adv_data.size   = p_gap_evt->params.adv_report.dlen;

        // Search for advertising names.
        bool found_name = false;
        err_code = adv_report_parse(BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME,
                                    &adv_data,
                                    &dev_name);
        if (err_code != NRF_SUCCESS)
        {
            // Look for the short local name if it was not found as complete.
            err_code = adv_report_parse(BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME, &adv_data, &dev_name);
            if (err_code != NRF_SUCCESS)
            {
                // If we can't parse the data, then exit.
                //return;
            }
            else
            {
                found_name = true;
            }
        }
        else
        {
            found_name = true;
        }

        if (found_name)
        {
            // Check if the device name matches the BLINKY name filter
            if (strlen(m_target_periph_name) != 0)
            {
                if (memcmp(m_target_periph_name, dev_name.p_data, strlen(m_target_periph_name)) == 0)
                {
                    // Copy the name to a static variable, to pass it on to the smart phone later
                    if(dev_name.size > strlen(m_target_periph_name))
                    {
                        memcpy(m_device_name_being_connected_to, 
                               dev_name.p_data + strlen(m_target_periph_name), 
                               dev_name.size - strlen(m_target_periph_name));
                        m_device_name_being_connected_to[dev_name.size - strlen(m_target_periph_name)] = 0;
                    }
                    m_device_type_being_connected_to = DEVTYPE_BLINKY;
                }
            }
        }
        
        // Look for Thingy UUID
        // Filter on RSSI to avoid connecting to everything in the room
        const uint8_t thingy_service_uuid[] = {0x42, 0x00, 0x74, 0xA9, 0xFF, 0x52, 0x10, 0x9B, 0x33, 0x49, 0x35, 0x9B, 0x00, 0x01, 0x68, 0xEF};
        if(p_gap_evt->params.adv_report.rssi > THINGY_RSSI_CONNECT_LIMIT)
        {
            err_code = adv_report_parse(BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_MORE_AVAILABLE, &adv_data, &service_uuid);
            if (err_code == NRF_SUCCESS)
            {
                if (memcmp(service_uuid.p_data, thingy_service_uuid, 16) == 0)
                {
                    if(found_name)
                    {
                        memcpy(m_device_name_being_connected_to, 
                               dev_name.p_data, dev_name.size);
                        m_device_name_being_connected_to[dev_name.size] = 0;
                    }
                    m_device_type_being_connected_to = DEVTYPE_THINGY;
                }
            }
        }

        if (m_device_type_being_connected_to != DEVTYPE_NONE)
        {
            // Initiate connection.
            err_code = sd_ble_gap_connect(peer_addr, &m_scan_params, &m_connection_param, APP_BLE_CONN_CFG_TAG);
            if (err_code != NRF_SUCCESS)
            {
                NRF_LOG_ERROR("Connection Request Failed, reason %d", err_code);
            }
        }
    }
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    //uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            break;
        case BLE_ADV_EVT_IDLE:
            break;
        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    // For readability.
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        // Upon connection, check which peripheral has connected, initiate DB
        // discovery, update LEDs status and resume scanning if necessary.
        case BLE_GAP_EVT_CONNECTED:
        {
            // Handle central connections
            if(p_gap_evt->params.connected.role == BLE_GAP_ROLE_CENTRAL)
            {
                NRF_LOG_INFO("Connection 0x%x established, starting DB discovery.",
                             p_gap_evt->conn_handle);

                //APP_ERROR_CHECK_BOOL(p_gap_evt->conn_handle < NRF_SDH_BLE_CENTRAL_LINK_COUNT);
                
                switch(m_device_type_being_connected_to)
                {
                    case DEVTYPE_BLINKY:
                        err_code = ble_lbs_c_handles_assign(&m_lbs_c[p_gap_evt->conn_handle],
                                                            p_gap_evt->conn_handle, NULL);
                        APP_ERROR_CHECK(err_code);
                        break;
                    
                    case DEVTYPE_THINGY:
                        err_code = ble_thingy_uis_c_handles_assign(&m_thingy_uis_c[p_gap_evt->conn_handle],
                                                                   p_gap_evt->conn_handle, NULL);
                        APP_ERROR_CHECK(err_code);
                        break;
                    
                    default:
                        break;
                }

                m_service_discovery_in_process = true;
                memset(&m_db_disc[p_gap_evt->conn_handle], 0x00, sizeof(ble_db_discovery_t));
                err_code = ble_db_discovery_start(&m_db_disc[p_gap_evt->conn_handle],
                                                  p_gap_evt->conn_handle);
                if (err_code != NRF_ERROR_BUSY)
                {
                    APP_ERROR_CHECK(err_code);
                }
                
                err_code = sd_ble_gap_rssi_start(p_gap_evt->conn_handle, 5, 4);
                APP_ERROR_CHECK(err_code);

                // Notify the aggregator service
                app_aggregator_on_central_connect(p_gap_evt, m_device_type_being_connected_to, 
                                                  m_device_name_being_connected_to);
                
                // Update LEDs status, and check if we should be looking for more
                // peripherals to connect to.
                bsp_board_led_on(CENTRAL_CONNECTED_LED);
                if (ble_conn_state_n_centrals() == NRF_SDH_BLE_CENTRAL_LINK_COUNT)
                {
                    bsp_board_led_off(CENTRAL_SCANNING_LED);
                }
                else
                {
                    // Resume scanning.
                    //bsp_board_led_on(CENTRAL_SCANNING_LED);
                    //scan_start();
                }
                
                m_device_type_being_connected_to = DEVTYPE_NONE;
            }
            // Handle links as a peripheral here
            else
            {
                m_per_con_handle = p_gap_evt->conn_handle;
                NRF_LOG_INFO("Peripheral connection 0x%x established.", m_per_con_handle);               
            }
        } break; // BLE_GAP_EVT_CONNECTED

        // Upon disconnection, reset the connection handle of the peer which disconnected, update
        // the LEDs status and start scanning again.
        case BLE_GAP_EVT_DISCONNECTED:
        {
            NRF_LOG_INFO("GAP_EVT_DISCONNECT: %i", p_gap_evt->conn_handle);
            // Handle central disconnections
            if(p_gap_evt->conn_handle != m_per_con_handle)
            {
                NRF_LOG_INFO("LBS central link 0x%x disconnected (reason: 0x%x)",
                             p_gap_evt->conn_handle,
                             p_gap_evt->params.disconnected.reason);

                if (ble_conn_state_n_centrals() == 0)
                {
                    err_code = app_button_disable();
                    APP_ERROR_CHECK(err_code);

                    // Turn off connection indication LED
                    bsp_board_led_off(CENTRAL_CONNECTED_LED);
                }

                // Start scanning if we are not currently in the service discovery state
                if(!m_service_discovery_in_process)
                {
                    scan_start();
                }

                // Turn on LED for indicating scanning
                bsp_board_led_on(CENTRAL_SCANNING_LED);
                
                // Notify aggregator service
                app_aggregator_on_central_disconnect(p_gap_evt);
            }
            // Handle peripheral disconnect
            else
            {
                NRF_LOG_INFO("Peripheral connection disconnected (reason: 0x%x)", p_gap_evt->params.disconnected.reason);
                m_per_con_handle = BLE_CONN_HANDLE_INVALID;
                
                // Start advertising
                err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
                APP_ERROR_CHECK(err_code);
            }

        } break;

        case BLE_GAP_EVT_ADV_REPORT:
            on_adv_report(p_ble_evt);
            break;

        case BLE_GAP_EVT_TIMEOUT:
        {
            // We have not specified a timeout for scanning, so only connection attemps can timeout.
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_DEBUG("Connection request timed out.");
            }
            else if(p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISING)
            {
                uart_printf("Adv restart\r\n");
                // Start advertising
                err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
                APP_ERROR_CHECK(err_code);
            }
                
        } break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
        {
            
            NRF_LOG_INFO("BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST. CH%i, MaxMin CI %i, %i", p_gap_evt->conn_handle, 
                            p_gap_evt->params.conn_param_update_request.conn_params.max_conn_interval, 
                            p_gap_evt->params.conn_param_update_request.conn_params.min_conn_interval);
            // Accept parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                        &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
        } break;

#if defined(S132)
        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;
#endif

        case BLE_GAP_EVT_PHY_UPDATE:
        {
            app_aggregator_phy_update(p_ble_evt->evt.gap_evt.conn_handle,
                                      p_ble_evt->evt.gap_evt.params.phy_update.tx_phy,
                                      p_ble_evt->evt.gap_evt.params.phy_update.rx_phy);
            /*ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);*/
        } break;        
        
        case BLE_GATTC_EVT_TIMEOUT:
        {
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTS_EVT_TIMEOUT:
        {
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
        } break;
        
        case BLE_GAP_EVT_RSSI_CHANGED:
            app_aggregator_rssi_changed(p_ble_evt->evt.gap_evt.conn_handle, p_ble_evt->evt.gap_evt.params.rssi_changed.rssi);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_agg_cfg_service_init_t agg_cfg_service_init;

    memset(&agg_cfg_service_init, 0, sizeof(agg_cfg_service_init));

    agg_cfg_service_init.data_handler = agg_cfg_service_data_handler;

    err_code = ble_agg_cfg_service_init(&m_agg_cfg_service, &agg_cfg_service_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief LED Button collector initialization. */
static void lbs_c_init(void)
{
    ret_code_t       err_code;
    ble_lbs_c_init_t lbs_c_init_obj;

    lbs_c_init_obj.evt_handler = lbs_c_evt_handler;

    for (uint32_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
    {
        err_code = ble_lbs_c_init(&m_lbs_c[i], &lbs_c_init_obj);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief LED Button collector initialization. */
static void thingy_uis_c_init(void)
{
    ret_code_t       err_code;
    ble_thingy_uis_c_init_t thingy_uis_c_init_obj;

    thingy_uis_c_init_obj.evt_handler = thingy_uis_c_evt_handler;

    for (uint32_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
    {
        err_code = ble_thingy_uis_c_init(&m_thingy_uis_c[i], &thingy_uis_c_init_obj);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupts.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = PERIPHERAL_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = PERIPHERAL_ADV_TIMEOUT_IN_SECONDS;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for writing to the LED characteristic of all connected clients.
 *
 * @details Based on if the button is pressed or released, this function writes a high or low
 *          LED status to the server.
 *
 * @param[in] button_action The button action (press/release).
 *            Determines if the LEDs of the servers will be ON or OFF.
 *
 * @return If successful NRF_SUCCESS is returned. Otherwise, the error code from @ref ble_lbs_led_status_send.
 */
static ret_code_t led_status_send_to_all(uint8_t button_action, uint8_t r, uint8_t g, uint8_t b)
{
    ret_code_t err_code;

    for (uint32_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
    {
        err_code = ble_lbs_led_status_send(&m_lbs_c[i], button_action);

        if(err_code != NRF_SUCCESS)
        {
            err_code = ble_thingy_uis_led_set_constant(&m_thingy_uis_c[i], button_action ? r : 0, button_action ? g : 0, button_action ? b : 0);
            if (err_code != NRF_SUCCESS &&
                err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
                err_code != NRF_ERROR_INVALID_STATE)
            {
                return err_code;
            }
        }
    }
    return NRF_SUCCESS;
}

static ret_code_t led_status_send_by_mask(uint8_t button_action, uint8_t r, uint8_t g, uint8_t b, uint32_t mask)
{
    ret_code_t err_code;

    for (uint32_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
    {
        if((mask & (1 << i)) != 0)
        {
            err_code = ble_lbs_led_status_send(&m_lbs_c[i], button_action);
            if(err_code != NRF_SUCCESS)
            {
                err_code = ble_thingy_uis_led_set_constant(&m_thingy_uis_c[i], button_action ? r : 0, button_action ? g : 0, button_action ? b : 0);
                if (err_code != NRF_SUCCESS &&
                    err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
                    err_code != NRF_ERROR_INVALID_STATE)
                {
                    return err_code;
                }
            }
        }
    }
    return NRF_SUCCESS;    
}

ret_code_t led_status_on_off_send_by_mask(bool on, uint32_t mask)
{
    ret_code_t err_code;

    app_aggregator_on_led_update(on, mask);
    
    for (uint32_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
    {
        if((mask & (1 << i)) != 0)
        {
            err_code = ble_lbs_led_status_send(&m_lbs_c[i], on);
            if(err_code != NRF_SUCCESS)
            {
                err_code = ble_thingy_uis_led_set_on_off(&m_thingy_uis_c[i], on);
                if (err_code != NRF_SUCCESS &&
                    err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
                    err_code != NRF_ERROR_INVALID_STATE)
                {
                    return err_code;
                }
            }
        }
    }
    return NRF_SUCCESS;      
}

ret_code_t post_connect_message(uint8_t conn_handle)
{
    ret_code_t err_code = NRF_SUCCESS;
    for(int i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
    {
        if(m_lbs_c[i].conn_handle == conn_handle)
        {
            err_code = ble_thingy_uis_led_set_on_off(&m_thingy_uis_c[i], true);
        }
        
        if(m_thingy_uis_c[i].conn_handle == conn_handle)
        {
            err_code = ble_thingy_uis_led_set_constant(&m_thingy_uis_c[i], 255, 255, 255);           
        }
    }
    return err_code;
}

ret_code_t disconnect_all_peripherals()
{
    ret_code_t err_code;
    for(int i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
    {
        if(m_lbs_c[i].conn_handle != BLE_CONN_HANDLE_INVALID)
        {
            err_code = sd_ble_gap_disconnect(m_lbs_c[i].conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if(err_code != NRF_SUCCESS)
            {
                return err_code;
            }
        }
        
        if(m_thingy_uis_c[i].conn_handle != BLE_CONN_HANDLE_INVALID)
        {
            err_code = sd_ble_gap_disconnect(m_lbs_c[i].conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if(err_code != NRF_SUCCESS)
            {
                return err_code;
            }
        }
    }    
    return NRF_SUCCESS;
}

/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press/release).
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
    ret_code_t err_code;

    switch (pin_no)
    {
        case LEDBUTTON_BUTTON:
            /*err_code = led_status_send_to_all(button_action, 255, 255, 255);
            if (err_code == NRF_SUCCESS)
            {
                NRF_LOG_INFO("LBS write LED state %d", button_action);
            }*/
            if(agg_cmd_received == 0)
            {
                agg_cmd_received = APPCMD_SET_LED_ON_OFF_ALL;
                agg_cmd[0] = button_action;
                agg_cmd[1] = agg_cmd[2] = agg_cmd[3] = agg_cmd[4] = 0xFF;
            }
            break;

        default:
            APP_ERROR_HANDLER(pin_no);
            break;
    }
}


/**@brief Function for initializing the button handler module.
 */
static void buttons_init(void)
{
    ret_code_t err_code;

   // The array must be static because a pointer to it will be saved in the button handler module.
    static app_button_cfg_t buttons[] =
    {
        {LEDBUTTON_BUTTON, false, BUTTON_PULL, button_event_handler}
    };

    err_code = app_button_init(buttons, ARRAY_SIZE(buttons), BUTTON_DETECTION_DELAY);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling database discovery events.
 *
 * @details This function is callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    NRF_LOG_DEBUG("call to ble_lbs_on_db_disc_evt for instance %d and link 0x%x!",
                  p_evt->conn_handle,
                  p_evt->conn_handle);

    ble_lbs_on_db_disc_evt(&m_lbs_c[p_evt->conn_handle], p_evt);
    ble_thingy_uis_on_db_disc_evt(&m_thingy_uis_c[p_evt->conn_handle], p_evt);
}


/** @brief Database discovery initialization.
 */
static void db_discovery_init(void)
{
    ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
    APP_ERROR_CHECK(err_code);
}


/** @brief Function to sleep until a BLE event is received by the application.
 */
static void power_manage(void)
{
    ret_code_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


/** @brief Function for initializing the log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}


static void uart_init()
{
    uint32_t err_code;
    const app_uart_comm_params_t comm_params =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud460800
    };

    APP_UART_FIFO_INIT(&comm_params,
                       16,
                       1024,
                       uart_error_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);

    APP_ERROR_CHECK(err_code);
}


/** @brief Function for initializing the timer.
 */
static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
    
    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, 64);
    APP_ERROR_CHECK(err_code);
}


static void process_app_commands()
{               
    if(agg_cmd_received != 0)
    {          
        uint32_t mask;
        switch(agg_cmd_received)
        {
            case APPCMD_SET_LED_ALL:
                for(int i = 2; i >= 0; i--)
                {
                    mask = mask << 8 | agg_cmd[4 + i];
                }
                led_status_send_by_mask(agg_cmd[0], agg_cmd[1], agg_cmd[2], agg_cmd[3], mask);
                break;
                
            case APPCMD_SET_LED_ON_OFF_ALL:
                for(int i = 2; i >= 0; i--)
                {
                    mask = mask << 8 | agg_cmd[1 + i];
                }
                led_status_on_off_send_by_mask(agg_cmd[0], mask);
                break;
                
            case APPCMD_POST_CONNECT_MESSAGE:
                post_connect_message(agg_cmd[0]);
                break;
            
            case APPCMD_DISCONNECT_PERIPHERALS:
                disconnect_all_peripherals();
                break;
            
            default:
                break;
        }
        agg_cmd_received = 0;
    }
}


int main(void)
{
    uint32_t err_code;
    
    log_init();
    timer_init();
    uart_init();
    leds_init();
    buttons_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    app_aggregator_init(&m_agg_cfg_service);
    db_discovery_init();
    lbs_c_init();
    thingy_uis_c_init();
    ble_conn_state_init();
    advertising_init();

    NRF_LOG_INFO("Multilink example started");
    uart_printf("Multilink example started\r\n");

    // Start scanning for peripherals and initiate connection to devices which  advertise.
    scan_start();

    // Start advertising
    err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
    
    // Turn on the LED to signal scanning.
    bsp_board_led_on(CENTRAL_SCANNING_LED);
        
    err_code = app_button_enable();
    APP_ERROR_CHECK(err_code);
    
    for (;;)
    {
        if (!NRF_LOG_PROCESS())
        {
            if(m_per_con_handle != BLE_CONN_HANDLE_INVALID)
            {
                while(app_aggregator_flush_ble_commands());
            }

            process_app_commands();

            device_list_print();
            
            power_manage();
        }
    }
}
