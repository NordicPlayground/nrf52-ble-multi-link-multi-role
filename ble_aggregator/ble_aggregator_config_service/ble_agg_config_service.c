/**
 * Copyright (c) 2012 - 2017, Nordic Semiconductor ASA
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
#include "sdk_common.h"

#include "ble.h"
#include "ble_agg_config_service.h"
#include "ble_srv_common.h"


#define BLE_UUID_AGG_CFG_SERVICE_RX_CHARACTERISTIC 0x0002                      /**< The UUID of the RX Characteristic. */
#define BLE_UUID_AGG_CFG_SERVICE_TX_CHARACTERISTIC 0x0003                      /**< The UUID of the TX Characteristic. */

#define BLE_AGG_CFG_SERVICE_MAX_RX_CHAR_LEN        BLE_AGG_CFG_SERVICE_MAX_DATA_LEN        /**< Maximum length of the RX Characteristic (in bytes). */
#define BLE_AGG_CFG_SERVICE_MAX_TX_CHAR_LEN        BLE_AGG_CFG_SERVICE_MAX_DATA_LEN        /**< Maximum length of the TX Characteristic (in bytes). */

#define AGG_CFG_SERVICE_BASE_UUID                  {{0xbc,0x45,0x4c,0x23,0xdd,0xd7,0x40,0x44,0x82,0xb6,0x68,0x13,0x8f,0xc7,0x52,0x3e}} /**< Used vendor specific UUID. */

/**@brief Function for handling the @ref BLE_GAP_EVT_CONNECTED event from the SoftDevice.
 *
 * @param[in] p_agg_cfg_service     Nordic UART Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_connect(ble_agg_cfg_service_t * p_agg_cfg_service, ble_evt_t const * p_ble_evt)
{
    if(p_ble_evt->evt.gap_evt.params.connected.role == BLE_GAP_ROLE_PERIPH)
    {
        p_agg_cfg_service->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
    }
}


/**@brief Function for handling the @ref BLE_GAP_EVT_DISCONNECTED event from the SoftDevice.
 *
 * @param[in] p_agg_cfg_service     Nordic UART Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_disconnect(ble_agg_cfg_service_t * p_agg_cfg_service, ble_evt_t const * p_ble_evt)
{
    if(p_ble_evt->evt.gap_evt.params.connected.role == BLE_GAP_ROLE_PERIPH)
    {
        UNUSED_PARAMETER(p_ble_evt);
        p_agg_cfg_service->conn_handle = BLE_CONN_HANDLE_INVALID;
    }
}


/**@brief Function for handling the @ref BLE_GATTS_EVT_WRITE event from the SoftDevice.
 *
 * @param[in] p_agg_cfg_service     Nordic UART Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_write(ble_agg_cfg_service_t * p_agg_cfg_service, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    ble_agg_cfg_service_evt_t evt;
    evt.p_agg_cfg_service = p_agg_cfg_service;
    if (   (p_evt_write->handle == p_agg_cfg_service->tx_handles.cccd_handle)
        && (p_evt_write->len == 2))
    {
        if (ble_srv_is_notification_enabled(p_evt_write->data))
        {
            p_agg_cfg_service->is_notification_enabled = true;
            evt.type = BLE_AGG_CFG_SERVICE_EVT_COMM_STARTED;
        }
        else
        {
            p_agg_cfg_service->is_notification_enabled = false;
            evt.type = BLE_AGG_CFG_SERVICE_EVT_COMM_STOPPED;
        }
        p_agg_cfg_service->data_handler(&evt);
    }
    else if (   (p_evt_write->handle == p_agg_cfg_service->rx_handles.value_handle)
             && (p_agg_cfg_service->data_handler != NULL))
    {
        evt.params.rx_data.p_data = p_evt_write->data;
        evt.params.rx_data.length = p_evt_write->len;
        evt.type = BLE_AGG_CFG_SERVICE_EVT_RX_DATA;
        p_agg_cfg_service->data_handler(&evt);
    }
    else
    {
        // Do Nothing. This event is not relevant for this service.
    }
}


/**@brief Function for adding TX characteristic.
 *
 * @param[in] p_agg_cfg_service       Nordic UART Service structure.
 * @param[in] p_agg_cfg_service_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t tx_char_add(ble_agg_cfg_service_t * p_agg_cfg_service, ble_agg_cfg_service_init_t const * p_agg_cfg_service_init)
{
    /**@snippet [Adding proprietary characteristic to the SoftDevice] */
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_agg_cfg_service->uuid_type;
    ble_uuid.uuid = BLE_UUID_AGG_CFG_SERVICE_TX_CHARACTERISTIC;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = BLE_AGG_CFG_SERVICE_MAX_TX_CHAR_LEN;

    return sd_ble_gatts_characteristic_add(p_agg_cfg_service->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_agg_cfg_service->tx_handles);
    /**@snippet [Adding proprietary characteristic to the SoftDevice] */
}


/**@brief Function for adding RX characteristic.
 *
 * @param[in] p_agg_cfg_service       Nordic UART Service structure.
 * @param[in] p_agg_cfg_service_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t rx_char_add(ble_agg_cfg_service_t * p_agg_cfg_service, const ble_agg_cfg_service_init_t * p_agg_cfg_service_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.write         = 1;
    char_md.char_props.write_wo_resp = 1;
    char_md.p_char_user_desc         = NULL;
    char_md.p_char_pf                = NULL;
    char_md.p_user_desc_md           = NULL;
    char_md.p_cccd_md                = NULL;
    char_md.p_sccd_md                = NULL;

    ble_uuid.type = p_agg_cfg_service->uuid_type;
    ble_uuid.uuid = BLE_UUID_AGG_CFG_SERVICE_RX_CHARACTERISTIC;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 1;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = BLE_AGG_CFG_SERVICE_MAX_RX_CHAR_LEN;

    return sd_ble_gatts_characteristic_add(p_agg_cfg_service->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_agg_cfg_service->rx_handles);
}


void ble_agg_cfg_service_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    if ((p_context == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    ble_agg_cfg_service_t * p_agg_cfg_service = (ble_agg_cfg_service_t *)p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_agg_cfg_service, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_agg_cfg_service, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_agg_cfg_service, p_ble_evt);
            break;

        case BLE_GATTS_EVT_HVN_TX_COMPLETE:
        {
            //notify with empty data that some tx was completed.
            ble_agg_cfg_service_evt_t evt = {
                    .type = BLE_AGG_CFG_SERVICE_EVT_TX_RDY,
                    .p_agg_cfg_service = p_agg_cfg_service
            };
            p_agg_cfg_service->data_handler(&evt);
            break;
        }
        default:
            // No implementation needed.
            break;
    }
}


uint32_t ble_agg_cfg_service_init(ble_agg_cfg_service_t * p_agg_cfg_service, ble_agg_cfg_service_init_t const * p_agg_cfg_service_init)
{
    uint32_t      err_code;
    ble_uuid_t    ble_uuid;
    ble_uuid128_t agg_cfg_service_base_uuid = AGG_CFG_SERVICE_BASE_UUID;

    VERIFY_PARAM_NOT_NULL(p_agg_cfg_service);
    VERIFY_PARAM_NOT_NULL(p_agg_cfg_service_init);

    // Initialize the service structure.
    p_agg_cfg_service->conn_handle             = BLE_CONN_HANDLE_INVALID;
    p_agg_cfg_service->data_handler            = p_agg_cfg_service_init->data_handler;
    p_agg_cfg_service->is_notification_enabled = false;

    /**@snippet [Adding proprietary Service to the SoftDevice] */
    // Add a custom base UUID.
    err_code = sd_ble_uuid_vs_add(&agg_cfg_service_base_uuid, &p_agg_cfg_service->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_agg_cfg_service->uuid_type;
    ble_uuid.uuid = BLE_UUID_AGG_CFG_SERVICE_SERVICE;

    // Add the service.
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_agg_cfg_service->service_handle);
    /**@snippet [Adding proprietary Service to the SoftDevice] */
    VERIFY_SUCCESS(err_code);

    // Add the RX Characteristic.
    err_code = rx_char_add(p_agg_cfg_service, p_agg_cfg_service_init);
    VERIFY_SUCCESS(err_code);

    // Add the TX Characteristic.
    err_code = tx_char_add(p_agg_cfg_service, p_agg_cfg_service_init);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}


uint32_t ble_agg_cfg_service_string_send(ble_agg_cfg_service_t * p_agg_cfg_service, uint8_t * p_string, uint16_t * p_length)
{
    ble_gatts_hvx_params_t hvx_params;

    VERIFY_PARAM_NOT_NULL(p_agg_cfg_service);

    if ((p_agg_cfg_service->conn_handle == BLE_CONN_HANDLE_INVALID) || (!p_agg_cfg_service->is_notification_enabled))
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (*p_length > BLE_AGG_CFG_SERVICE_MAX_DATA_LEN)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = p_agg_cfg_service->tx_handles.value_handle;
    hvx_params.p_data = p_string;
    hvx_params.p_len  = p_length;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;

    return sd_ble_gatts_hvx(p_agg_cfg_service->conn_handle, &hvx_params);
}
