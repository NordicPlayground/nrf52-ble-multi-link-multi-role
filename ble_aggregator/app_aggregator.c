#include "app_aggregator.h"
#include "nrf_log.h"
#include <string.h>

#define BLE_AGG_CMD_BUFFER_SIZE 512
#define BLE_AGG_CMD_MAX_LENGTH  128

static uint8_t tx_command_payload[20];
static uint16_t tx_command_payload_length;
static ble_agg_cfg_service_t *m_ble_service;

static uint8_t ble_cmd_buf[BLE_AGG_CMD_BUFFER_SIZE] = {0};
static uint32_t ble_cmd_buf_in_ptr = 0, ble_cmd_buf_out_ptr = 0;

enum TX_COMMANDS {AGG_BLE_LINK_CONNECTED = 1, AGG_BLE_LINK_DISCONNECTED};

static bool cmd_buffer_put(uint8_t *data, uint16_t length)
{
    if(length > BLE_AGG_CMD_MAX_LENGTH) return false;
    
    if(ble_cmd_buf_out_ptr <= ble_cmd_buf_in_ptr)
    {
        if(length < (BLE_AGG_CMD_BUFFER_SIZE - ble_cmd_buf_in_ptr - 1))
        {
            ble_cmd_buf[ble_cmd_buf_in_ptr++] = length;
            memcpy(&ble_cmd_buf[ble_cmd_buf_in_ptr], data, length);
            ble_cmd_buf_in_ptr += length;
            ble_cmd_buf[ble_cmd_buf_in_ptr] = 0;
        }
        else if(ble_cmd_buf_out_ptr > (length + 1))
        {          
            ble_cmd_buf[0] = length;
            memcpy(&ble_cmd_buf[1], data, length);
            ble_cmd_buf[length + 1] = 0;
            ble_cmd_buf_in_ptr = length + 1;
        }
        else return false;
    }
    else
    {
        if((length + 1) < (ble_cmd_buf_out_ptr - ble_cmd_buf_in_ptr))
        {
            ble_cmd_buf[ble_cmd_buf_in_ptr++] = length;
            memcpy(&ble_cmd_buf[ble_cmd_buf_in_ptr], data, length);
            ble_cmd_buf_in_ptr += length;
            ble_cmd_buf[ble_cmd_buf_in_ptr] = 0;            
        }
        else return false;
    }
    //NRF_LOG_INFO("BPUT: In: %i, Out: %i\r\n", ble_cmd_buf_in_ptr, ble_cmd_buf_out_ptr);
    return true;
}

static bool cmd_buffer_peek(uint8_t **data_ptr, uint16_t *length_ptr)
{
    if(ble_cmd_buf[ble_cmd_buf_out_ptr] != 0)
    {
        *data_ptr = &ble_cmd_buf[ble_cmd_buf_out_ptr + 1];
        *length_ptr = ble_cmd_buf[ble_cmd_buf_out_ptr];
        return true;
    }
    return false;    
}

static bool cmd_buffer_get(uint8_t **data_ptr, uint16_t *length_ptr)
{
    if(ble_cmd_buf[ble_cmd_buf_out_ptr] != 0)
    {
        *data_ptr = &ble_cmd_buf[ble_cmd_buf_out_ptr + 1];
        *length_ptr = ble_cmd_buf[ble_cmd_buf_out_ptr];
        ble_cmd_buf_out_ptr += (*length_ptr + 1);
        if(ble_cmd_buf_in_ptr < ble_cmd_buf_out_ptr && ble_cmd_buf[ble_cmd_buf_out_ptr] == 0)
        {
            ble_cmd_buf_out_ptr = 0;
        }
        //NRF_LOG_INFO("BGET: In: %i, Out: %i\r\n", ble_cmd_buf_in_ptr, ble_cmd_buf_out_ptr);
        return true;
    }
    return false;
}

void app_aggregator_init(ble_agg_cfg_service_t *agg_cfg_service)
{
    m_ble_service = agg_cfg_service;
}

void app_aggregator_on_central_connect(const ble_gap_evt_t *ble_gap_evt)
{
    tx_command_payload[0] = AGG_BLE_LINK_CONNECTED;
    tx_command_payload[1] = ble_gap_evt->conn_handle >> 8;
    tx_command_payload[2] = ble_gap_evt->conn_handle & 0xFF;
    tx_command_payload_length = 3;
    cmd_buffer_put(tx_command_payload, tx_command_payload_length);
}

void app_aggregator_on_central_disconnect(const ble_gap_evt_t *ble_gap_evt)
{
    tx_command_payload[0] = AGG_BLE_LINK_DISCONNECTED;
    tx_command_payload[1] = ble_gap_evt->conn_handle >> 8;
    tx_command_payload[2] = ble_gap_evt->conn_handle & 0xFF;
    tx_command_payload_length = 3;
    cmd_buffer_put(tx_command_payload, tx_command_payload_length);
}

uint8_t     *data_ptr;
uint8_t     tmp_buffer[BLE_AGG_CMD_MAX_LENGTH];
uint16_t    length;
bool reuse_packet = false;

bool app_aggregator_flush_ble_commands(void)
{

    uint32_t    err_code;
    if(!reuse_packet)
    {
        if(cmd_buffer_get(&data_ptr, &length))
        {
            //NRF_LOG_INFO("PCKBUF: \r\n");
            //NRF_LOG_HEXDUMP_INFO(data_ptr, length);
            err_code = ble_agg_cfg_service_string_send(m_ble_service, data_ptr, &length);
            if(err_code != NRF_SUCCESS)
            {
                memcpy(tmp_buffer, data_ptr, length);
                data_ptr = tmp_buffer;
                reuse_packet = true;
                return false;
            }
            return true;
        }
    }
    else
    {
        err_code = ble_agg_cfg_service_string_send(m_ble_service, data_ptr, &length);
        if(err_code == NRF_SUCCESS)
        {
            reuse_packet = false;
            return true;
        }   

    }
    return false;
}

