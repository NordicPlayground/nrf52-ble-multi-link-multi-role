#include "app_aggregator.h"
#include "nrf_log.h"
#include <string.h>

#define BLE_AGG_CMD_BUFFER_SIZE 512
#define BLE_AGG_CMD_MAX_LENGTH  128

enum {APP_AGG_ERROR_CONN_HANDLE_CONFLICT = 1, APP_AGG_ERROR_LINK_INFO_LIST_FULL, APP_AGG_ERROR_CONN_HANDLE_NOT_FOUND};
enum TX_COMMANDS {AGG_BLE_LINK_CONNECTED = 1, AGG_BLE_LINK_DISCONNECTED};
enum {APP_AGG_DEVICE_TYPE_UNKNOWN, APP_AGG_DEVICE_TYPE_BLINKY, APP_AGG_DEVICE_TYPE_THINGY, APP_AGG_DEVICE_TYPE_END};
static char *device_type_string_list[] = {"Unknown", "Blinky", "Thingy"};

static uint8_t tx_command_payload[20];
static uint16_t tx_command_payload_length;
static ble_agg_cfg_service_t *m_ble_service;

static uint8_t ble_cmd_buf[BLE_AGG_CMD_BUFFER_SIZE] = {0};
static uint32_t ble_cmd_buf_in_ptr = 0, ble_cmd_buf_out_ptr = 0;

static link_info_t m_link_info_list[MAX_NUMBER_OF_LINKS];
static uint32_t m_error_flags = 0;

static volatile bool m_schedule_device_list_print = true;

static uint16_t device_list_search(uint16_t conn_handle);
static uint16_t device_list_find_available(void);
static void device_connected(uint16_t conn_handle, uint16_t dev_type);
static void device_disconnected(uint16_t conn_handle);

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
    for(int i = 0; i < MAX_NUMBER_OF_LINKS; i++)
    {
        m_link_info_list[i].conn_handle = BLE_CONN_HANDLE_INVALID;
    }
    
}

void app_aggregator_on_central_connect(const ble_gap_evt_t *ble_gap_evt, uint32_t dev_type)
{
    uint16_t conn_handle = ble_gap_evt->conn_handle;
    
    // Update local device list
    device_connected(conn_handle, dev_type);
    
    // Send info to central device (if connected)
    tx_command_payload[0] = AGG_BLE_LINK_CONNECTED;
    tx_command_payload[1] = ble_gap_evt->conn_handle >> 8;
    tx_command_payload[2] = ble_gap_evt->conn_handle & 0xFF;
    tx_command_payload_length = 3;
    cmd_buffer_put(tx_command_payload, tx_command_payload_length);
}

void app_aggregator_on_central_disconnect(const ble_gap_evt_t *ble_gap_evt)
{
    uint16_t conn_handle = ble_gap_evt->conn_handle;
    
    // Update local device list
    device_disconnected(conn_handle);
       
    // Send info to central device (if connected)
    tx_command_payload[0] = AGG_BLE_LINK_DISCONNECTED;
    tx_command_payload[1] = ble_gap_evt->conn_handle >> 8;
    tx_command_payload[2] = ble_gap_evt->conn_handle & 0xFF;
    tx_command_payload_length = 3;
    cmd_buffer_put(tx_command_payload, tx_command_payload_length);
}

void app_aggregator_on_blinky_data(uint16_t conn_handle, uint8_t button_state)
{
    uint16_t device_index = device_list_search(conn_handle);
    if(device_index != BLE_CONN_HANDLE_INVALID)
    {
        m_link_info_list[device_index].button_state = button_state;
        m_schedule_device_list_print = true;
    }
}

uint8_t   *data_ptr;
uint8_t   tmp_buffer[BLE_AGG_CMD_MAX_LENGTH];
uint16_t  length;
bool      reuse_packet = false;

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

static uint16_t device_list_search(uint16_t conn_handle)
{
    for(int i = 0; i < MAX_NUMBER_OF_LINKS; i++)
    {
        if(m_link_info_list[i].conn_handle == conn_handle) return i;
    }
    return BLE_CONN_HANDLE_INVALID;
}

static uint16_t device_list_find_available()
{
    return device_list_search(BLE_CONN_HANDLE_INVALID);
}

static void device_connected(uint16_t conn_handle, uint16_t device_type)
{
    if(device_list_search(conn_handle) == BLE_CONN_HANDLE_INVALID)
    {
        uint16_t new_device_index;
        new_device_index = device_list_find_available();
        if(new_device_index != 0xFFFF)
        {
            m_link_info_list[new_device_index].conn_handle = conn_handle;
            m_link_info_list[new_device_index].device_type = device_type; 
            m_link_info_list[new_device_index].button_state = 0; 
            m_link_info_list[new_device_index].led_state = 0; 
            m_schedule_device_list_print = true;
        }
        else m_error_flags |= 1 << APP_AGG_ERROR_LINK_INFO_LIST_FULL;
    }
    else m_error_flags |= 1 << APP_AGG_ERROR_CONN_HANDLE_CONFLICT;
}

static void device_disconnected(uint16_t conn_handle)
{
    uint16_t device_index;
    if((device_index = device_list_search(conn_handle)) != BLE_CONN_HANDLE_INVALID)
    {
        m_link_info_list[device_index].conn_handle = BLE_CONN_HANDLE_INVALID;
        m_schedule_device_list_print = true;
    }
    else m_error_flags |= 1 << APP_AGG_ERROR_CONN_HANDLE_NOT_FOUND;       
}

void device_list_print()
{
    if(m_schedule_device_list_print)
    {
        m_schedule_device_list_print = false;
        printf("\r\n-------------------- Device list overview --------------------\r\n");
        printf("Con No.\t\tType\t\tBtn state\tLED state\t  \r\n");
        for(int i = 0; i < MAX_NUMBER_OF_LINKS; i++)
        {
            if(m_link_info_list[i].conn_handle != BLE_CONN_HANDLE_INVALID)
            {
                if(m_link_info_list[i].device_type < APP_AGG_DEVICE_TYPE_END)
                {
                    printf("%i\t\t%s\t\t%i\t\t%i \r\n", m_link_info_list[i].conn_handle, device_type_string_list[m_link_info_list[i].device_type],
                                                        m_link_info_list[i].button_state, m_link_info_list[i].led_state);
                }
                else
                {
                     printf("%i\t\tInvalid device!\t%i\t\t%i \r\n", m_link_info_list[i].conn_handle, 
                                                         m_link_info_list[i].button_state, m_link_info_list[i].led_state);                   
                }
            }
        }
        printf("\r\n");
    }
}
