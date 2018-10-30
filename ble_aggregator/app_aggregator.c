#include "app_aggregator.h"
#include "nrf_log.h"
#include <string.h>

#define BLE_AGG_CMD_BUFFER_SIZE 2048
#define BLE_AGG_CMD_MAX_LENGTH  64

enum {APP_AGG_ERROR_CONN_HANDLE_CONFLICT = 1, APP_AGG_ERROR_LINK_INFO_LIST_FULL, APP_AGG_ERROR_CONN_HANDLE_NOT_FOUND};
enum TX_COMMANDS {AGG_BLE_LINK_CONNECTED = 1, AGG_BLE_LINK_DISCONNECTED, AGG_BLE_LINK_DATA_UPDATE, AGG_BLE_LED_BUTTON_PRESSED};
enum {APP_AGG_DEVICE_TYPE_UNKNOWN, APP_AGG_DEVICE_TYPE_BLINKY, APP_AGG_DEVICE_TYPE_THINGY, APP_AGG_DEVICE_TYPE_END};
//static char *device_type_string_list[] = {"Unknown", "Blinky", "Thingy"};
static char    *m_phy_name_string_list[] = {"NONE", "1Mbps", "2Mbps", "INVALID", "Coded"};
static char     m_device_name_header_string[MAX_ADV_NAME_LENGTH + 1];

static uint8_t tx_command_payload[BLE_AGG_CMD_MAX_LENGTH];
static uint16_t tx_command_payload_length;
static ble_agg_cfg_service_t *m_ble_service;

static uint8_t ble_cmd_buf[BLE_AGG_CMD_BUFFER_SIZE] = {0};
static uint32_t ble_cmd_buf_in_ptr = 0, ble_cmd_buf_out_ptr = 0;

static link_info_t m_link_info_list[MAX_NUMBER_OF_LINKS];
static uint32_t m_error_flags = 0;

static volatile bool m_schedule_device_list_print = true;

static uint16_t device_list_search(uint16_t conn_handle);
static uint16_t device_list_find_available(void);
static void device_connected(uint16_t conn_handle, connected_device_info_t *con_dev_info);
static void device_disconnected(uint16_t conn_handle);

/*static bool cmd_buffer_put(uint8_t *data, uint16_t length)
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
}*/

static bool cmd_buffer_put(uint8_t *data, uint16_t length)
{
    if(length > BLE_AGG_CMD_MAX_LENGTH) return false;
    
    uint32_t in_ptr_new_pos = (ble_cmd_buf_in_ptr + BLE_AGG_CMD_MAX_LENGTH) % BLE_AGG_CMD_BUFFER_SIZE;
    
    if(in_ptr_new_pos == ble_cmd_buf_out_ptr)
    {
        // Buffer full, exit
        return false;
    }
    
    ble_cmd_buf[ble_cmd_buf_in_ptr] = length;
    memcpy(&ble_cmd_buf[ble_cmd_buf_in_ptr + 1], data, length);
    ble_cmd_buf_in_ptr = in_ptr_new_pos;

    return true;   
}

static bool cmd_buffer_get(uint8_t ** data_ptr, uint16_t * length_ptr)
{
    uint32_t out_ptr_new_pos = (ble_cmd_buf_out_ptr + BLE_AGG_CMD_MAX_LENGTH) % BLE_AGG_CMD_BUFFER_SIZE;
    
    if(ble_cmd_buf_out_ptr == ble_cmd_buf_in_ptr)
    {
        // Buffer empty, return false
        return false;
    }

    *data_ptr = &ble_cmd_buf[ble_cmd_buf_out_ptr + 1];
    *length_ptr = ble_cmd_buf[ble_cmd_buf_out_ptr];
    ble_cmd_buf_out_ptr = out_ptr_new_pos;

    //NRF_LOG_INFO("BGET: In: %i, Out: %i\r\n", ble_cmd_buf_in_ptr, ble_cmd_buf_out_ptr);
    return true;
}

static void cmd_buffer_flush()
{
    ble_cmd_buf_in_ptr = ble_cmd_buf_out_ptr = 0;
}

/*static bool cmd_buffer_peek(uint8_t **data_ptr, uint16_t *length_ptr)
{
    if(ble_cmd_buf[ble_cmd_buf_out_ptr] != 0)
    {
        *data_ptr = &ble_cmd_buf[ble_cmd_buf_out_ptr + 1];
        *length_ptr = ble_cmd_buf[ble_cmd_buf_out_ptr];
        return true;
    }
    return false;    
}*/

/*static bool cmd_buffer_get(uint8_t **data_ptr, uint16_t *length_ptr)
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
}*/

void app_aggregator_init(ble_agg_cfg_service_t *agg_cfg_service)
{
    m_ble_service = agg_cfg_service;
    for(int i = 0; i < MAX_NUMBER_OF_LINKS; i++)
    {
        m_link_info_list[i].conn_handle = BLE_CONN_HANDLE_INVALID;
    }
    strcpy(m_device_name_header_string, "Name");
    for(int i = 4; i < MAX_ADV_NAME_LENGTH; i++)
    {
        m_device_name_header_string[i] = ' ';
    }
    m_device_name_header_string[MAX_ADV_NAME_LENGTH] = 0;
}

void app_aggregator_on_central_connect(const ble_gap_evt_t *ble_gap_evt, connected_device_info_t *con_dev_info)
{
    uint16_t conn_handle = ble_gap_evt->conn_handle;
    
    // Update local device list
    device_connected(conn_handle, con_dev_info);
    
    // Send info to central device (if connected)
    tx_command_payload[0] = AGG_BLE_LINK_CONNECTED;
    tx_command_payload[1] = ble_gap_evt->conn_handle >> 8;
    tx_command_payload[2] = ble_gap_evt->conn_handle & 0xFF;
    tx_command_payload[3] = con_dev_info->dev_type & 0xFF;
    tx_command_payload[4] = 0; //Button state
    tx_command_payload[5] = 1; // LED state;
    tx_command_payload[6] = 0xFF; // Color R
    tx_command_payload[7] = 0xFF; // Color G
    tx_command_payload[8] = 0xFF; // Color B
    tx_command_payload[9] = 0; // RSSI
    tx_command_payload[10] = con_dev_info->phy; // PHY
    if(strlen(con_dev_info->dev_name) <= MAX_ADV_NAME_LENGTH)
    {
        memcpy(&tx_command_payload[11], con_dev_info->dev_name, strlen(con_dev_info->dev_name));
        tx_command_payload_length = 11 + strlen(con_dev_info->dev_name);
    }
    else 
    {
        tx_command_payload_length = 11;
    }
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

void app_aggregator_data_update(uint16_t conn_handle, uint8_t *p_data, uint32_t length)
{
    if(length <= 255)
    {
        tx_command_payload[0] = AGG_BLE_LINK_DATA_UPDATE;
        tx_command_payload[1] = conn_handle >> 8;
        tx_command_payload[2] = conn_handle & 0xFF;
        tx_command_payload[3] = length & 0xFF;
        memcpy(&tx_command_payload[4], p_data, length);
        tx_command_payload_length = length + 4;
        cmd_buffer_put(tx_command_payload, tx_command_payload_length);
    }
}

void app_aggregator_data_update_by_index(uint16_t device_index)
{
    tx_command_payload[0] = AGG_BLE_LINK_DATA_UPDATE;
    tx_command_payload[1] = m_link_info_list[device_index].conn_handle >> 8;
    tx_command_payload[2] = m_link_info_list[device_index].conn_handle & 0xFF;
    tx_command_payload[3] = 3;
    tx_command_payload[4] = m_link_info_list[device_index].button_state;
    tx_command_payload[5] = m_link_info_list[device_index].rf_phy;
    tx_command_payload[6] = m_link_info_list[device_index].last_rssi;
    tx_command_payload_length = 7;
    cmd_buffer_put(tx_command_payload, tx_command_payload_length);    
}

void app_aggregator_all_led_update(uint8_t button_state)
{
    tx_command_payload[0] = AGG_BLE_LED_BUTTON_PRESSED;
    tx_command_payload[1] = button_state;
    tx_command_payload_length = 2;
    cmd_buffer_put(tx_command_payload, tx_command_payload_length);  
}

void app_aggregator_on_blinky_data(uint16_t conn_handle, uint8_t button_state)
{
    uint16_t device_index = device_list_search(conn_handle);
    if(device_index != BLE_CONN_HANDLE_INVALID)
    {
        m_link_info_list[device_index].button_state = button_state;
        app_aggregator_data_update_by_index(device_index);
        //app_aggregator_data_update(conn_handle, &button_state, 1);
        m_schedule_device_list_print = true;
    }
}

void app_aggregator_on_led_update(uint8_t led_state, uint32_t conn_handle_mask)
{
    for(int i = 0; i < MAX_NUMBER_OF_LINKS; i++)
    {
        if(m_link_info_list[i].conn_handle < 32 && ((conn_handle_mask & (1 << m_link_info_list[i].conn_handle)) != 0))
        {
            m_link_info_list[i].led_state = led_state;
            m_schedule_device_list_print = true;
        }
    }
}

void app_aggregator_on_led_color_set(uint8_t r, uint8_t g, uint8_t b, uint32_t conn_handle_mask)
{
    for(int i = 0; i < MAX_NUMBER_OF_LINKS; i++)
    {
        if(m_link_info_list[i].conn_handle < 32 && ((conn_handle_mask & (1 << m_link_info_list[i].conn_handle)) != 0))
        {
            NRF_LOG_DEBUG("Device %i color update: %i, %i, %i", m_link_info_list[i].conn_handle, r, g, b);
            m_link_info_list[i].led_color[APP_AGGR_COL_IND_RED]   = r;
            m_link_info_list[i].led_color[APP_AGGR_COL_IND_GREEN] = g;
            m_link_info_list[i].led_color[APP_AGGR_COL_IND_BLUE]  = b;
        }
    }
}

void app_aggregator_rssi_changed(uint16_t conn_handle, int8_t rssi)
{
    uint16_t device_index = device_list_search(conn_handle);
    if(device_index != BLE_CONN_HANDLE_INVALID)
    {
        m_link_info_list[device_index].last_rssi = rssi;
    }
}

void app_aggregator_phy_update(uint16_t conn_handle, uint8_t tx_phy, uint8_t rx_phy)
{
    uint16_t device_index = device_list_search(conn_handle);
    if(device_index != BLE_CONN_HANDLE_INVALID)
    {
        m_link_info_list[device_index].rf_phy = tx_phy;
        app_aggregator_data_update_by_index(device_index);
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

void app_aggregator_clear_buffer(void)
{
    cmd_buffer_flush();
}

void app_aggregator_update_link_status(void)
{
    NRF_LOG_INFO("Clear buffer update link status");
    
    for(int i = 0; i < MAX_NUMBER_OF_LINKS; i++)
    {
        if(m_link_info_list[i].conn_handle != BLE_CONN_HANDLE_INVALID)
        {
            // Prepare new connection packet for each link in the list
            tx_command_payload[0] = AGG_BLE_LINK_CONNECTED;
            tx_command_payload[1] = m_link_info_list[i].conn_handle >> 8;
            tx_command_payload[2] = m_link_info_list[i].conn_handle & 0xFF;
            tx_command_payload[3] = m_link_info_list[i].device_type;
            tx_command_payload[4] = m_link_info_list[i].button_state;
            tx_command_payload[5] = m_link_info_list[i].led_state;
            tx_command_payload[6] = m_link_info_list[i].led_color[APP_AGGR_COL_IND_RED];
            tx_command_payload[7] = m_link_info_list[i].led_color[APP_AGGR_COL_IND_GREEN];
            tx_command_payload[8] = m_link_info_list[i].led_color[APP_AGGR_COL_IND_BLUE];
            tx_command_payload[9] = m_link_info_list[i].last_rssi;
            tx_command_payload[10] = m_link_info_list[i].rf_phy;
            if(strlen((char *)m_link_info_list[i].adv_name) <= MAX_ADV_NAME_LENGTH)
            {
                memcpy(&tx_command_payload[11], m_link_info_list[i].adv_name, strlen((char *)m_link_info_list[i].adv_name));
                tx_command_payload_length = 11 + strlen((char *)m_link_info_list[i].adv_name);
            }
            else 
            {
                tx_command_payload_length = 11;
            }
            cmd_buffer_put(tx_command_payload, tx_command_payload_length);
            NRF_LOG_HEXDUMP_INFO(tx_command_payload, tx_command_payload_length);
        }
    }

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

static void device_connected(uint16_t conn_handle, connected_device_info_t *con_dev_info)
{
    if(device_list_search(conn_handle) == BLE_CONN_HANDLE_INVALID)
    {
        uint16_t new_device_index;
        new_device_index = device_list_find_available();
        if(new_device_index != 0xFFFF)
        {
            uint32_t device_name_length;
            m_link_info_list[new_device_index].conn_handle = conn_handle;
            m_link_info_list[new_device_index].device_type = con_dev_info->dev_type; 
            m_link_info_list[new_device_index].button_state = 0; 
            m_link_info_list[new_device_index].led_state = 1; 
            m_link_info_list[new_device_index].led_color[0] = 0xFF;
            m_link_info_list[new_device_index].led_color[1] = 0xFF;
            m_link_info_list[new_device_index].led_color[2] = 0xFF;
            device_name_length = (strlen(con_dev_info->dev_name) > MAX_ADV_NAME_LENGTH) ? MAX_ADV_NAME_LENGTH : strlen(con_dev_info->dev_name);
            memcpy(m_link_info_list[new_device_index].adv_name, con_dev_info->dev_name, device_name_length);
            for(int i = device_name_length; i < MAX_ADV_NAME_LENGTH; i++) m_link_info_list[new_device_index].adv_name[i] = ' ';
            m_link_info_list[new_device_index].adv_name[MAX_ADV_NAME_LENGTH - 1] = 0;
            m_link_info_list[new_device_index].rf_phy = con_dev_info->phy;
            m_link_info_list[new_device_index].last_rssi = 0;
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
        uint32_t device_count = 0;
        m_schedule_device_list_print = false;
        for(int i = 0; i < MAX_NUMBER_OF_LINKS; i++)
        {
            if(m_link_info_list[i].conn_handle != BLE_CONN_HANDLE_INVALID)
            {
                device_count++;
            }
        }
        uart_printf("\r\n------ Device list overview (%i device%s------\r\n\n", device_count, (device_count != 1) ? "s total) " : " total) -");
        uart_printf("ID   %sBtn LED Phy   RSSI\r\n", m_device_name_header_string);
        for(int i = 0; i < MAX_NUMBER_OF_LINKS; i++)
        {
            if(m_link_info_list[i].conn_handle != BLE_CONN_HANDLE_INVALID)
            {
                if(m_link_info_list[i].device_type < APP_AGG_DEVICE_TYPE_END)
                {
                    if(m_link_info_list[i].conn_handle < 10)
                    {
                        uart_printf(" %i   ", m_link_info_list[i].conn_handle);
                    }
                    else
                    {
                        uart_printf("%i   ", m_link_info_list[i].conn_handle);
                    }
                    uart_printf("%s ",       m_link_info_list[i].adv_name);
                    uart_printf("%i   ",     m_link_info_list[i].button_state);
                    uart_printf("%i   ",     m_link_info_list[i].led_state);
                    uart_printf("%s ",       m_link_info_list[i].rf_phy <= 4 ? m_phy_name_string_list[m_link_info_list[i].rf_phy] : "ERR!");
                    uart_printf("%i\r\n", (int)m_link_info_list[i].last_rssi);
                }
            }
        }
        uart_printf("\r\n");
    }
}
