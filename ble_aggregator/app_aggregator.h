#ifndef __APP_AGGREGATOR_H
#define __APP_AGGREGATOR_H

#include <stdint.h>
#include <stdbool.h>
#include "ble_gap.h"
#include "ble_agg_config_service.h"

#define MAX_NUMBER_OF_LINKS 20

#define MAX_ADV_NAME_LENGTH 15

enum {APP_AGGR_COL_IND_RED, APP_AGGR_COL_IND_GREEN, APP_AGGR_COL_IND_BLUE};

typedef struct
{
    uint16_t conn_handle;
    uint8_t  device_type;
    uint8_t  button_state;
    uint8_t  led_state;
    uint8_t  led_color[3];
    uint8_t  rf_phy;
    int8_t   last_rssi;
    uint8_t  adv_name[MAX_ADV_NAME_LENGTH + 1];
}link_info_t;

typedef struct
{
    uint32_t dev_type;
    const char *dev_name;
    uint32_t phy;
}connected_device_info_t;

void app_aggregator_init(ble_agg_cfg_service_t *agg_cfg_service);

void app_aggregator_on_central_connect(const ble_gap_evt_t *ble_gap_evt, connected_device_info_t *con_dev_info);

void app_aggregator_on_central_disconnect(const ble_gap_evt_t *ble_gap_evt);

void app_aggregator_on_blinky_data(uint16_t conn_handle, uint8_t button_state);

void app_aggregator_on_led_update(uint8_t led_state, uint32_t conn_handle_mask);

void app_aggregator_on_led_color_set(uint8_t r, uint8_t g, uint8_t b, uint32_t conn_handle_mask);

void app_aggregator_phy_update(uint16_t conn_handle, uint8_t tx_phy, uint8_t rx_phy);

void app_aggregator_rssi_changed(uint16_t conn_handle, int8_t rssi);

bool app_aggregator_flush_ble_commands(void);

void app_aggregator_clear_buffer(void);

void app_aggregator_update_link_status(void);

void device_list_print(void);

void app_aggregator_all_led_update(uint8_t button_state);

void uart_printf(const char *fmt, ...);

#endif
