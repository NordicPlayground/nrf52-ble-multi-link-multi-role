#ifndef __APP_AGGREGATOR_H
#define __APP_AGGREGATOR_H

#include <stdint.h>
#include <stdbool.h>
#include "ble_gap.h"
#include "ble_agg_config_service.h"

#define MAX_NUMBER_OF_LINKS 20

#define MAX_ADV_NAME_LENGTH 16

typedef struct
{
    uint16_t conn_handle;
    uint8_t  device_type;
    uint8_t  button_state;
    uint8_t  led_state;
}link_info_t;

void app_aggregator_init(ble_agg_cfg_service_t *agg_cfg_service);

void app_aggregator_on_central_connect(const ble_gap_evt_t *ble_gap_evt, uint32_t dev_type, 
                                       const char *dev_name);

void app_aggregator_on_central_disconnect(const ble_gap_evt_t *ble_gap_evt);

void app_aggregator_on_blinky_data(uint16_t conn_handle, uint8_t button_state);

bool app_aggregator_flush_ble_commands(void);

void device_list_print(void);

#endif
