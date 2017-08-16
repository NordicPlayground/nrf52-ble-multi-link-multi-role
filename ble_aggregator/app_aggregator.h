#ifndef __APP_AGGREGATOR_H
#define __APP_AGGREGATOR_H

#include <stdint.h>
#include <stdbool.h>
#include "ble_gap.h"
#include "ble_agg_config_service.h"

void app_aggregator_init(ble_agg_cfg_service_t *agg_cfg_service);

void app_aggregator_on_central_connect(const ble_gap_evt_t *ble_gap_evt);

void app_aggregator_on_central_disconnect(const ble_gap_evt_t *ble_gap_evt);

bool app_aggregator_flush_ble_commands(void);

#endif
