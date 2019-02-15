#ifndef __NEOPIXEL_EFFECTS_H
#define __NEOPIXEL_EFFECTS_H

#include <stdint.h>
#include <stdbool.h>
#include "nrf_gfx_ext.h"

#define LED_UPDATE_INTERVAL_MS  10

typedef enum {NPEFFECT_MODE_IDLE, 
              NPEFFECT_MODE_FADE_TO_COLOR, 
              NPEFFECT_MODE_COLOR_AND_RSSI, 
              NPEFFECT_MODE_FLASHY, 
              NPEFFECT_MODE_COLOR_BY_RSSI,
              NPEFFECT_MODE_RADIAL_TWIST
             } neopixel_effect_mode;

typedef struct
{
    uint32_t effect_mode;
    uint32_t new_color;
    uint32_t effect_duration;
    int8_t   new_rssi;
}neopixel_effect_config_t;

uint32_t neopixel_effects_init(nrf_lcd_t * led_matrix);

void neopixel_update_graphics(void);

uint32_t neopixel_effect_start(neopixel_effect_config_t * effect);

#endif
