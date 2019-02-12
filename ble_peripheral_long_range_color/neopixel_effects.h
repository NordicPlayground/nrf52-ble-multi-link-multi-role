#ifndef __NEOPIXEL_EFFECTS_H
#define __NEOPIXEL_EFFECTS_H

#include <stdint.h>
#include <stdbool.h>
#include "nrf_gfx_ext.h"

typedef enum {NPEFFECT_MODE_FADE_COLOR} neopixel_effect_mode;

typedef struct
{
    uint32_t effect_mode;
}neopixel_effect_config_t;

uint32_t neopixel_effects_init(nrf_lcd_t * led_matrix);

void neopixel_update_graphics(void);

uint32_t neopixel_effect_start(neopixel_effect_config_t * effect);

#endif
