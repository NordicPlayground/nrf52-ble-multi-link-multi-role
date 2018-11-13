
#ifndef WS2812_GFX_GLUE_LAYER_H__
#define WS2812_GFX_GLUE_LAYER_H__

#include "nrf_lcd.h"
#include <nrfx.h>


#define GFX_LED_DRV_MATRIX  \
{   \
    .lcd_display          = drv_ws2812_gfx_glue_display,  \
    .lcd_display_invert   = drv_ws2812_gfx_glue_display_invert,  \
    .lcd_init             = drv_ws2812_gfx_glue_init,  \
    .lcd_pixel_draw       = drv_ws2812_gfx_glue_pixel_draw,  \
    .lcd_rect_draw        = drv_ws2812_gfx_glue_rect_draw,  \
    .lcd_rotation_set     = drv_ws2812_gfx_glue_rotation_set,  \
    .lcd_uninit           = drv_ws2812_gfx_glue_uninit,  \
}

// Function for initializing the LCD controller.
ret_code_t drv_ws2812_gfx_glue_init(void);

// Function for uninitializing the LCD controller.
void drv_ws2812_gfx_glue_uninit(void);

// Function for drawing a single pixel.
void drv_ws2812_gfx_glue_pixel_draw(uint16_t x, uint16_t y, uint32_t color);

// Function for drawing a filled rectangle.
void drv_ws2812_gfx_glue_rect_draw(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint32_t color);

// Function for displaying data from an internal frame buffer. 
void drv_ws2812_gfx_glue_display(void);

// Function for rotating the screen.
void drv_ws2812_gfx_glue_rotation_set(nrf_lcd_rotation_t rotation);

// Function for setting inversion of colors on the screen.
void drv_ws2812_gfx_glue_display_invert(bool invert);

#endif // WS2812_GFX_GLUE_LAYER_H__

