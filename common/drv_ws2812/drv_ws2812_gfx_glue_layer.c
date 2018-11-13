
#include <stdbool.h>
#include <stdint.h>
#include "nrf_assert.h"
#include "nrf_gpio.h"
#include "sdk_config.h"
#include "sdk_errors.h"
#include "drv_ws2812.h"
#include "drv_ws2812_gfx_glue_layer.h"

// Function for initializing the LCD controller.
ret_code_t drv_ws2812_gfx_glue_init(void)
{
    return drv_ws2812_init();
}

// Function for uninitializing the LCD controller.
void drv_ws2812_gfx_glue_uninit(void)
{
    // No implementation
}

// Function for drawing a single pixel.
void drv_ws2812_gfx_glue_pixel_draw(uint16_t x, uint16_t y, uint32_t color)
{
    drv_ws2812_pixel_draw(x, y, color);
}

// Function for drawing a filled rectangle.
void drv_ws2812_gfx_glue_rect_draw(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint32_t color)
{
    drv_ws2812_rectangle_draw(x, y, width, height, color);
}

// Function for displaying data from an internal frame buffer. 
void drv_ws2812_gfx_glue_display(void)
{
    drv_ws2812_display();
}

// Function for rotating the screen.
void drv_ws2812_gfx_glue_rotation_set(nrf_lcd_rotation_t rotation)
{
    // No implementation
}

// Function for setting inversion of colors on the screen.
void drv_ws2812_gfx_glue_display_invert(bool invert)
{
    // No implementation
}


