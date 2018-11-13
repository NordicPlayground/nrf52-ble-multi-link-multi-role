

#ifndef DRV_WS2812_H__
#define DRV_WS2812_H__

#include <nrfx.h>

#define WS2812_PIN  3

#define LED_MATRIX_WIDTH       10
#define LED_MATRIX_HEIGHT      1


typedef struct
{
    uint8_t b;
    uint8_t g;
    uint8_t r;
}rgb_t;

typedef union
{
    uint32_t u;
    rgb_t    s;
}color_t;

uint32_t drv_ws2812_init(void);

/**
 * @brief This function must be called to draw the actual buffer onto the LED matrix
 */
uint32_t drv_ws2812_display(void);

/**
 * @brief Draws a single pixel in the buffer. 
 *          drv_ws2812_display() must be called to update the LED matrix
 */
uint32_t drv_ws2812_pixel_draw(uint16_t x, uint16_t y, uint32_t color);

/**
 * @brief Draws a rectangle in the buffer. 
 *          drv_ws2812_display() must be called to update the LED matrix
 *          BE AWARE OF BUG IN GFX library: https://devzone.nordicsemi.com/f/nordic-q-a/37284/bug-gfx-line-drawing-mishandles-start-end-values
 */
uint32_t drv_ws2812_rectangle_draw(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint32_t color);


#endif // DRV_WS2812_H__

