
#include <nrfx.h>
#include <string.h>
#include <nrfx_pwm.h>
#include <hal/nrf_gpio.h>
#include "drv_ws2812.h"

#define WS2812_T1H                  14 | 0x8000
#define WS2812_T0H                  6 | 0x8000

#if defined(NEOPIXEL_RING)
#define LED_MATRIX_TOTAL_BYTE_WIDTH	LED_MATRIX_WIDTH * LED_MATRIX_HEIGHT * 4
#define LED_MATRIX_TOTAL_BIT_WIDTH	LED_MATRIX_TOTAL_BYTE_WIDTH * 8
#else
#define LED_MATRIX_TOTAL_BYTE_WIDTH	LED_MATRIX_WIDTH * LED_MATRIX_HEIGHT * 3
#define LED_MATRIX_TOTAL_BIT_WIDTH	LED_MATRIX_TOTAL_BYTE_WIDTH * 8
#endif


static rgb_color_t led_matrix_buffer[LED_MATRIX_WIDTH][LED_MATRIX_HEIGHT];

static nrfx_pwm_t m_pwm0 = NRFX_PWM_INSTANCE(0);
static nrf_pwm_values_individual_t pwm_duty_cycle_values[LED_MATRIX_TOTAL_BIT_WIDTH];
volatile bool pwm_sequencue_finished = true;

void pwm_handler(nrfx_pwm_evt_type_t event_type)
{
    switch(event_type)
    {
	case NRFX_PWM_EVT_FINISHED:
	    pwm_sequencue_finished = true;
	    break;
	default:
	    break;
    }
}

static nrf_pwm_sequence_t pwm_sequence =
{
    .values.p_individual = pwm_duty_cycle_values,
    .length          = (sizeof(pwm_duty_cycle_values) / sizeof(uint16_t)),
    .repeats         = 0,
    .end_delay       = 0
};

static uint32_t pwm_init(void)
{
    nrfx_pwm_config_t pwm_config = NRFX_PWM_DEFAULT_CONFIG;
    pwm_config.output_pins[0] = NRFX_PWM_PIN_NOT_USED; 
    pwm_config.output_pins[1] = WS2812_PIN; 
    pwm_config.output_pins[2] = NRFX_PWM_PIN_NOT_USED;
    pwm_config.output_pins[3] = NRFX_PWM_PIN_NOT_USED;
    pwm_config.load_mode    = NRF_PWM_LOAD_INDIVIDUAL;
    // WS2812 protocol requires a 800 kHz PWM frequency. PWM Top value = 20 and Base Clock = 16 MHz achieves this
    pwm_config.top_value    = 20; 
    pwm_config.base_clock   = NRF_PWM_CLK_16MHz;
    
    return nrfx_pwm_init(&m_pwm0, &pwm_config, pwm_handler);
}



static void convert_rgb_to_pwm_sequence(void)
{
    uint8_t * ptr = (uint8_t *)led_matrix_buffer;
    uint32_t i = 0;
    for(int led = 0; led < LED_MATRIX_TOTAL_BYTE_WIDTH; led++)
    {
        for(int bit = 7; bit >= 0; bit--)
        {
            uint8_t b = (*ptr >> bit) & 0x01;
            uint16_t pwm = 0;
            if(b == 1)
            {
                pwm = WS2812_T1H;
            }
            else
            {
                pwm = WS2812_T0H;
            }
            pwm_duty_cycle_values[i++].channel_1 = pwm;
        }
        ptr++;
    }
}

uint32_t drv_ws2812_init(void)
{   
    volatile uint32_t size = sizeof(led_matrix_buffer);
    memset(led_matrix_buffer, 0x00, sizeof(led_matrix_buffer));   
    return pwm_init();
}

uint32_t drv_ws2812_display(void)
{
    if(!pwm_sequencue_finished) 
    {
        return NRF_ERROR_BUSY;
    }
    convert_rgb_to_pwm_sequence();
    pwm_sequencue_finished = false;
    uint32_t err_code = nrfx_pwm_simple_playback(&m_pwm0, &pwm_sequence, 1, NRFX_PWM_FLAG_STOP);
    return err_code;
}

uint32_t drv_ws2812_pixel_draw(uint16_t x, uint16_t y, uint32_t color)
{
    uint32_t err_code = NRF_SUCCESS;
    if(x > LED_MATRIX_WIDTH - 1)
    {
        err_code = NRF_ERROR_INVALID_PARAM;
    }
    if(y > LED_MATRIX_HEIGHT - 1)
    {
	err_code = NRF_ERROR_INVALID_PARAM;
    }
 
#if defined(NEOPIXEL_RING)
    led_matrix_buffer[x][y].w = (color & 0xFF000000) >> 24;
#endif
    led_matrix_buffer[x][y].r = (color & 0x00FF0000) >> 16;
    led_matrix_buffer[x][y].g = (color & 0x0000FF00) >> 8;
    led_matrix_buffer[x][y].b = (color & 0x000000FF);
    
    return err_code;
}

uint32_t drv_ws2812_rectangle_draw(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint32_t color)
{
    uint32_t err_code;
    for(int h = y; h < (y + height); h++)
    {
        for(int w = x; w < (x + width); w++)
        {
            err_code = drv_ws2812_pixel_draw(w, h, color);
            if(err_code) return err_code;
        }
    }
    return NRF_SUCCESS;
}

