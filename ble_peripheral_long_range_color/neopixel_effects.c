#include <math.h>
#include "neopixel_effects.h"
#include "nrf_error.h"
#include "app_timer.h"

APP_TIMER_DEF(m_neopixel_effects_update_timer_id);

static nrf_lcd_t * m_led_matrix_ptr = 0;

static struct
{
    uint32_t current_effect_mode;
} m_effect_state;

static void neopixel_effects_update(void * p)
{
    static float xf, yf, center_x = 0.0f, center_y = 0.0f, cx2 = 4.0f, cy2 = 7.0f;
    static float phase = 0.0f, phase2 = 0.0f;
    static float distance_from_center,dfc2, alpha, alpha2;
    static nrf_gfx_point_t point;
    for(int x = 0; x < LED_MATRIX_WIDTH; x++)
    {
        xf = (float)x;
        for(int y = 0; y < LED_MATRIX_HEIGHT; y++)
        {
            yf = (float)y;
            point.x = x;
            point.y = y;
            distance_from_center = sqrtf((center_x - xf) * (center_x - xf) + (center_y - yf) * (center_y - yf));
            dfc2 = sqrtf((cx2 - xf) * (cx2 - xf) + (cy2 - yf) * (cy2 - yf));

            alpha = distance_from_center * 2.0f;
            alpha += phase;
            alpha2 = dfc2 * 0.5f;
            alpha2 += phase2;
                 
            // Apply sinus function
            alpha = (sinf(alpha) + 1.0f) / 2.0f;
            alpha2 = (sinf(alpha2) + 1.0f) / 2.0f;

            //while(alpha > 1.0f) alpha -= 1.0f;

            // Primitive gamma compentasion to make low brightness values darker
            alpha *= alpha;
            alpha2 *= alpha2;

            // Update pixel
            nrf_gfx_point_draw(m_led_matrix_ptr, &point, (int)(alpha * 255.0f) << 16 | (int)(alpha2 * 255.0f));
        }
    }
    phase -= 0.10f;
    phase2 -= 0.022f;
    //if(phase > 1.0f) phase -= 1.0f;
    nrf_gfx_display(m_led_matrix_ptr);   
}


uint32_t neopixel_effects_init(nrf_lcd_t * led_matrix)
{
    m_led_matrix_ptr = led_matrix;
    uint32_t err_code = app_timer_create(&m_neopixel_effects_update_timer_id, APP_TIMER_MODE_REPEATED, neopixel_effects_update);
    if(err_code != NRF_SUCCESS) return err_code;
    
    return NRF_SUCCESS;
}

uint32_t neopixel_effect_start(neopixel_effect_config_t * effect)
{
    uint32_t err_code = app_timer_start(m_neopixel_effects_update_timer_id, APP_TIMER_TICKS(10), 0);
    return err_code;
}
