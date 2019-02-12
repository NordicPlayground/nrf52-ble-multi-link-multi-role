#include <math.h>
#include "neopixel_effects.h"
#include "nrf_error.h"
#include "app_timer.h"

APP_TIMER_DEF(m_neopixel_effects_update_timer_id);
#define COLOR_CH_RED_FLOAT(a) ((float)(((a) >> 0) & 0xFF) / 255.0f)
#define COLOR_CH_GREEN_FLOAT(a) ((float)(((a) >> 8) & 0xFF) / 255.0f)
#define COLOR_CH_BLUE_FLOAT(a) ((float)(((a) >> 16) & 0xFF) / 255.0f)
#define COLOR_INT_FROM_FLOAT_BUF(buf) ((uint32_t)(buf[0] * 255.0f) | (uint32_t)(buf[1] * 255.0f) << 8 | (uint32_t)(buf[2] * 255.0f) << 16)  

static nrf_lcd_t * m_led_matrix_ptr = 0;

static struct
{
    uint32_t current_mode;
    uint32_t current_color;
    uint32_t effect_age, effect_duration;
    uint32_t start_color, end_color;
    float    start_color_ch[3];
    float    end_color_ch[3];
    bool     timer_running;
} m_effect_state;

static void sinus_effect(void)
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

static void fill_screen_with_color(uint32_t color)
{
    static nrf_gfx_rect_t r = {0,0,LED_MATRIX_WIDTH,LED_MATRIX_HEIGHT};
    nrf_gfx_rect_draw(m_led_matrix_ptr, &r, 1, color, true);    
    nrf_gfx_display(m_led_matrix_ptr);
}

static void neopixel_effects_update(void * p)
{
    float rel_age;
    float tmp_color[3];
    if(m_effect_state.effect_age > m_effect_state.effect_duration)
    {
        m_effect_state.effect_age = m_effect_state.effect_duration;
    }
    switch(m_effect_state.current_mode)
    {
        case NPEFFECT_MODE_FADE_TO_COLOR:
            rel_age = (float)m_effect_state.effect_age / (float)m_effect_state.effect_duration;
            for(int i = 0; i < 3; i++)
            {
                tmp_color[i] = m_effect_state.start_color_ch[i] * (1.0f - rel_age) + m_effect_state.end_color_ch[i] * rel_age;
            }
            m_effect_state.current_color = COLOR_INT_FROM_FLOAT_BUF(tmp_color);
            fill_screen_with_color(m_effect_state.current_color);
            break;
    }
    m_effect_state.effect_age += LED_UPDATE_INTERVAL_MS;
    if(m_effect_state.effect_age > m_effect_state.effect_duration)
    {
        m_effect_state.current_mode = NPEFFECT_MODE_IDLE;
    }
}


uint32_t neopixel_effects_init(nrf_lcd_t * led_matrix)
{
    m_led_matrix_ptr = led_matrix;
    uint32_t err_code = app_timer_create(&m_neopixel_effects_update_timer_id, APP_TIMER_MODE_REPEATED, neopixel_effects_update);
    if(err_code != NRF_SUCCESS) return err_code;
    m_effect_state.current_mode = NPEFFECT_MODE_IDLE;
    m_effect_state.timer_running = false;
    m_effect_state.current_color = 0;
    return NRF_SUCCESS;
}

uint32_t neopixel_effect_start(neopixel_effect_config_t * effect)
{
    uint32_t err_code;
    if(!m_effect_state.timer_running)
    {
        err_code = app_timer_start(m_neopixel_effects_update_timer_id, APP_TIMER_TICKS(LED_UPDATE_INTERVAL_MS), 0);
        m_effect_state.timer_running = true;
        if(err_code != NRF_SUCCESS) return err_code;
    }

    switch(effect->effect_mode)
    {
        case NPEFFECT_MODE_FADE_TO_COLOR:
            m_effect_state.current_mode = NPEFFECT_MODE_FADE_TO_COLOR;
            m_effect_state.effect_age = 0;
            m_effect_state.effect_duration = effect->effect_duration;
            m_effect_state.end_color = effect->new_color;
            m_effect_state.end_color_ch[0] = COLOR_CH_RED_FLOAT(effect->new_color);
            m_effect_state.end_color_ch[1] = COLOR_CH_GREEN_FLOAT(effect->new_color);
            m_effect_state.end_color_ch[2] = COLOR_CH_BLUE_FLOAT(effect->new_color);
            m_effect_state.start_color_ch[0] = COLOR_CH_RED_FLOAT(m_effect_state.current_color);
            m_effect_state.start_color_ch[1] = COLOR_CH_GREEN_FLOAT(m_effect_state.current_color);
            m_effect_state.start_color_ch[2] = COLOR_CH_BLUE_FLOAT(m_effect_state.current_color);
            break;
    }

    return NRF_SUCCESS;
}
