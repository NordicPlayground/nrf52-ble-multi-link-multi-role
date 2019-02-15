#include <math.h>
#include "neopixel_effects.h"
#include "nrf_error.h"
#include "app_timer.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

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
    float    relative_age;
    uint32_t start_color, end_color;
    float    start_color_ch[3];
    float    end_color_ch[3];
    bool     timer_running;
    float    current_rssi_rel;
} m_effect_state;

static uint32_t fade_color(uint32_t color, uint32_t alpha)
{
    uint32_t ret_val = (((color & 0xFF0000) * alpha) >> 8) & 0xFF0000;
    ret_val |= (((color & 0x00FF00) * alpha) >> 8) & 0x00FF00;
    ret_val |= (((color & 0x0000FF) * alpha) >> 8) & 0x0000FF;
    return ret_val;
}

typedef struct RgbColor
{
    unsigned char r;
    unsigned char g;
    unsigned char b;
} RgbColor;

typedef struct HsvColor
{
    unsigned char h;
    unsigned char s;
    unsigned char v;
} HsvColor;

uint32_t HsvToRgb(uint8_t h, uint8_t s, uint8_t v)
{
    RgbColor rgb;
    unsigned char region, remainder, p, q, t;

    if (s == 0)
    {
        rgb.r = v;
        rgb.g = v;
        rgb.b = v;
        return  (uint32_t)rgb.r | (uint32_t)rgb.g << 8 | (uint32_t)rgb.b << 16;
    }

    region = h / 43;
    remainder = (h - (region * 43)) * 6; 

    p = (v * (255 - s)) >> 8;
    q = (v * (255 - ((s * remainder) >> 8))) >> 8;
    t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;

    switch (region)
    {
        case 0:
            rgb.r = v; rgb.g = t; rgb.b = p;
            break;
        case 1:
            rgb.r = q; rgb.g = v; rgb.b = p;
            break;
        case 2:
            rgb.r = p; rgb.g = v; rgb.b = t;
            break;
        case 3:
            rgb.r = p; rgb.g = q; rgb.b = v;
            break;
        case 4:
            rgb.r = t; rgb.g = p; rgb.b = v;
            break;
        default:
            rgb.r = v; rgb.g = p; rgb.b = q;
            break;
    }

    return (uint32_t)rgb.r | (uint32_t)rgb.g << 8 | (uint32_t)rgb.b << 16;
}

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

static void sinus_effect2_rssi(float rel_rssi)
{
    static float xf, yf;
    static float center_x = (float)LED_MATRIX_WIDTH / 2.0f - 0.5f;
    static float center_y = (float)LED_MATRIX_HEIGHT / 2.0f - 0.5f;
    static float phase = 0.0f, phase2 = 0.0f;
    static float distance_from_center,alpha;
    static nrf_gfx_point_t point;
    static float fake_rssi = 0.00f;
    fake_rssi += 0.01f;
    if(fake_rssi > 1.00f) fake_rssi = 0.0f;
    uint32_t rssi_color = HsvToRgb((uint8_t)((0.68f - (rel_rssi / 3.0f)) * 255.0f), 255, 255);
    NRF_LOG_INFO("Fake rssi: %i", (uint32_t)(rel_rssi * 1000.0f));
    for(int x = 0; x < LED_MATRIX_WIDTH; x++)
    {
        xf = (float)x;
        for(int y = 0; y < LED_MATRIX_HEIGHT; y++)
        {
            yf = (float)y;
            point.x = x;
            point.y = y;
            distance_from_center = sqrtf((center_x - xf) * (center_x - xf) + (center_y - yf) * (center_y - yf));

            alpha = distance_from_center * 2.0f;
            alpha += phase;
                 
            // Apply sinus function
            alpha = (sinf(alpha) + 1.0f) / 2.0f;

            //while(alpha > 1.0f) alpha -= 1.0f;

            // Primitive gamma compentasion to make low brightness values darker
            alpha *= alpha;

            if(m_effect_state.effect_duration > 0)
            {
                nrf_gfx_point_draw(m_led_matrix_ptr, &point, fade_color(m_effect_state.start_color, (uint32_t)((1.0f-m_effect_state.relative_age) * 255.0f)) + 
                                                             fade_color(rssi_color, (uint32_t)(alpha * m_effect_state.relative_age * 255.0f)));
            }
            else
            {
                // Update pixel
                nrf_gfx_point_draw(m_led_matrix_ptr, &point, fade_color(rssi_color, (uint32_t)(alpha * 255.0f)));
            }
        }
    }
    phase -= 0.20f * (rel_rssi + 0.1f);
    if(phase > 1.0f) phase -= 1.0f;
    nrf_gfx_display(m_led_matrix_ptr);   
}


static void radial_twist(void)
{
    static float xf, yf, alpha, phase = 0.0f;
    static float center_x = (float)LED_MATRIX_WIDTH / 2.0f - 0.5f;
    static float center_y = (float)LED_MATRIX_HEIGHT / 2.0f - 0.5f;
    static float rotation = 0.0f;
    static float angle_from_center;
    static nrf_gfx_point_t point;
    for(int x = 0; x < LED_MATRIX_WIDTH; x++)
    {
        xf = (float)x;
        for(int y = 0; y < LED_MATRIX_HEIGHT; y++)
        {
            yf = (float)y;
            point.x = x;
            point.y = y;
            angle_from_center = atan2f(y - center_y, x - center_x);

            alpha = 1.0f - (angle_from_center + 3.141593f) / (3.141593f * 2.0f);
            alpha += phase;
            while(alpha > 1.0f) alpha -= 1.0f;

            // Primitive gamma compentasion to make low brightness values darker
            alpha *= alpha;
            alpha *= alpha;

            // Update pixel
            nrf_gfx_point_draw(m_led_matrix_ptr, &point, fade_color(0xFF0000, (uint32_t)(alpha * 255.0f)));
        }
    }
    phase -= 0.012f;
    if(phase < 0.0f) phase += 1.0f;
    nrf_gfx_display(m_led_matrix_ptr); 
}


static void fill_screen_with_color(uint32_t color, float rel_rssi)
{
    static nrf_gfx_rect_t r;
    float modulo;
    int num_squares = LED_MATRIX_HEIGHT;// (int)((float)LED_MATRIX_HEIGHT * rel_rssi);
    r.x = 0;
    r.y = 0;
    r.width = LED_MATRIX_WIDTH;
    r.height = num_squares + 1;
    modulo = (float)LED_MATRIX_HEIGHT * rel_rssi - num_squares;
    modulo *= modulo;
    nrf_gfx_rect_draw(m_led_matrix_ptr, &r, 1, color, true); 
    r.y = num_squares;
    r.height = 2;
    nrf_gfx_rect_draw(m_led_matrix_ptr, &r, 1, fade_color(color, (int)(modulo * 255.0f)), true);
    r.y = num_squares + 1;
    r.height = LED_MATRIX_HEIGHT - num_squares;
    nrf_gfx_rect_draw(m_led_matrix_ptr, &r, 1, 0x000000, true); 
    nrf_gfx_display(m_led_matrix_ptr);
    //NRF_LOG_INFO("int: %i, modulo: %i", num_squares, (int)(modulo * 1000.0f));
}


static void neopixel_effects_update(void * p)
{
    float rel_age;
    float tmp_color[3];
    static float rel_rssi = 1.0f;
    if(rel_rssi < m_effect_state.current_rssi_rel) rel_rssi += 0.001f;
    else rel_rssi -= 0.001f;

    switch(m_effect_state.current_mode)
    {
        case NPEFFECT_MODE_FADE_TO_COLOR:
        case NPEFFECT_MODE_COLOR_AND_RSSI:
            if(m_effect_state.effect_age > m_effect_state.effect_duration)
            {
                m_effect_state.effect_age = m_effect_state.effect_duration;
            }
            rel_age = (float)m_effect_state.effect_age / (float)m_effect_state.effect_duration;
            for(int i = 0; i < 3; i++)
            {
                tmp_color[i] = m_effect_state.start_color_ch[i] * (1.0f - rel_age) + m_effect_state.end_color_ch[i] * rel_age;
            }
            m_effect_state.current_color = COLOR_INT_FROM_FLOAT_BUF(tmp_color);
            
            fill_screen_with_color(m_effect_state.current_color, rel_rssi);
            break;
        case NPEFFECT_MODE_FLASHY:
            sinus_effect();
            break;

        case NPEFFECT_MODE_COLOR_BY_RSSI:
            if(m_effect_state.effect_age > m_effect_state.effect_duration)
            {
                m_effect_state.effect_duration = 0;
                m_effect_state.effect_age = 0;
            }
            if(m_effect_state.effect_duration > 0)
            {
                m_effect_state.relative_age = (float)m_effect_state.effect_age / (float)m_effect_state.effect_duration;
            }
            sinus_effect2_rssi(rel_rssi);
            break;

        case NPEFFECT_MODE_RADIAL_TWIST:
            radial_twist();
            break;
    }
    if(m_effect_state.effect_duration != 0) 
    {
        m_effect_state.effect_age += LED_UPDATE_INTERVAL_MS;
    }
    /*if(m_effect_state.effect_age > m_effect_state.effect_duration)
    {
        m_effect_state.current_mode = NPEFFECT_MODE_IDLE;
    }*/
}


uint32_t neopixel_effects_init(nrf_lcd_t * led_matrix)
{
    m_led_matrix_ptr = led_matrix;
    uint32_t err_code = app_timer_create(&m_neopixel_effects_update_timer_id, APP_TIMER_MODE_REPEATED, neopixel_effects_update);
    if(err_code != NRF_SUCCESS) return err_code;
    m_effect_state.current_mode = NPEFFECT_MODE_IDLE;
    m_effect_state.timer_running = false;
    m_effect_state.current_color = 0;
    m_effect_state.current_rssi_rel = 1.0f;
    return NRF_SUCCESS;
}


static float scale_rssi(int8_t rssi_unscaled)
{
    float rssi;
    if(rssi_unscaled != 0)
    {
        rssi = ((float)(rssi_unscaled + 100) / 70.0f);
        if(rssi > 1.0f)
        {
            rssi = 1.0f;
        }
        return rssi;
    } 
    return 1.0f;
}


uint32_t neopixel_effect_start(neopixel_effect_config_t * effect)
{
    uint32_t err_code;
    if(!m_effect_state.timer_running)
    {
        err_code = app_timer_start(m_neopixel_effects_update_timer_id, 
                                   APP_TIMER_TICKS(LED_UPDATE_INTERVAL_MS), 0);
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

        case NPEFFECT_MODE_COLOR_AND_RSSI:
            if(effect->new_rssi != 0)   
            {
                m_effect_state.current_rssi_rel = scale_rssi(effect->new_rssi);  
            }
            if(effect->effect_duration > 0)
            {
                m_effect_state.effect_age = 0;
                m_effect_state.effect_duration = effect->effect_duration;
                m_effect_state.start_color = effect->new_color;
                m_effect_state.start_color_ch[0] = COLOR_CH_RED_FLOAT(effect->new_color);
                m_effect_state.start_color_ch[1] = COLOR_CH_GREEN_FLOAT(effect->new_color);
                m_effect_state.start_color_ch[2] = COLOR_CH_BLUE_FLOAT(effect->new_color);
            }
            break;

        case NPEFFECT_MODE_FLASHY:
            m_effect_state.current_mode = NPEFFECT_MODE_FLASHY;
            break;

        case NPEFFECT_MODE_COLOR_BY_RSSI:
            if(effect->new_rssi != 0)   
            {
                m_effect_state.current_rssi_rel = scale_rssi(effect->new_rssi);  
            }
            if(effect->effect_duration > 0)
            {
                m_effect_state.effect_age = 0;
                m_effect_state.effect_duration = effect->effect_duration;
                m_effect_state.start_color = effect->new_color;
                m_effect_state.start_color_ch[0] = COLOR_CH_RED_FLOAT(effect->new_color);
                m_effect_state.start_color_ch[1] = COLOR_CH_GREEN_FLOAT(effect->new_color);
                m_effect_state.start_color_ch[2] = COLOR_CH_BLUE_FLOAT(effect->new_color);
            }
            m_effect_state.current_mode = NPEFFECT_MODE_COLOR_BY_RSSI;
            break;

        case NPEFFECT_MODE_RADIAL_TWIST:
            m_effect_state.current_mode = NPEFFECT_MODE_RADIAL_TWIST;
            break;
    }

    return NRF_SUCCESS;
}
