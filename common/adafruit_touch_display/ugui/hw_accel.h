#pragma once

#include "nrf_gfx.h"

//MQ
UG_RESULT _HW_draw_line(UG_S16 x1, UG_S16 y1, UG_S16 x2, UG_S16 y2, UG_COLOR c)
{
    nrf_gfx_line_t line;
    ret_code_t err_code;
    
    line.x_start = x1;
    line.y_start = y1;
    line.x_end = x2;
    line.y_end = y2;
    line.thickness = 1; //MQ  10/15/17  adjustable
    err_code = nrf_gfx_line_draw(p_lcd, &line, c);
    APP_ERROR_CHECK(err_code);
    return UG_RESULT_OK;
}

UG_RESULT _HW_fill_frame( UG_S16 x1, UG_S16 y1, UG_S16 x2 , UG_S16 y2 , UG_COLOR c)
{
    nrf_gfx_rect_t rect;
    ret_code_t err_code;
    
    rect.x = x1;
    rect.y = y1;
    rect.width = x2 - x1;
    rect.height = y2 - y1;
    err_code = nrf_gfx_rect_draw(p_lcd, &rect, 0, c, 1);
    APP_ERROR_CHECK(err_code);
    return UG_RESULT_OK;
}