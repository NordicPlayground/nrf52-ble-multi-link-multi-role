/**
 * Copyright (c) 2015 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
 /** @file
 *
 * @defgroup ble_sdk_app_beacon_main_gui ft6206.c
 * @{
 * @ingroup ble_sdk_app_beacon_gui
 * @brief Touchscreen driver for FT6206 chip.
 *
 * Touchscreen: https://www.adafruit.com/product/1947
 * This file contains the source code for the FT6206 touchscreen controller chip running in polled mode.
 * (interrupt mode requires hardware modification).
 * 
 */
#include "ft6206.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

uint8_t const ft6206_ven_id_reg_addr = FT6206_REG_VENDID;
uint8_t const ft6206_chip_id_reg_addr = FT6206_REG_CHIPID;
uint8_t const ft6206_num_touches_reg_addr = FT6206_REG_NUMTOUCHES;
uint8_t const ft6206_read_data_reg_addr = 0x00;

//Forward declarations of internal functions
static void twi_config(void);  //Initialize TWI master controller
static void ft6206_get_all_registers_bg(void);   //Burst read from FT6206 in non-blocking mode
static void get_data_cb(ret_code_t result, void *p_user_data);  //Callback function for burst read
static void ft6206_get_touch_bg(void);   //Read touch register of FT6206 in non-blocking mode
static void get_touch_cb(ret_code_t result, void *p_user_data);  //Callback function for read touch register function 
static void ft6206_get_point(void);  //Post-processing of touch controller data gotten from a burst read
static long map(long x, long in_min, long in_max, long out_min, long out_max);  //Mapping function for orientation

// Set threshold.
static uint8_t const default_config[] = { FT6206_REG_THRESHHOLD, FT6206_DEFAULT_THRESSHOLD };
static app_twi_transfer_t const ft6206_init_transfers[FT6206_INIT_TRANSFER_COUNT] =
{
    APP_TWI_WRITE(FT6206_ADDR, default_config, sizeof(default_config), 0)
};

static void twi_config(void)
{
    uint32_t err_code;

    nrf_drv_twi_config_t const config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_LOWEST,
       .clear_bus_init     = false
    };

    err_code = app_twi_init(&m_app_twi, &config);
    APP_ERROR_CHECK(err_code);
}

static void get_data_cb(ret_code_t result, void *p_user_data)
{   
    ft6206_get_point();
    m_gui_fns_struct.gtu(m_x, m_y, 1);
}

//MQ  burst read on TWI: keep reading n registers with no TWI STOP assertion on bus
static void ft6206_get_all_registers_bg()
{
    static app_twi_transfer_t const transfers[] =  { FT6206_READ(&ft6206_read_data_reg_addr, &m_buffer[0], 16) };
    
    static app_twi_transaction_t const transaction =
    {
        .callback            = get_data_cb,
        .p_user_data         = NULL,
        .p_transfers         = transfers,
        .number_of_transfers = sizeof(transfers) / sizeof(transfers[0])
    };
    
    APP_ERROR_CHECK(app_twi_schedule(&m_app_twi, &transaction));   
}

/* Simple debouncing algorithm: higher number means we wait for more discrete touch datapoints before validating it as a touch. */
#define DEBOUNCE_SENSITIVITY 5
static void get_touch_cb(ret_code_t result, void *p_user_data)
{
    static int is_touchscreen_pressed = 0;
    
    m_touches = m_buffer[0x0];
    if (m_buffer[0] == 1 || m_buffer[0] == 2) {  
        ++is_touchscreen_pressed;        
    }
    else {        
        is_touchscreen_pressed = 0;
        //NRF_LOG_INFO("touch count is 0\r\n");
        m_gui_fns_struct.gtu(-1, -1, 0);        
    }
    if (is_touchscreen_pressed == DEBOUNCE_SENSITIVITY) {        
        ft6206_get_all_registers_bg();
    }
}

static void ft6206_get_touch_bg()
{
    static app_twi_transfer_t const transfers[] = { FT6206_READ_NUM_TOUCHES(&m_buffer[0]) };
    
    static app_twi_transaction_t const transaction =    
    {
        .callback            = get_touch_cb,
        .p_user_data         = NULL,
        .p_transfers         = transfers,
        .number_of_transfers = sizeof(transfers) / sizeof(transfers[0])
    };    

    //MQ  Must test if TWI block is available or we overflow internal software queues
    if (app_twi_is_idle(&m_app_twi)) {
        APP_ERROR_CHECK(app_twi_schedule(&m_app_twi, &transaction));
    }
}

static long map(long x, long in_min, long in_max, long out_min, long out_max) 
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static void ft6206_get_point(void)
{    
    if (m_touches > 2) {
        m_touches = 0;
        m_x = m_y = 0;
    }
    
    if (m_touches == 0) {
        m_x = m_y = 0;
        return;
    }
       
    for (uint8_t i=0; i<2; i++) {
        m_touchX[i] = m_buffer[0x03 + i*6] & 0x0F;
        m_touchX[i] <<= 8;
        m_touchX[i] |= m_buffer[0x04 + i*6]; 
        m_touchY[i] = m_buffer[0x05 + i*6] & 0x0F;
        m_touchY[i] <<= 8;
        m_touchY[i] |= m_buffer[0x06 + i*6];
        m_touchID[i] = m_buffer[0x05 + i*6] >> 4;
    }
    
    //Remap touch coordinates to account for orientation
    m_x = map(m_touchX[0], 0, 240, 240, 0);
    m_y = map(m_touchY[0], 0, 320, 320, 0);

    //NRF_LOG_INFO("m_x = %i\r\n", m_x);
    //NRF_LOG_INFO("m_y = %i\r\n\r\n", m_y);
    //NRF_LOG_FLUSH();
}

//tmp -- remove!
#if 0
extern int UG_TextboxSetBackColor( void*, int, int );
extern int UG_TextboxShow(void *, int);
extern int UG_TextboxHide(void *, int);
extern void* ptr_window7;
#endif

// Timeout handler for the repeated timer
static void timeout_handler(void * p_context)
{
#if 0
    static bool switcher = true;
    NRF_LOG_INFO("tick\r\n");
    if (switcher) 
         UG_TextboxSetBackColor( ptr_window7, 0, 0x9A85 );
    else
        UG_TextboxSetBackColor( ptr_window7, 0, 0xFFBB );
//    if (switcher)
//        UG_TextboxHide( ptr_window7, 0 );
//    else
//        UG_TextboxShow( ptr_window7, 0 );
    switcher = !switcher; 
#endif    
    ft6206_get_touch_bg();
    m_gui_fns_struct.gu(); //call GUI lib update function periodically
}

static void timer_init(void)
{
    //rtc_config();
    uint32_t err_code;
    
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
    
    err_code = app_timer_create(&m_gui_timer_id, APP_TIMER_MODE_REPEATED, timeout_handler);
    APP_ERROR_CHECK(err_code);
    
    err_code = app_timer_start(m_gui_timer_id, APP_TIMER_TICKS(50), NULL);
    APP_ERROR_CHECK(err_code);
}


//MQ  The FT6206 chip driver intialization function
//    takes GUI lib's update and touch update functions.  Bit of a hack but
//    want to keep direct GUI lib calls out of the driver so no linking dependency
//    between ft6206.c's and ugui.c's (or alternate GUI lib's) compilation units.   
//    This way we can technically swap uGUI with another GUI library.
void init_ft6206(void *gui_touch_update_function, void *gui_update_function)
{
    if ((!gui_touch_update_function) || (!gui_update_function))
        return;
    m_gui_fns_struct.gtu = (gui_touch_update)gui_touch_update_function;
    m_gui_fns_struct.gu  = (gui_update)gui_update_function;
    twi_config();
    APP_ERROR_CHECK(app_twi_perform(&m_app_twi, ft6206_init_transfers, FT6206_INIT_TRANSFER_COUNT, NULL)); //blocking TWI write
    timer_init();
}
