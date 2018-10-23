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
#pragma once

#include "app_twi.h"
#include "boards.h"

//MQ 1/20/18  Switching to app_timer instead of hogging whole RTC2 just for GUI updates
#include "app_timer.h"
#include "nrf_drv_clock.h"
//#include "nrf_drv_rtc.h"

//MQ    For i2c
#define TWI_INSTANCE_ID             1
#define MAX_PENDING_TRANSACTIONS    5
APP_TWI_DEF(m_app_twi, MAX_PENDING_TRANSACTIONS, TWI_INSTANCE_ID);

#ifdef __cplusplus
extern "C" {
#endif

#define FT6206_ADDR             (0x38U >> 0) 
#define FT6206_G_FT5201ID       0xA8
#define FT6206_REG_NUMTOUCHES   0x02

#define FT6206_NUM_X            0x33
#define FT6206_NUM_Y            0x34

#define FT6206_REG_MODE         0x00
#define FT6206_REG_CALIBRATE    0x02
#define FT6206_REG_WORKMODE     0x00
#define FT6206_REG_FACTORYMODE  0x40
#define FT6206_REG_THRESHHOLD   0x80
#define FT6206_REG_POINTRATE    0x88
#define FT6206_REG_FIRMVERS     0xA6
#define FT6206_REG_CHIPID       0xA3
#define FT6206_REG_VENDID       0xA8

#define FT6206_NUMBER_OF_REGISTERS 32

// calibrated for Adafruit 2.8" ctp screen
#define FT6206_DEFAULT_THRESSHOLD 128
     
extern uint8_t const ft6206_ven_id_reg_addr;
extern uint8_t const ft6206_chip_id_reg_addr;
extern uint8_t const ft6206_num_touches_reg_addr;
extern uint8_t const ft6206_read_data_reg_addr;

#define FT6206_READ(p_reg_addr, p_buffer, byte_cnt) \
    APP_TWI_WRITE(FT6206_ADDR, p_reg_addr, 1,        APP_TWI_NO_STOP), \
    APP_TWI_READ (FT6206_ADDR, p_buffer,   byte_cnt, 0)
    
#define FT6206_READ_VENDOR_ID(p_buffer) \
    FT6206_READ(&ft6206_ven_id_reg_addr, p_buffer, 1)
    
#define FT6206_READ_CHIP_ID(p_buffer) \
    FT6206_READ(&ft6206_chip_id_reg_addr, p_buffer, 1)

#define FT6206_READ_NUM_TOUCHES(p_buffer) \
    FT6206_READ(&ft6206_num_touches_reg_addr, p_buffer, 1)

#define FT6206_READ_DATA(p_buffer) \
    FT6206_READ(&ft6206_read_data_reg_addr, p_buffer, 16) 

#define FT6206_INIT_TRANSFER_COUNT 1
#define FT6206_NUM_TOUCH_REG_TRANSFER_COUNT 1

extern app_twi_transfer_t const
    ft6206_init_transfers[FT6206_INIT_TRANSFER_COUNT];

/* MQ  These functions pointers would be to the GUI library's functions that update itself upon touch and that redraw. */
typedef void (*gui_touch_update)(int16_t, int16_t, uint8_t); /* void UG_TouchUpdate( UG_S16 xp, UG_S16 yp, UG_U8 state ) */
typedef void (*gui_update)(void);                            /* void UG_Update( void ); */
/* MQ  Encapsulate GUI functions into structure for access by other driver functions. */
struct gui_functions_struct {
    gui_touch_update gtu;
    gui_update gu;
};
static struct gui_functions_struct m_gui_fns_struct ={0}; /* Holds function ptrs to GUI lib functions */

void init_ft6206(void *gui_touch_update_function, void *gui_update_function);


#define BUFFER_SIZE  FT6206_NUMBER_OF_REGISTERS
static uint8_t m_buffer[BUFFER_SIZE] = {0};  //Buffer for TWI burst read from sensor.
static uint8_t m_touches = 0;
static uint16_t m_touchX[2] = {0,0};
static uint16_t m_touchY[2] = {0,0};
static volatile uint16_t m_touchID[2] = {0,0}; //MQ 10/16/2017  Declare volatile to kill compiler warning
static uint16_t m_x = 0, m_y = 0;
//static const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(2); /**< Declaring an instance of nrf_drv_rtc for RTC2. */

APP_TIMER_DEF(m_gui_timer_id);

#ifdef __cplusplus
}
#endif
