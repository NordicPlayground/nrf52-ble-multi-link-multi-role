#include "radio_time_analysis.h"
#include "nrf.h"
#include "nrf_log.h"
#include "nrf_sdh.h"
#include "nrf_soc.h"
#include "nrf_nvic.h"

#define RADIO_ANALYSIS_TIMER        NRF_TIMER2
#define RADIO_ANALYSIS_TIMER_IRQn   TIMER2_IRQn
#define RADIO_ANALYSIS_IRQHandler   TIMER2_IRQHandler

/**@brief Function for enabling radio timing measurements when running the BLE SoftDevice
 *
 * @details Measures the length of the BLE RX event by connecting a timer to the ADDRESS event and the END event through PPI. 
 */
void radio_analysis_enable(void)
{
    RADIO_ANALYSIS_TIMER->PRESCALER = 4;
    RADIO_ANALYSIS_TIMER->CC[1] = 4000;
    RADIO_ANALYSIS_TIMER->BITMODE = TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos;
    RADIO_ANALYSIS_TIMER->INTENSET = TIMER_INTENSET_COMPARE0_Msk | TIMER_INTENSET_COMPARE1_Msk;
    RADIO_ANALYSIS_TIMER->SHORTS = TIMER_SHORTS_COMPARE1_CLEAR_Msk | TIMER_SHORTS_COMPARE1_STOP_Msk;

    sd_ppi_channel_assign(0, &NRF_RADIO->EVENTS_ADDRESS, &RADIO_ANALYSIS_TIMER->TASKS_START);
    sd_ppi_channel_assign(1, &NRF_RADIO->EVENTS_END, &RADIO_ANALYSIS_TIMER->TASKS_CAPTURE[0]);
    sd_ppi_channel_assign(2, &NRF_RADIO->EVENTS_END, &NRF_PPI->TASKS_CHG[0].DIS);
    
    sd_ppi_group_assign(0, (1 << 1));

    sd_ppi_channel_enable_set(0x07);

    sd_nvic_SetPriority(RADIO_ANALYSIS_TIMER_IRQn, 7);
    sd_nvic_EnableIRQ(RADIO_ANALYSIS_TIMER_IRQn);
}

/**@brief Interrupt handler run after each timer measurement
 *
 * @details Called 4ms after the radio event, having stored the length of the RX event in the CC[0] register
 */
void RADIO_ANALYSIS_IRQHandler(void)
{
    if(RADIO_ANALYSIS_TIMER->EVENTS_COMPARE[0])
    {
        RADIO_ANALYSIS_TIMER->EVENTS_COMPARE[0] = 0;

    }
    
    if(RADIO_ANALYSIS_TIMER->EVENTS_COMPARE[1])
    {
        RADIO_ANALYSIS_TIMER->EVENTS_COMPARE[1] = 0;
        NRF_LOG_INFO("RX time - %ius", RADIO_ANALYSIS_TIMER->CC[0]);
        RADIO_ANALYSIS_TIMER->CC[0] = 20000;
        NRF_PPI->TASKS_CHG[0].EN = 1;
    }
}
