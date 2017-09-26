#ifndef __RADIO_TIME_ANALYSIS_H
#define __RADIO_TIME_ANALYSIS_H

#include <stdint.h>

/**@brief Function for enabling radio timing measurements when running the BLE SoftDevice
 *
 * @details Measures the length of the BLE RX event by connecting a timer to the ADDRESS event and the END event through PPI. 
 */
void radio_analysis_enable(void);

#endif
