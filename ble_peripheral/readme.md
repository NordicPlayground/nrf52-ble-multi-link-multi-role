## Task 1 - Change advertising name and connect to the central device

1) Change the DEVICE_NAME define at the top of main.c to include your central group prefix plus your own unique name.
   For example, if your group prefix is *'GRP1:'* and your name is *'John'* the advertising name should be *'GRP1:John'*.
   WARNING: Don't make the name longer than 25 characters, or it won't fit in the advertise packet. 

2) Compile the code (F7) and run it on your kit. To run the code you can either download it to the kit (CTRL T, L), or start a debug session (press F5 two times). Once the code is running, verify that your board connects to the central device, and that you can see your name in the list. 

3) Press button 1 on your kit, and verify that the value is updated on the central device.

4) Press button 1 on the central device, and verify that LED3 on your kit is updated. 

## Task 2 - Change the output power of your board and see how it affects the range 

1) Inside gap_params_init() in main.c, change the TX output power of the BLE stack by calling sd_ble_gap_tx_power_set(int8_t tx_power)

2) Try different values and see how they affect the maximum range that you can get between the peripheral and the central
   How far can you move away from the central device when the TX power is set to -40?
   What about -20, 0 or 8?
   
## Task 3 - Change the physical RF mode

1) Inside button_event_handler(..) on line ~500 of main.c, add code to request a 2MBPS PHY update when Button 2 is pressed
   Hint 1: To check if the button is pressed or not you can check if the button_action variable equals APP_BUTTON_PUSH
   Hint 2: You can use the sd_ble_gap_phy_request function to change the PHY 
           The first argument to this function is the connection handle, which is stored in the m_conn_handle variable
           The second argument is a pointer to a struct of type ble_gap_phys_t, which contains fields for requesting a change to the TX and RX phy
           You can ask for the TX and RX PHY to be the same, and you can choose between the following values: BLE_GAP_PHY_2MBPS, BLE_GAP_PHY_1MBPS or BLE_GAP_PHY_CODED
		   
2) Flash your kit with the updated code, and verify that you can connect to the central device again

3) Press button 2 on your kit, and verify that the Phy is updated from 1Mbps to 2Mbps on the central side

4) After updating the phy, verify that you can still send button presses to the central, and receive LED updates from the central. 
   Try to experiment with the range, and see how the range is affected by using a different phy.
   
## Task 4 - Add an app_timer instance to send automatic updates every 5 seconds

1) At the top of main.c, use the APP_TIMER_DEF macro to define a new app_timer instance. You can call the instance variable m_data_update_timer. 
  Hint: For more help on using the app_timer library, refer to the [app_timer documentation](https://infocenter.nordicsemi.com/topic/com.nordic.infocenter.sdk5.v14.0.0/lib_timer.html?cp=4_0_0_3_43)
  
2) At the top of main.c, create the callback function that will be called by the app_timer library
   This function will need to be on the form *void func(void *p_context)*
   
3) Inside the main function, create your timer by using app_timer_create(app_timer_id_t const * p_timer_id, app_timer_mode_t mode, app_timer_timeout_handler_t timeout_handler)
   The app_timer_id_t should point to the variable you created in step 1.
   The app_timer_mode should be APP_TIMER_MODE_REPEATED. 
   The timeout handler should point to the function you created in step 2
   
4) Inside the BLE_GAP_EVT_CONNECTED case of the ble_evt_handler(..) function in main.c, start your timer by calling app_timer_start (app_timer_id_t timer_id, uint32_t timeout_ticks, void *p_context)
   The timer_id variable should refer to the variable you created in step 1
   The timeout_ticks variable should be configured to give you a callback time of 5000ms. To convert from ms to ticks you can use the APP_TIMER_TICKS(MS) macro. 
   The p_context variable will not be used, and can be set to 0
   
5) Inside the BLE_GAP_EVT_DISCONNECTED case of the ble_evt_handler(..) function, stop your timer by calling app_timer_stop (app_timer_id_t timer_id) function
   The timer_id refers to the variable you created in step 1
   
6) Inside the callback function you implemented in step 2, add code to send an alternating button press each time the function is called.
   You can copy the code from the button_event_handler, inside the LEDBUTTON_BUTTON case, to see how you can send a button press to the central device. 
   To send an alternating button value you can create a static bool variable inside the callback that you can switch between 1 and 0 for each run of the function, and pass this as a parameter to the ble_lbs_on_button_change function. 
   
7) Compile and load the code. Connect to the central, and verify that the button state will change automatically every 5 seconds.

