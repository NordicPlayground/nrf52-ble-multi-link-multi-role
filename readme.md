# Bluetooth 5 multi link hands on
### Overview
The scope of this hands on exercise is to get some experience with the multi link feature of the Nordic SoftDevices, as well as experiment with the different physical radio modes introduced by Bluetooth 5. 

The example is based around an aggregator node, running on an nRF52DK or an nRF52840 PDK. The aggregator node is set up to connect to any device in the area that supports either the Thingy:52 services, or the LBS service used by the ble_app_blinky example. By using the 20 link feature of the S132 or S140 SoftDevice the aggregator can connect to up to 20 devices at the same time, where 19 of these devices will be ble_app_blinky or Thingy devices, and 1 device can be a smart phone. 
The aggregator will collect information on all the connected devices, and relay the information to the UART and to the connected smart phone over BLE. From the phone application the user can control the LED color and state of the connected devices, and get an overview over the following details from the connected devices:
- Advertising name
- Connection handle (ID) 
- Button state
- LED state
- Phy
- RSSI

![System overview](https://github.com/NordicSemiconductor/nrf52-ble-multi-link-multi-role/blob/master/pics/overview.png)

### Scope
The hands on portion of this exercise require the attendants to go together in groups, where each group will need 1 aggregator device. All the attendands will connect to the same aggregator device, and use the aggregator device to verify that the hands on is completed. 
The aggregator device will need either a PC with a terminal program or an Android device (preferrably a tablet with a large screen) to show the state of the devices connected to the aggregator. 

### Requirements
To attend the exercise each person will need the following:
- 1x nRF52840 PDK
- A PC running either Windows, Linux, or Mac OS-X
- 1x micro USB cable
- [Segger Embedded Studio](https://www.segger.com/products/development-tools/embedded-studio/)
- The latest version of the [J-Link Software and Documentation pack](https://www.segger.com/downloads/jlink#)
- [nRF5_SDK_v14.0.0](http://developer.nordicsemi.com/nRF5_SDK/nRF5_SDK_v14.x.x/nRF5_SDK_14.0.0_3bcc1f7.zip)

Each group will need the following for the aggregator
- 1x nRF52840 PDK
- A PC running a terminal program, or an Android device with a large screen

## Preparations
1) The *ble_app_multi_link_multi_role* folder will have to be copied into your local SDKv14 folder in order for the examples to compile. 
Create a folder under *\nRF5_SDK_14.0.0\examples\\* called *training* and copy the ble_app_multi_link_multi_role folder into it. When you are finished it should look like this:
![SDK examples folder](https://github.com/NordicSemiconductor/nrf52-ble-multi-link-multi-role/blob/master/pics/sdk_folder.png)
2) One person in each group needs to set up the aggregator. Ask the Nordic attendants for help if you are unsure who should set up the aggregator. 
   * Open the aggregator Segger project file: *\nRF5_SDK_14.0.0\examples\training\ble_app_multi_link_multi_role\ble_aggregator\pca10056\s140\ses\ble_aggregator_pca10056_s140.emProject*
   * Change the m_target_periph_name define on line 120 of main.c to something unique, so that only members of your group will be able to connect to your aggregator. 
   * Change the DEVICE_NAME define on line 81 of main.c to something unique if you want to use the Android application, so that you know which aggregator to connect to. 
   * Build the project and download the code to an nRF52840 PDK. 
   * Connect to the aggregator board either over UART (baudrate 460800) or from the Android application
   * To use the Android application, install the Android apk located in \ble_app_multi_link_multi_role\android_apk


## Task 1 - Change advertising name and connect to the central device
1) Open the ble_peripheral example project, available here: 
*\nRF5_SDK_14.0.0\examples\training\ble_app_multi_link_multi_role\ble_peripheral\pca10056\s140\ses\ble_peripheral_pca10056_s140.emProject*

2) Change the DEVICE_NAME define at the top of main.c to include your central group prefix plus your own unique name.
   For example, if your group prefix is *'GRP1:'* and your name is *'John'* the advertising name should be *'GRP1:John'*.
   WARNING: Don't make the name longer than 25 characters, or it won't fit in the advertise packet. 
   ![Change Advertise Name](https://github.com/NordicSemiconductor/nrf52-ble-multi-link-multi-role/blob/master/pics/per_handson_01.png)

3) Compile the code (F7) and run it on your kit. To run the code you can either download it to the kit (CTRL T, L), or start a debug session (press F5 two times). Once the code is running, verify that your board connects to the central device, and that you can see your name in the list. 
    ```C
    ---- Device list overview (1 device connected) ----
    
    ID   Name           Btn LED Phy   RSSI
     2   MyBlinky       0   0   1Mbps -15
     ```

4) Press button 1 on your kit, and verify that the value is updated on the central device.
    ```C
    ---- Device list overview (1 device connected) ----
    
    ID   Name           Btn LED Phy   RSSI
     2   MyBlinky       1   0   1Mbps -15
    ```
5) Press button 1 on the central device, and verify that LED3 on your kit will light up. 

## Task 2 - Change the output power of your board and see how it affects the range 

1) Inside *gap_params_init()* in main.c, change the TX output power of the BLE stack by calling:
    ```C
    sd_ble_gap_tx_power_set(int8_t tx_power)
    ```
2) Try different values and see how they affect the maximum range that you can get between the peripheral and the central
   How far can you move away from the central device when the TX power is set to -40?
   What about -20, 0 or 8?
   
## Task 3 - Change the physical RF mode

1) Inside *button_event_handler(..)* on line ~500 of main.c, add code to request a 2MBPS PHY update when Button 2 is pressed
   - Hint 1: To check if the button is pressed or not you can check if the *button_action* variable equals *APP_BUTTON_PUSH*
   - Hint 2: You can use the *sd_ble_gap_phy_request(uint16_t conn_handle, ble_gap_phys_t const \*p_gap_phys)* function to change the PHY. 
   The first argument to this function is the connection handle, which is stored in the m_conn_handle variable. 
   The second argument is a pointer to a struct of type ble_gap_phys_t, which contains fields for requesting a change to the TX and RX phy

    You can ask for the TX and RX PHY to be the same, and you can choose between the following values: *BLE_GAP_PHY_2MBPS*, *BLE_GAP_PHY_1MBPS* or *BLE_GAP_PHY_CODED*.
		   
2) Flash your kit with the updated code, and verify that you can connect to the central device again

3) Press button 2 on your kit, and verify that the Phy is updated from 1Mbps to 2Mbps on the central side
    ```C
    ---- Device list overview (1 device connected) ----
    
    ID   Name           Btn LED Phy   RSSI
     1   MyBlinky       0   0   2Mbps -16
     ```

4) After updating the phy, verify that you can still send button presses to the central, and receive LED updates from the central. 
   Try to experiment with the range, and see how the range is affected by using a different phy.
   
5) To be able to used the CODED phy we need to increase the length of the Bluetooth events. The reason for this is that the coded packets are 8 times as long as the normal BLE packets, and require a significantly longer timeslot on air to be sent. 
   To do this navigate to line 5116 of sdk_config.h and change the NRF_SDH_BLE_GAP_EVENT_LENGTH define from 3 to 6. 
   
6) Change the code you wrote in step 1) to set the PHY to CODED instead of 2Mbps. Compile and download the code, and verify that you can successfully change the phy. 

    ```C
    ---- Device list overview (1 device connected) ----
    
    ID   Name           Btn LED Phy   RSSI
     1   MyBlinky       0   0   Coded -16
     ```
	 
7) Double check that the phy is updated by running the debugger and compare the RX packet time, as reported over the RTT log:
   ```C
   <info> app: RX time - 40us
   ```

   When using the standard BLE phy an empty RX packet should be 40us long (not including access address and preamble). How does this change when setting the phy to 2Mbps or coded?
 
## Task 4 - Add an app_timer instance to send automatic updates every 5 seconds

1) At the top of main.c, use the *APP_TIMER_DEF* macro to define a new app_timer instance. You can call the instance variable *m_data_update_timer*. 
  Hint: For more help on using the app_timer library, refer to the [app_timer documentation](https://infocenter.nordicsemi.com/topic/com.nordic.infocenter.sdk5.v14.0.0/lib_timer.html?cp=4_0_0_3_43)
  
2) At the top of main.c, create the callback function that will be called by the app_timer library
   This function will need to be on the form *void func(void \*p_context)*
   
3) Inside the main function, create your timer by using:
    ```C
    app_timer_create(app_timer_id_t const * p_timer_id, app_timer_mode_t mode, app_timer_timeout_handler_t timeout_handler)
    ```
   * The *p_timer_id* parameter should point to the variable you created in step 1.
   * The *app_timer_mode* should be *APP_TIMER_MODE_REPEATED*. 
   * The timeout handler should point to the function you created in step 2
   
4) Inside the *BLE_GAP_EVT_CONNECTED* case of the *ble_evt_handler(..)* function in main.c, start your timer by calling:
    ```C
    app_timer_start (app_timer_id_t timer_id, uint32_t timeout_ticks, void * p_context)
    ```
   * The *timer_id* variable should refer to the variable you created in step 1
   * The *timeout_ticks* variable should be configured to give you a callback time of 5000ms. To convert from ms to ticks you can use the *APP_TIMER_TICKS(MS)* macro. 
   * The *p_context* variable will not be used, and can be set to 0
   
5) Inside the *BLE_GAP_EVT_DISCONNECTED* case of the *ble_evt_handler(..)* function, stop your timer by calling:
    ```C
    app_timer_stop (app_timer_id_t timer_id)
    ```
   * The *timer_id* refers to the variable you created in step 1
   
6) Inside the callback function you implemented in step 2, add code to send an alternating button press each time the function is called.
   You can copy the code from the *button_event_handler*, inside the *LEDBUTTON_BUTTON case*, to see how you can send a button press to the central device. 
    ```C
    NRF_LOG_INFO("Send button state change.");
    err_code = ble_lbs_on_button_change(m_conn_handle, &m_lbs, button_action);
    if (err_code != NRF_SUCCESS && err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
    err_code != NRF_ERROR_INVALID_STATE && err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
    {
        APP_ERROR_CHECK(err_code);
    }
    break;
    ```
   To send an alternating button value you can create a static bool variable inside the callback that you can switch between 1 and 0 for each run of the function, and pass this as a parameter to the *ble_lbs_on_button_change* function. 
   
7) Compile and load the code. Connect to the central, and verify that the button state will change automatically every 5 seconds.

