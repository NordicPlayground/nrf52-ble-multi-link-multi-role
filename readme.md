# Bluetooth 5 multi link demo
## Overview
This demo is meant to illustrate the multi role and multi phy capabilities of the Nordic SoftDevices.

The demo is based around an aggregator node, running on an nRF52840DK (recommended) or an nRF52DK. 
The aggregator node is set up to connect to any device in the area that supports either the Thingy:52 services, or the LBS service used by the ble_app_blinky example. By using the 20 link feature of the S132 or S140 SoftDevices the aggregator can connect to up to 20 devices at the same time, where 19 of these devices will be ble_app_blinky or Thingy devices, and 1 device can be a smart phone. 
The aggregator will collect information on all the connected devices, and relay the information to the UART and to the connected smart phone over BLE. From the phone application the user can control the LED color and state of the connected devices, and get an overview over the following details from the connected devices:
- Advertising name
- Connection handle (ID) 
- Button state
- LED state
- Phy
- RSSI

![System overview](https://github.com/NordicSemiconductor/nrf52-ble-multi-link-multi-role/blob/master/pics/overview.png)

## Requirements
- 1x nRF52840DK or nRF52DK for the aggregator (nRF52840DK recommended for Bluetooth long range support)
- Up to 19 total of the following devices
  - Thingy:52 running the standard firmware
  - nRF52DK or nRF52840DK running the ble_app_blinky firmware available in the nRF5 SDK
  - nRF52840DK and an [Adafruit Touch Display](https://www.adafruit.com/product/1947) running the ble_peripheral_long_range demo included in this repo
  - nRF52840DK and a [Neopixel Shield](https://www.adafruit.com/product/1430) running the ble_peripheral_long_range_color demo included in this repo
- An Android device running the multi link app (optimized for large phones or small tablets). APK included in the *android_apk* folder. 
- [nRF5_SDK_v15.2.0](http://developer.nordicsemi.com/nRF5_SDK/nRF5_SDK_v15.x.x/nRF5_SDK_15.2.0_9412b96.zip) is required to compile the examples.

## Demo usage
### Buttons and LED's
**Aggregator**

| LED |                 | Button |          |
| --- | --------------- | ------ | -------- |
| 1 | **Blinking:** Advertising <br>**On:** Connected to phone app | 1 | Disconnect phone app |
| 2 | **Fast blink:** Scanning for BLE devices in normal 1M mode <br>**Slow blink:** Scanning for BLE devices in coded phy (long range) mode <br>**Off:** Scanning disabled, or busy establishing a connection to a device | 2 | Enable/disable scanning | 
| 3 | Shows incoming button presses from connected long range peripherals | 3 | Toggles LED on/off on all connected peripherals | 
| 4 | One or more long range devices connected | 4 | - |

**ble_long_range_peripheral_color**

| LED |                 | Button |          |
| --- | --------------- | ------ | -------- |
| 1 | **On:** Advertising | 1 | Toggle between normal mode and range test mode (see description below) |
| 2 | **On:** Connected   | 2 | Toggle between smooth or fast LED fading when changing LED color |
| 3 | - | 3 | - |
| 4 | - | 4 | Send button status to the aggregator | 

*Operation mode:* This demo can be used in two different modes when connected to the aggregator. In normal mode the LED matrix will show a single color, and the color and intensity can be controlled by the aggregator. In long range test mode the color will be set by the link quality. When the link is good the color will be green, and as the link gets poorer the color will fade into yellow and red while the speed of the pulsing will be reduced. This allows the quality of the link to be monitored continuosluy while the kit is moved around. 

*LED fade mode:* In normal mode the LED color is controlled by the aggregator. By default the LED's will fade slowly from one color to another when changing the color on the aggregator side, but this can be changed to make the color change more immediate. This makes it easier to evaluate latency, and makes the LED update similar to how it works on the Thingy:52 devices. 
