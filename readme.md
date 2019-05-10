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
..- Thingy:52 running the standard firmware
..- nRF52DK or nRF52840DK running the ble_app_blinky firmware available in the nRF5 SDK
..- nRF52840DK and an [Adafruit Touch Display](https://www.adafruit.com/product/1947) running the ble_peripheral_long_range demo included in this repo
..- nRF52840DK and a [Neopixel Shield](https://www.adafruit.com/product/1430) running the ble_peripheral_long_range_color demo included in this repo
- An Android device running the multi link app (optimized for large phones or small tablets). APK included in the *android_apk* folder. 
- [nRF5_SDK_v15.2.0](http://developer.nordicsemi.com/nRF5_SDK/nRF5_SDK_v15.x.x/nRF5_SDK_15.2.0_9412b96.zip)

## Demo usage
### Buttons and LED's
**Aggregator**
| LED's  |                 | Buttons |
| ------ | --------------- | ------- |
| LED 1  | **Blinking** Advertising, **ON** Connected to phone app | Button 1 | Disconnect phone app |
| LED 2  | **Fast blink** Scanning for BLE devices in normal 1M mode **Slow blink** Scanning for BLE devices in coded phy (long range) mode | Button 2 | Enable/disable scanning | 
| LED 3  | Shows incoming button presses from connected long range peripherals | Button 3 | Toggles LED on/off on all connected peripherals | 
| LED 4  | One or more long range devices connected | Button 4 |  |