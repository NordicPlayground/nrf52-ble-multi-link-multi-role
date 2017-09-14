# nRF52 Multi Link Multi Role Demo
----------------------------------

**Scope:** This is a demo example used to showcase the ability of the S132 and S140 SoftDevice to connect to up to 20 devices in either peripheral or central role. 
One device acts as a central node (called the aggregator), which can connect to up to 20 other devices and show the state of these devices over the UART or through an Android application (using one of the 20 available links to connect to the aggregator). 

Currently two types of peripherals can be connected to the aggregator:
- nRF52/nRF52840 devkits running the ble_app_blinky example from the SDK
- The Thingy:52

Requirements
------------
- Keil uVision v5
- nRF5 SDK version 14.0.0
- nRF52 DK (PCA10040) or nRF52840 DK (PCA10056) for the Aggregator
- nRF52 DK's, nRF52840 DK's or Thingy:52's for the connected peripherals
- (optional) Android phone/tablet for displaying the state of the peripherals connected to the aggregator

Todo
----

- Release the Android application (currently under development)
- Update projects to support Segger Embedded Studio (only Keil support ATM)

About this project
------------------
This application is one of several applications that has been built by the support team at Nordic Semiconductor, as a demo of some particular feature or use case. It has not necessarily been thoroughly tested, so there might be unknown issues. It is hence provided as-is, without any warranty. 

However, in the hope that it still may be useful also for others than the ones we initially wrote it for, we've chosen to distribute it here on GitHub. 

The application is built to be used with the official nRF5 SDK, that can be downloaded from developer.nordicsemi.com

Please post any questions about this project on [devzone](https://devzone.nordicsemi.com)