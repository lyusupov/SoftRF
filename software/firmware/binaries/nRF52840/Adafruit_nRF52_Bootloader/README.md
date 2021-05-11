Every ZIP file in this folder contains a certain version of pre-built Adafruit nRF52 bootloader for LilyGO T-Echo board.<br>
There are few files inside the archive. Use case of each one is determined by it's name suffix.

Suffix|Notes
---|---
s140_6.1.1.hex|use this file for initial (factory) T-Echo programming over SWD. It contains both the bootloader and **S**oft**D**evice
s140_6.1.1.zip|it is useful for the bootloader and SD upgrade by **adafruit-nrfutil** _(over USB CDC)_ or BLE OTA update methods
nosd.uf2|for 'drag-and-drop' upgrade of the bootloader only by USB Mass Storage update method
