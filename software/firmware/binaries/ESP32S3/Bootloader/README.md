Every ZIP file in this folder contains a certain version of pre-built [TinyUF2 Bootloader for LilyGO T-Beam Supreme](https://github.com/lyusupov/tinyuf2/) board.<br>

## Instructions

1. Take ESP32 flash download tool from this location: https://www.espressif.com/en/support/download/other-tools <br>

2. Download an appropriate version of the bootloader and unzip the archive ; <br>

3. Connect the SoftRF Prime Edition Mk.3 device to your PC by means of a USB cable (Type-A <-> Type-C) ;

4. Press and keep holding RESET button of the SoftRF device ; <br>

5. Press and keep holding BOOT button ; <br>

6. Release RESET button first, then the BOOT one ; <br>

7. Execute the tool; Select **ESP32-S3** Chip Type, **Develop** WorkMode and **USB** LoadMode ; <br>

8. Select COM port, enter bootloader file name and address, select options ; <br>

   Here is an example:<br>

![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/Prime3-9.jpg)



9. Press **START** button and wait for completion.

"Stock" devices may also require to apply full flash memory erase (use **ERASE** UI "button") prior to first flashing with bootloader.

## Partition table

Name|Type|SubType|Offset|Size|Notes
---|---|---|---|---|---
nvs|data|nvs|0x9000|20K
otadata|data|ota|0xe000|8K
ota_0|0|ota_0|0x10000|2048K
ota_1|0|ota_1|0x210000|2048K
uf2|app|factory|0x410000|256K
ffat|data|fat|0x450000|2752K|SoftRF, CircuitPython, other
spiffs|data|spiffs|0x700000|1024K|Meshtastic
