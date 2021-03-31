# First-time firmware installation procedure

- [NodeMCU](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#nodemcu) (ESP8266)
- [ESP32](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#esp32)
- [S76G](https://github.com/lyusupov/SoftRF/wiki/AcSiP-S7xG-flashing-instructions#s76g) (STM32)
- [CC1352R](https://github.com/lyusupov/SoftRF/wiki/Uni-Edition.-Firmware-maintenance-procedures#initial-installation)
- [CubeCell](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#cubecell) (ASR650x)
- [nRF52840](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#nrf52840)

## NodeMCU

### Select NodeMCU COM port
![](https://github.com/lyusupov/SoftRF/blob/master/documents/images/NodeMCU-Flasher-1.GIF)



### Select firmware file
![](https://github.com/lyusupov/SoftRF/blob/master/documents/images/NodeMCU-Flasher-2.GIF)



### Start flashing cycle
![](https://github.com/lyusupov/SoftRF/blob/master/documents/images/NodeMCU-Flasher-3.GIF)



### Wait for completion
![](https://github.com/lyusupov/SoftRF/blob/master/documents/images/NodeMCU-Flasher-4.GIF)

## ESP32

1. Take ESP32 flash download tool from this location: https://www.espressif.com/en/support/download/other-tools <br>
You might also need to install a [driver for the CP210X USB to UART bridge from Silicon Labs](https://www.silabs.com/products/development-tools/software/usb-to-uart-bridge-vcp-drivers) prior to first use of the ESP32 tool ;
2. Download appropriate version of the SoftRF firmware from [this location](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/ESP32) and unzip the archive ; 

3. Select COM port, enter partition files and addresses, select options ;<br>

   Here is an example:<br>

![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/ESP32-Flasher-1.JPG)



4. Press **START** button and wait for completion.

For some boards you may need to push **BOOT** button in order to activate flash download mode.<br>
"Stock" modules may also require to apply full flash memory erase (use **ERASE** UI "button") prior to first flashing with SoftRF's firmware.    

<br>

## CubeCell

1. Take **CubeCellflash** for Windows <sub>_(Linux and MacOS variants are also available there)_</sub> download tool from this location: https://resource.heltec.cn/download/<br>
You might also need to install a [driver for the CP210X USB to UART bridge from Silicon Labs](https://www.silabs.com/products/development-tools/software/usb-to-uart-bridge-vcp-drivers) prior to first use of the CubeCellflash tool ;
2. Download appropriate version of the SoftRF firmware from [this location](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/ASR650x) ;

![](https://github.com/lyusupov/SoftRF/blob/master/documents/images/Mini-1.jpg)

3. Use Windows command line tool to execute firmware flashing procedure as follows:

![](https://github.com/lyusupov/SoftRF/blob/master/documents/images/Mini-2.jpg)

<br>

## nRF52840

The board typically comes with factory pre-installed [Adafruit_nRF52_Bootloader](https://github.com/adafruit/Adafruit_nRF52_Bootloader).<br>
The Bootloader is capable to self-program an application firmware into the device. In order to simplify the firmware transfer, the bootloader emulates a "USB Mass Storage" interface.

1. Download appropriate version of the SoftRF firmware from [this location](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/nRF52840/SoftRF/MassStorage) ;

2. Connect the SoftRF Badge Edition device to your PC by means of a USB cable (Type-A <-> Type-C) ;

3. Double click (within 0.5 seconds) onto the SoftRF device RESET button. A virtual disk with **NRF52BOOT** label should appear in your "File manager" afterwards.

4. Drag the downloaded firmware file by your pointing device (mouse, trackball,...) , then drop it into **NRF52BOOT** disk. Wait until the file transfer is complete.

<br>

<img src="https://github.com/lyusupov/SoftRF/raw/master/documents/images/Badge-2.jpg" height="302" width="800">
