# First-time firmware installation procedure

- [NodeMCU](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#nodemcu) (ESP8266)
- [ESP32](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#esp32)
- [S76G](https://github.com/lyusupov/SoftRF/wiki/AcSiP-S7xG-flashing-instructions#s76g) (STM32L073)
- [CC1352R](https://github.com/lyusupov/SoftRF/wiki/Uni-Edition.-Firmware-maintenance-procedures#initial-installation)
- [CubeCell](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#cubecell) (ASR650x)
- [nRF52840](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#nrf52840)
- [LPC4320](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#lpc4320)
- [ASR6601](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#asr6601)
- [RP2040](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#rp2040)
- [STM32WLE5](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#stm32wle5)

## List by Model

Model(s)|Instructions|Firmware folder
---|:---:|:---:
Standalone Edition|[NodeMCU](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#nodemcu)|[NodeMCU](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/NodeMCU)
Prime Edition Mk2|[ESP32](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#esp32)|[ESP32](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/ESP32)
Prime Edition Mk3|ESP32S3|ESP32S3
Badge Edition|[nRF52840](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#nrf52840)|[nRF52840](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/nRF52840)
Dongle Edition|[S76G](https://github.com/lyusupov/SoftRF/wiki/AcSiP-S7xG-flashing-instructions#s76g)|[S76G](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/STM32/S76G)
ES Edition|[LPC4320](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#lpc4320)|[LPC4320](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/LPC4320)
Lego Edition|[RP2040](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#rp2040)|[RP2040](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/RP2040)
Balkan Edition|[STM32WLE5](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#stm32wle5)|[STM32WLE5](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/STM32/WLE5)
Uni Edition|[CC1352R](https://github.com/lyusupov/SoftRF/wiki/Uni-Edition.-Firmware-maintenance-procedures#initial-installation)|[CC1352R](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/CC13XX/CC1352)
Mini Edition|[CubeCell](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#cubecell)|[ASR650x](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/ASR650x)
Octave Concept|[ASR6601](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#asr6601)|[ASR6601](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/ASR6601)
<br>

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
You might also need to install:
* a [driver for the CP210X USB to UART bridge from Silicon Labs](https://www.silabs.com/products/development-tools/software/usb-to-uart-bridge-vcp-drivers) or
* [CH9102F driver](https://github.com/Xinyuan-LilyGO/CH9102_Driver)<br>
prior to first use of the ESP32 tool ;
2. Download an appropriate version of SoftRF firmware from [this location](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/ESP32) and unzip the archive ;

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
2. Download an appropriate version of SoftRF firmware from [this location](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/ASR650x) and unzip the archive ;

![](https://github.com/lyusupov/SoftRF/blob/master/documents/images/Mini-1.jpg)

3. Use Windows command line tool to execute firmware flashing procedure as follows:

![](https://github.com/lyusupov/SoftRF/blob/master/documents/images/Mini-2.jpg)

<br>

## nRF52840

The board typically comes with factory pre-installed [Adafruit_nRF52_Bootloader](https://github.com/adafruit/Adafruit_nRF52_Bootloader).<br>
The Bootloader is capable to self-program an application firmware into the device. In order to simplify the firmware transfer, the bootloader emulates a "USB Mass Storage" interface.

1. Download an appropriate version of SoftRF firmware from [this location](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/nRF52840/SoftRF/MassStorage) ;

2. Connect the SoftRF Badge Edition device to your PC by means of a USB cable (Type-A <-> Type-C) ;

3. Double click (within 0.5 seconds) onto the SoftRF device RESET button. A virtual disk with **NRF52BOOT** (or **TECHOBOOT**) label should appear in your "File manager" afterwards.

4. Drag the downloaded firmware file by your pointing device (mouse, trackball,...) , then drop it into **NRF52BOOT** (or **TECHOBOOT**) disk. Wait until the file transfer is complete.

<br>

<img src="https://github.com/lyusupov/SoftRF/raw/master/documents/images/Badge-2.jpg" height="302" width="800">

<br>

## LPC4320

For Linux and Mac OS X users, you will need a few tools installed on your computer before you begin:

* [dfu-util](http://dfu-util.sourceforge.net/) 0.8 or newer - Used to load and run the stock HackRF One firmware from RAM. dfu-util 0.8 is recommended, as it is the most extensively tested with the HackRF One hardware and build software.
* [hackrf](https://github.com/greatscottgadgets/hackrf) - All you need is the host tools, specifically, hackrf_spiflash.

### Backup of factory firmware

```
$ hackrf_spiflash -v -r HackRF_One_factory_firmware.bin
Reading 256 bytes from 0x000000.
Reading 256 bytes from 0x000100.

< ... skipped ... >

$ ls -la HackRF_One_factory_firmware.bin
-rw-r--r-- 1 pi pi 1048576 Nov  4 10:18 HackRF_One_factory_firmware.bin
```

### Flashing

1. Download an appropriate version of SoftRF firmware from [this location](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/LPC4320) ;
2. Plug the HackRF One into a PC USB slot while holding down DFU button (the button closer to antenna jack). Release the DFU button, then execute:

```
$ dfu-util -D hackrf_one_usb.dfu --reset
$ hackrf_spiflash -v -w SoftRF-firmware-v1.1-LPC4320.bin
```

### Restore of HackRF One firmware

Plug the HackRF One into a PC USB slot while holding down DFU button (the button closer to antenna jack). Release the DFU button, then execute:

```
$ dfu-util -D hackrf_one_usb.dfu --reset
$ hackrf_spiflash -v -w HackRF_One_factory_firmware.bin
```

<br>

## ASR6601

1. Download an appropriate version of SoftRF firmware from [this location](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/ASR6601) ;
2. Take a copy of **tremo_loader.py** script from [ASR SDK](https://raw.githubusercontent.com/akarsh98/ASR6601-getting-started-guide/main/SDK/build/scripts/tremo_loader.py) ;
3. Connect an appropriate USB-Serial adapter to ASR6601 MCU pins as follows:

Adapter|MCU
---|---
GND|GND
3V3|VCC
TX|GPIO16
RX|GPIO17
DTR&nbsp;<sup>1</sup>|GPIO2
RTS|RESET

<sup>1</sup> - certain USB-Serial adapters may require to **invert DTR** signal either in hardware or in the Python script.<br>

4. Plug the USB-Serial adapter into spare USB slot of your PC ;
5. Use the loader tool to read the serial number of the MCU. This is a safety action to make sure that all the connections are good ;

```
$ python tremo_loader.py --port /dev/ttyUSB0 read_sn
Connecting...
Connected
The SN is: 0c15458cc5fb3201
```

6. Write the SoftRF firmware binary into flash memory of the ASR6601.

```
$ python tremo_loader.py --port /dev/ttyUSB0 flash 0x08000000 SoftRF-firmware-v1.1-ASR6601.bin
Connecting...
Connected
('send: ', 512)
('send: ', 1024)
('send: ', 1536)
('send: ', 2048)
('send: ', 2560)
('send: ', 3072)

< ... skipped ... >

('send: ', 102400)
('send: ', 102912)
('send: ', 103228)
Download files successfully
```
<br>

## RP2040

Every RAK11310 WisBlock Core module has built-in (ROM) bootloader.<br>
The bootloader is capable to self-program an application firmware into the device. In order to simplify the firmware transfer, the bootloader emulates a "USB Mass Storage" interface.

1. Download an appropriate version of SoftRF firmware from [this location](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/RP2040/) ;

2. Plug RAK11310 WisBlock Core module into RAK5005-O WisBlock Base ;

3. Connect the RAK5005-O to your PC by means of a USB cable (Type-A <-> micro Type-B) ;

4. Press and keep holding BOOT button of RAK11310 Core module ;

5. Press and release RESET button on RAK5005-O Base ;

6. Release BOOT button of RAK11310 module. A virtual disk with **RPI-RP2** label should appear in your "File manager" afterwards ;

7. Drag the downloaded firmware file by your pointing device (mouse, trackball,...) , then drop it into **RPI-RP2** disk. Wait until the file transfer is complete.

<br>

<img src="https://github.com/lyusupov/SoftRF/raw/master/documents/images/Lego-14.jpg" height="295" width="800">

<br>

## STM32WLE5

1. Take pre-built **OpenOCD** tool for Windows from this location: https://gnutoolchains.com/arm-eabi/openocd/ and extract the archive ;
2. Download an appropriate version of SoftRF firmware from [this location](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/STM32/WLE5) ;
3. Connect the SoftRF Balkan Edition device to your PC by means of a USB cable ;
4. Use Windows command line tool to execute firmware flashing procedure as follows:

```
C:\OpenOCD-20211118-0.11.0>.\bin\openocd.exe -f interface/cmsis-dap.cfg -c "cmsis_dap_vid_pid 0x15ba 0x0044" -c "transport select swd" -c "adapter speed 4000" -f "target/stm32wlx.cfg" -c "program SoftRF-firmware-v1.2-WLE5.bin 0x08000000"
```

Example:<br>

![](https://github.com/lyusupov/SoftRF/blob/master/documents/images/Balkan-7.jpg)

<br>
