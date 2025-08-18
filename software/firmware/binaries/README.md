# First-time firmware installation procedure

- [NodeMCU](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#nodemcu) (ESP8266)
- [ESP32](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#esp32)
- [ESP32-S3](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#esp32-s3)
- [ESP32-S2](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#esp32-s2)
- [ESP32-C3](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#esp32-c3)
- [ESP32-C6](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#esp32-c6)
- [ESP32-P4](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#esp32-p4)
- [S76G](https://github.com/lyusupov/SoftRF/wiki/AcSiP-S7xG-flashing-instructions#s76g) (STM32L073)
- [CC1352R](https://github.com/lyusupov/SoftRF/wiki/Uni-Edition.-Firmware-maintenance-procedures#initial-installation)
- [CubeCell](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#cubecell) (ASR650x)
- [nRF52840](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#nrf52840)
- [LPC4320](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#lpc4320)
- [ASR6601](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#asr6601)
- [RP2040](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#rp2040)
- [RP2350](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#rp2350)
- [STM32WLE5](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#stm32wle5)
- [CH32](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#ch32)

## List by Model

Model|Instructions|Firmware folder
---|:---:|:---:
[Standalone Edition](https://github.com/lyusupov/SoftRF/wiki/Standalone-Edition)|[NodeMCU](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#nodemcu)<br>[ESP32-C3](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#esp32-c3)<br>[ESP32-C6](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#esp32-c6)<br>[RP2350](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#rp2350)|[NodeMCU](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/NodeMCU/SoftRF)<br>[ESP32C3](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/ESP32C3/SoftRF)<br>[ESP32C6](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/ESP32C6/SoftRF)<br>[RP2350](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/RP2350/SoftRF)
[Prime Edition Mk3](https://github.com/lyusupov/SoftRF/wiki/Prime-Edition-MkIII)|[ESP32-S3](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#esp32-s3)|[ESP32S3](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/ESP32S3/SoftRF/MassStorage)
[Ham Edition](https://github.com/lyusupov/SoftRF/wiki/Ham-Edition)|[ESP32-S3](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#esp32-s3)|[ESP32S3](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/ESP32S3/SoftRF/MassStorage)
[Midi Edition](https://github.com/lyusupov/SoftRF/wiki/Midi-Edition)|[ESP32-S3](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#esp32-s3)|[ESP32S3](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/ESP32S3/SoftRF/MassStorage)
[Gizmo Edition](https://github.com/lyusupov/SoftRF/wiki/Gizmo-Edition)&nbsp;![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/new-icon.jpg)|[Web Flasher](https://github.com/lyusupov/SoftRF/wiki/Gizmo-Edition#quick-start)|[ESP32S3](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/ESP32S3/SoftRF/OTA)
[Nano Edition](https://github.com/lyusupov/SoftRF/wiki/Nano-Edition)&nbsp;![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/new-icon.jpg)|[ESP32-C3](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#esp32-c3)|[ESP32C3](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/ESP32C3/SoftRF)
[Badge Edition](https://github.com/lyusupov/SoftRF/wiki/Badge-Edition)|[nRF52840](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#nrf52840)|[nRF52840](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/nRF52840)
[Handheld Edition](https://github.com/lyusupov/SoftRF/wiki/Handheld-Edition)&nbsp;![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/new-icon.jpg)|[nRF52840](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#nrf52840)|[nRF52840](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/nRF52840)
[Card Edition](https://github.com/lyusupov/SoftRF/wiki/Card-Edition)&nbsp;![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/new-icon.jpg)|[nRF52840](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#nrf52840)|[nRF52840](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/nRF52840)
[Cozy Edition](https://github.com/lyusupov/SoftRF/wiki/Cozy-Edition)&nbsp;![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/new-icon.jpg)|[nRF52840](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#nrf52840)|[nRF52840](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/nRF52840)
[Dongle Edition](https://github.com/lyusupov/SoftRF/wiki/Dongle-Edition)|[S76G](https://github.com/lyusupov/SoftRF/wiki/AcSiP-S7xG-flashing-instructions#s76g)|[S76G](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/STM32/S76G)
[ES Edition](https://github.com/lyusupov/SoftRF/wiki/ES-Edition)|[LPC4320](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#lpc4320)|[LPC4320](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/LPC4320)
[Lego Edition](https://github.com/lyusupov/SoftRF/wiki/Lego-Edition)|[RP2040](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#rp2040)|[RP2040](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/RP2040/SoftRF)
[Balkan Edition](https://github.com/lyusupov/SoftRF/wiki/Balkan-Edition)|[STM32WLE5](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#stm32wle5)|[STM32WLE5](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/STM32/WLE5)
[Uni Edition](https://github.com/lyusupov/SoftRF/wiki/Uni-Edition)|[CC1352R](https://github.com/lyusupov/SoftRF/wiki/Uni-Edition.-Firmware-maintenance-procedures#initial-installation)|[CC1352](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/CC13XX/CC1352)
[Mini Edition](https://github.com/lyusupov/SoftRF/wiki/Mini-Edition)|[CubeCell](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#cubecell)|[ASR650x](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/ASR650x)
[Octave Concept](https://github.com/lyusupov/SoftRF/wiki/Octave-Concept)|[ASR6601](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#asr6601)|[ASR6601](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/ASR6601)
[Raspberry Edition](https://github.com/lyusupov/SoftRF/wiki/Raspberry-Edition)||[RaspberryPi](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/RaspberryPi)
[UAV Edition](https://github.com/lyusupov/SoftRF/wiki/UAV-Edition)|[NodeMCU](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#nodemcu)|[NodeMCU](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/NodeMCU/SoftRF)
[Academy Edition](https://github.com/lyusupov/SoftRF/wiki/Academy-Edition)|N/A<sup>1</sup>|N/A<sup>1</sup>
[Eco Edition](https://github.com/lyusupov/SoftRF/wiki/Eco-Edition)&nbsp;![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/new-icon.jpg)|N/A<sup>2</sup>|N/A<sup>2</sup>

<sup>1</sup> - since primary purpose of the **Academy Edition** is education - we do not provide firmware binaries for this model. Students and a teacher are responsible to [build and install the SoftRF firmware](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/source/README.md) ( [SAMD21](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source#samd21) or [RA4M1](https://github.com/lyusupov/SoftRF/edit/master/software/firmware/source#ra4m1) ) from source code by themselves ;<br>
<sup>2</sup> - users of **Eco Edition** should build and install the SoftRF firmware from source code as well.<br>
<br>

Model|Instructions|Firmware folder
---|:---:|:---:
[SkyView EZ](https://github.com/lyusupov/SoftRF/wiki/SkyView-EZ)|[ESP32](https://github.com/lyusupov/SoftRF/wiki/SkyView.-Quick-start)|[ESP32](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/ESP32/SkyView)
[SkyView Pico](https://github.com/lyusupov/SoftRF/wiki/SkyView-Pico)|[RP2040](https://github.com/lyusupov/SoftRF/wiki/SkyView-Pico.-Quick-start#raspberry-pico-w)<br>[RP2350](https://github.com/lyusupov/SoftRF/wiki/SkyView-Pico.-Quick-start#raspberry-pico-w)<br>[ESP32-S3](https://github.com/lyusupov/SoftRF/wiki/SkyView-Pico.-Quick-start#banana-bpi-picow-s3)|[RP2040](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/RP2040/SkyView)<br>[RP2350](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/RP2350/SkyView)<br>[ESP32S3](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/ESP32S3/SkyView)
[WebTop Serial](https://github.com/lyusupov/SoftRF/wiki/WebTop-Serial-adapter)|[NodeMCU](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries#nodemcu)|[NodeMCU](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/NodeMCU/WebTop)
[WebTop USB](https://github.com/lyusupov/SoftRF/wiki/WebTop-USB)|[ESP32-S2](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#esp32-s2)|[ESP32S2](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/ESP32S2/WebTop)
[USB to Bluetooth](https://github.com/lyusupov/SoftRF/wiki/USB-to-Bluetooth-adapter)|[XIAO](https://github.com/lyusupov/SoftRF/wiki/USB-to-Bluetooth-adapter#quick-start)|[SAMD21](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/SAMD21/USB_Bluetooth)
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

1. Take ESP32 flash download tool from this location: http://www.espressif.com/en/support/download/other-tools <br>
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
"Stock" modules may also require to apply full flash memory erase (use **ERASE** UI "button") prior to first flashing with SoftRF's firmware.<br>
One may need to reduce BAUD rate down to 115200 bps.

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

The T-Echo board typically comes with factory pre-installed [Adafruit_nRF52_Bootloader](https://github.com/adafruit/Adafruit_nRF52_Bootloader).<br>
The Bootloader is capable to self-program an application firmware into the device. In order to simplify the firmware transfer, the bootloader emulates a "USB Mass Storage" interface.

1. Download an appropriate version of SoftRF firmware from [this location](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/nRF52840/SoftRF/MassStorage) ;

2. Connect the SoftRF Badge Edition device to your PC by means of a USB cable (Type-A <-> Type-C) ;

3. Double click (within 0.5 seconds) onto the SoftRF device RESET button _(LilyGO T-Echo, Heltec T114 or Elecrow M1)_ or connect twice (within 0.5 seconds) the USB cable while holding the device button been pressed _(Seeed T1000-E)_.<br>

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;A virtual disk with a device specific label should appear in your "File manager" afterwards.<br>

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;For **LilyGO T-Echo** the label is &nbsp; **TECHOBOOT** (or **NRF52BOOT**)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;For **Seeed T1000-E** &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; **T1000-E**<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;For **Heltec T114** &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; **HT-n5262**<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;For **Elecrow M1** &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; **ELECROWBOOT**<br>

4. Drag the downloaded firmware file by your pointing device (mouse, trackball,...) , then drop it into **TECHOBOOT** / **NRF52BOOT** / **T1000-E** / **HT-n5262** / **ELECROWBOOT** disk. Wait until the file transfer is complete.

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
2. Take a copy of **tremo_loader.py** script from [ASR SDK](https://github.com/asrlora/asr_lora_6601/raw/master/build/scripts/tremo_loader.py) ;
3. Connect an appropriate USB-Serial adapter to ASR6601 MCU pins as follows:

Adapter|MCU
---|---
GND|GND
3V3|VCC
TX|GPIO16
RX|GPIO17
DTR&nbsp;<sup>3</sup>|GPIO2
RTS|RESET

<sup>3</sup> - certain USB-Serial adapters may require to **invert DTR** signal either in hardware or in the Python script.<br>

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

1. Download an appropriate version of SoftRF firmware from [this location](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/RP2040/SoftRF) ;

2. Plug RAK11310 WisBlock Core module into RAK5005-O WisBlock Base ;

3. Connect the RAK5005-O to your PC by means of a USB cable (Type-A <-> micro Type-B) ;

4. Press and keep holding BOOT button of RAK11310 Core module ;

5. Press and release RESET button on RAK5005-O Base ;

6. Release BOOT button of RAK11310 module. A virtual disk with **RPI-RP2** label should appear in your "File manager" afterwards ;

7. Drag the downloaded firmware file by your pointing device (mouse, trackball,...) , then drop it into **RPI-RP2** disk. Wait until the file transfer is complete.

<br>

<img src="https://github.com/lyusupov/SoftRF/raw/master/documents/images/Lego-14.jpg" height="295" width="800">

<br>

## RP2350

1. Take an appropriate version of the SoftRF firmware from [this location](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/RP2350/SoftRF) ;

2. Un-zip the firmware file been downloaded ;

3. Press and keep holding BOOT button of the Raspberry **Pico 2W** ;

4. Connect the **Pico 2W** to your PC by means of a USB cable (Type-A <-> micro Type-B) ;

5. Release BOOT button of **Pico 2W**. A virtual disk with **RPI-RP2** label should appear in your "File manager" afterwards ;

6. Drag the downloaded firmware file by your pointing device (mouse, trackball,...) , then drop it into **RPI-RP2** disk. Wait until the file transfer is complete ;

<img src="https://github.com/lyusupov/SoftRF/raw/master/documents/images/Lego-14.jpg" width="800">

<br>

## ESP32-S2

1. Take **esptool-v4.4-win64.zip** flash tool for Windows from this location: [https://github.com/espressif/esptool/releases/tag/v4.4](https://github.com/espressif/esptool/releases/tag/v4.4) and unzip the archive ;
2. Download an appropriate version of WebTop firmware from [this location](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/ESP32S2/WebTop) and unzip the archive ;
3. Connect the ESP32-S2 board into spare USB slot of your Windows PC ;
4. Press and keep holding BOOT button of the ESP32-S2 board ;
5. Press and release RESET button on the ESP32-S2 board ;
6. Release BOOT button of the ESP32-S2 board. A virtual COM port should appear in "Device Manager" afterwards ;
7. Execute **essptool.exe** utility as folows :

<br>

![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/WebTop-9.jpg)

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

## ESP32-S3

The T-Beam Supreme board typically comes with factory pre-installed [**TinyUF2** bootloader](https://github.com/adafruit/tinyuf2/tree/master/ports/espressif#readme).<br>
The Bootloader is capable to self-program an application firmware into the device. In order to simplify the firmware transfer, the bootloader emulates a "USB Mass Storage" interface.

1. Download an appropriate version of SoftRF firmware from [this location](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/ESP32S3/SoftRF/MassStorage) and extract the archive ;

2. Connect the SoftRF device to your PC by means of a USB cable (Type-A <-> Type-C) ;

3. Press and release RESET button of the SoftRF device. Immediately (within 1 second) press and release the BOOT button. One should see this message on the OLED display.

&nbsp;&nbsp;&nbsp;&nbsp; For **T-Beam Supreme**:

![](https://github.com/lyusupov/SoftRF/blob/master/documents/images/Prime3-1.jpg)

&nbsp;&nbsp;&nbsp;&nbsp; For **T-TWR Plus**:

<img src="https://github.com/lyusupov/SoftRF/raw/master/documents/images/ham-1.jpg" width="400">

&nbsp;&nbsp;&nbsp;&nbsp; This white LED on the **Heltec Tracker** will start to flicker:

<img src="https://github.com/lyusupov/SoftRF/raw/master/documents/images/midi-2.jpg" width="400">

<br>

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;A virtual disk with a device specific label should appear in your "File manager" afterwards.<br>

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;For **T-Beam Supreme** the label is &nbsp; **TBEAMBOOT**<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;For **T-TWR Plus** &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; **TWRBOOT**<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;For **Heltec Tracker** &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; **HTBOOT**<br>

4. Drag the downloaded .uf2 firmware file by your pointing device (mouse, trackball,...) , then drop it into **TBEAMBOOT** / **TWRBOOT** / **HTBOOT** disk. Wait until the file transfer is complete.

<br>

<img src="https://github.com/lyusupov/SoftRF/raw/master/documents/images/Prime3-3.jpg" height="233" width="800">

<br>

## ESP32-C3

1. Take ESP32 flash download tool from this location: http://www.espressif.com/en/support/download/other-tools <br>

2. Download an appropriate version of SoftRF firmware from [this location](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/ESP32C3/SoftRF) and unzip the archive ; <br>

3. Connect the ESP32-C3 based device to your PC by means of a USB cable ;

4. Execute the tool; Select **ESP32-C3** Chip Type, **Develop** WorkMode and **UART** LoadMode ; <br>

5. Select COM port, enter partition files and addresses, select options ; <br>

   Here is an example:<br>

![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/Standalone-ESP32C3-3.jpg)



6. Press **START** button and wait for completion.

"Stock" devices may also require to apply full flash memory erase (use **ERASE** UI "button") prior to first flashing with SoftRF's firmware.

<br>

## ESP32-C6

1. Take ESP32 flash download tool from this location: http://www.espressif.com/en/support/download/other-tools <br>

2. Download an appropriate version of SoftRF firmware from [this location](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/ESP32C6/SoftRF) and unzip the archive ; <br>

3. Connect the ESP32-C6 based device to your PC by means of a USB cable ;

4. Execute the tool; Select **ESP32-C6** Chip Type, **Develop** WorkMode and **UART** LoadMode ; <br>

5. Select COM port, enter partition files and addresses, select options ; <br>

   Here is an example:<br>

![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/Standalone-ESP32C6-3.jpg)



6. Press **START** button and wait for completion.

"Stock" devices may also require to apply full flash memory erase (use **ERASE** UI "button") prior to first flashing with SoftRF's firmware.

<br>

## ESP32-P4

1. Take ESP32 flash download tool from this location: http://www.espressif.com/en/support/download/other-tools <br>

2. Download an appropriate version of SoftRF firmware from [this location](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/ESP32P4/SoftRF) and unzip the archive ; <br>

3. Connect the ESP32-P4 based device to your PC by means of a USB cable ;

4. Execute the tool; Select **ESP32-P4** Chip Type, **Develop** WorkMode and **UART** LoadMode ; <br>

5. Select COM port, enter partition files and addresses, select options ; <br>

   Here is an example:<br>

![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/Standalone-ESP32P4-2.jpg)



6. Press **START** button and wait for completion.

"Stock" devices may also require to apply full flash memory erase (use **ERASE** UI "button") prior to first flashing with SoftRF's firmware.

<br>

## CH32

1. Take **WCHISPStudio** tool from [this location](https://www.wch-ic.com/downloads/WCHISPTool_Setup_exe.html) ;
2. Install the **WCHISPStudio** tool ;
3. Download an appropriate version of SoftRF firmware from [this location](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/CH32/SoftRF) ;
4. Connect the CH32 based device to your PC by means of a USB cable ;
5. Press and hold **RESET** button, press and hold **BOOT0** button, release **RESET**, release **BOOT0** ;
6. Follow instructions in the operating manual for **WCHISPStudio** tool to complete the firmware installation procedure. Use this picture as an illustration.

<img src="https://github.com/lyusupov/SoftRF/raw/master/documents/images/Academy-9.jpg" width="800">

<br>
