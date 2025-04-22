# Release notes

## revision 0.14

### New features

One more of [hardware platforms](https://github.com/lyusupov/SoftRF#by-processing-unit) (SoCs) that the **SkyView** technology is able to operate on:
* Raspberry Pi Foundation [**RP2350**](https://en.wikipedia.org/wiki/RP2350) &nbsp; - dual _ARM [Cortex-M33](https://en.wikipedia.org/wiki/ARM_Cortex-M33) cores_ and/or _Hazard3 [RISC-V](https://en.wikipedia.org/wiki/RISC-V) cores_

### Raspberry Pi RP2350

* very first Release of [**SkyView Pico**](https://github.com/lyusupov/SoftRF/wiki/SkyView-Pico) for RP2350
* base component: **Raspberry Pico 2W** ![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/new-icon.jpg)

<img src="https://github.com/lyusupov/SoftRF/raw/master/documents/images/skyview-57.jpg" width="500">

### Known issues and limitations

* same that previous 0.13 release has

> [!IMPORTANT]
> Release 0.14 firmware **binaries are available for RP2350** target only.<br>
> Users of other hardware platforms should stay on Release 0.13 because no changes were made to the software except a few that are RP2350 specific ones.

<br>

## revision 0.13

### New features

Three more of [hardware platforms](https://github.com/lyusupov/SoftRF#by-processing-unit) (SoCs) that the **SkyView** technology is able to operate on:
* Raspberry Pi Foundation [**RP2040**](https://en.wikipedia.org/wiki/RP2040) &nbsp; - dual _ARM [Cortex-M0+](https://en.wikipedia.org/wiki/ARM_Cortex-M#Cortex-M0+) cores @ 133 MHz_
* Espressif &nbsp; &nbsp; &nbsp;  &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; [ESP32-**S3**](https://en.wikipedia.org/wiki/ESP32#ESP32-S3)&nbsp; - dual-core Xtensa LX7 with 2.4 GHz radio
* Espressif &nbsp; &nbsp; &nbsp;  &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; [ESP32-**C3**](https://en.wikipedia.org/wiki/ESP32#ESP32-C3)&nbsp; - single [RISC-V](https://en.wikipedia.org/wiki/RISC-V) core with 2.4 GHz radio

### Raspberry Pi RP2040

* very first Release of [**SkyView Pico**](https://github.com/lyusupov/SoftRF/wiki/SkyView-Pico) for RP2040 ![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/new-icon.jpg)
* base component: [**Raspberry Pico W**](https://s.click.aliexpress.com/e/_DD36aMv) ![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/new-icon.jpg)

![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/skyview-47.jpg)

### Espressif ESP32-S3

* very first Release of [**SkyView Pico**](https://github.com/lyusupov/SoftRF/wiki/SkyView-Pico#alternative-hardware-option) for ESP32-S3 ![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/new-icon.jpg)
* base component: [**Banana PicoW-S3**](https://s.click.aliexpress.com/e/_DFRHYMN) ![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/new-icon.jpg)

![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/skyview-56.jpg)

### Major improvements

#### Common

* GxEPD2 driver for GDEY027T91 e-Paper display
* an option to show aircraft type (class) in 'Text View' mode
* screen rotation feature
* Screen Saver feature _(backported from Badge)_
* print version number on boot screen
* indicate on e-paper display when battery voltage value is not available
* print connection settings into boot console
* let UART to be default input across all the platforms
* a fix for #139

#### ESP32

* builds with either Arduino Core **1.0.5** or Arduino Core **2.0.9**
* swap Up and Down buttons when screen is rotated at 180 degrees
* add the MODE button state guard upon shutdown
* update of EPD detect logic

#### ESP32-S2

* made build possible for ESP32-S2 target

#### ESP32-C3

* made build possible for ESP32-C3 target
* bringup on ESP32-C3 target is complete

#### ESP32-S3

* aircrafts database with USB mass storage interface
* support for Waveshare UPS-B voltage monitor
* PDM audio output over I2S0
* use of an audio library
* make the V2 version of Waveshare e-Paper Pico adapter to be the default one for ESP32-S3 targets
* status LED logic
* few more SkyView boot log messages
* wake SkyView up by RTC GPIO input signal
<!-- * make CS4344 external I2S DAC _(Waveshare Pico-Audio Rev. 2.1)_ to be default one for SkyView Pico -->

#### RP2040

* FATFS support for settings (JSON) and aircrafts data (uCDB)
* basic USB I/O operations
* PIO USB CDC Host function
* Bluetooth SPP 'master' role
* HM-10 compatible Bluetooth LE 'central' role
* support for 'version 2' of Waveshare e-paper adapter _(has GDEY027T91 display)_
* made the SkyView compatible with HC-05 BT SPP data source
* fix for EPD detect logic
* support for Waveshare UPS-B voltage monitor
* PWM audio output
* make the V2 version of Waveshare e-Paper Pico adapter to be the default one for RP2040 targets
* activate Wi-Fi power saving feature
* status LED logic
* remap of 'Mode' button from left to center
<!-- * cleanup of SkyView I2S audio logic -->

#### BCM283X

* support for Waveshare 2.7 inch e-Paper HAT V2 adapter _(uses GDEY027T91 display)_

### Known issues and limitations

* same that previous 0.12 release has, **plus**
* **RP2040:** new PIO USB Host feature is a kind of fragile and may cause issues when Wi-Fi or Bluetooth is active. When USB is your primary connection method - activate Wi-Fi 'power saving' feature to disable the Wi-Fi while it is not in use. When you use Wi-Fi or Bluetooth as a primary connection method - do NOT connect any device to the RP2040 PIO USB port ;
* **ESP32-S3**: picture on the GDEY027T91 display may partially loose contrast after a sequence of screen updates. One should consider to activate [e-Paper 'ghosts' removal](https://github.com/lyusupov/SoftRF/wiki/SkyView-settings#e-paper-ghosts-removal) feature as a workaround.

### Flashing instructions

&nbsp;&nbsp;&nbsp;&nbsp;**ESP32:**<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;'[Web Update](https://github.com/lyusupov/SoftRF/wiki/Firmware-update-%28Web-method%29#esp32)' method should work just fine when you are updating from **Regular** 0.12.<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;In case of troubles - use generic ('cable') method instead. Follow **Step 1** and **Step 2** of this [**Quick start**](https://github.com/lyusupov/SoftRF/wiki/SkyView.-Quick-start) guidance.<br>
&nbsp;&nbsp;&nbsp;&nbsp;**RP2040:**<br>
&nbsp;&nbsp;&nbsp;&nbsp;**ESP32-S3:**<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Both RP2040 and ESP32-S3 platforms can do firmware update over USB connection when the firmware binary is in UF2 format.<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Follow **SkyView Pico** firmware installation procedure as explained on [this page](https://github.com/lyusupov/SoftRF/wiki/SkyView-Pico.-Quick-start).<br>

<br>

## revision 0.12

### Major improvements

#### ESP32

- rebuilt in conjunction with most recent [Arduino Core 1.0.5](https://github.com/espressif/arduino-esp32/releases/tag/1.0.5). More stable and less bugs in:
    - Wi-Fi
    - Bluetooth SPP
    - Bluetooth LE

### Known issues and limitations

- same as in 0.11

### Flashing instructions

&nbsp;&nbsp;&nbsp;&nbsp;**Regular:**<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;'[Web Update](https://github.com/lyusupov/SoftRF/wiki/Firmware-update-%28Web-method%29#esp32)' method should work just fine when you are updating from **Regular** 0.11.<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;In case of troubles - use generic ('cable') method instead. Follow **Step 1** and **Step 2** of this [**Quick start**](https://github.com/lyusupov/SoftRF/wiki/SkyView.-Quick-start) guidance.

## revision 0.11

### Major improvements

#### Common

- improved traffic acceptor logic
- improved voice alerts logic
- fix for remote (>20km) traffic processing
- work around of an issue when ownship (pressure) altitude is not reported in GDL90 input data

#### ESP32

- confirmed to work nicely with version 2.8 of TTGO T5S
- sends proper NMEA sentences to put connected (by wires) SoftRF Dongle/Uni/Mini into sleep or wake it up
- battery current consumption in 'sleep' mode has been reduced 10-100 times down to: uSD in - 0.2 mA, uSD out - 0.1 mA
- better response on buttons press (except when a voice alert is in progress)
- improved buffering and response time on input data flow
- Bluetooth LE connection method works better now

### Known issues and limitations

- same as in 0.10

### Flashing instructions

&nbsp;&nbsp;&nbsp;&nbsp;**Regular:**<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;'[Web Update](https://github.com/lyusupov/SoftRF/wiki/Firmware-update-%28Web-method%29#esp32)' method should work just fine when you are updating from **Regular** 0.10.<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;In case of troubles - use generic ('cable') method instead. Follow **Step 1** and **Step 2** of this [**Quick start**](https://github.com/lyusupov/SoftRF/wiki/SkyView.-Quick-start) guidance.

## revision 0.10

### New features

- Bluetooth SPP and LE connection methods. Mostly applicable for a 'SoftRF on ESP32' partner - other devices may or may not work ;
- serial data input over EZ built-in micro-USB port ;
- one more radar view's zoom level ;
- 'Wi-Fi off' timer option ;
- traffic filter (by altitude) ;
- team member's aircraft (if any) is depicted in a bit different manner ;
- dual boot option ( demo application is [iArradio](https://github.com/TioRuben/iArradio) ).

### Known issues and limitations

- same as in 0.9, plus
- first Bluetooth SPP connection may cause restart of a partner SoftRF data source device ;
- due to high RAM memory usage, Bluetooth SPP may coexist with either voice or aircraft's data option but not both ;
- only Bluetooth 'Simple Pairing' (no key) method is currently supported.

### Flashing instructions

&nbsp;&nbsp;&nbsp;&nbsp;**Regular:**<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;'Web Update' UI feature does not work in previous 0.9 revision. Use generic ('cable') method instead.<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Follow **Step 1** and **Step 2** of this [**Quick start**](https://github.com/lyusupov/SoftRF/wiki/SkyView.-Quick-start) guidance.

&nbsp;&nbsp;&nbsp;&nbsp;**Dual boot:**<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Follow [these instructions](https://github.com/lyusupov/SoftRF/wiki/SkyView.-Dual-boot).<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;**NOTICE!**: [**iArradio**](https://github.com/TioRuben/iArradio) firmware that comes with it is a DEMO. I will reject your every claim or question.<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;If you need a custom weather, playlist or timezone - build your own firmware file by yourself.<br>

## revision 0.9

### Known issues and limitations

- maximum number of tracked objects is limited to 9 ;
- very short keypress may not work. Make a more distinct one ;
- keypress may not work when audio is playing ;
- view mode change may cause 2-3 seconds of '**NO DATA**' or '**NO FIX**' warning ;
- 'anti-ghosting' full screen refresh may cause 2-3 seconds of '**NO DATA**' or '**NO FIX**' warning ;
- '**NO DATA**' or '**NO FIX**' warning may appear right after voice traffic alert message ;
- **VOICE2** and **VOICE3** may have some WAV files missing ;
- Wi-Fi re-connect may fail sometimes. Reset of SoftRF server and/or SkyView client does typically help ;
- does not work with Stratux yet due to issues with DHCP leases and GDL90 data (over-)flow.

## revision 0.8

Very first deployment of SkyView's firmware binaries.
