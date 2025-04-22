// RH_STM32WLx.h
//
// Definitions for the ST Microelectronics STM32WLE5JC chip which contains
// a SX1261/1262 LoRa capable radio.
// https://wiki.seeedstudio.com/LoRa_E5_mini/
// https://www.rfsolutions.co.uk/downloads/1537522406DS_SX1261-2_V1.1_SEMTECH.pdf
// https://cdn.sparkfun.com/assets/6/b/5/1/4/SX1262_datasheet.pdf
// https://files.seeedstudio.com/products/317990687/res/LoRa-E5+module+datasheet_V1.0.pdf
// https://forum.seeedstudio.com/t/lora-e5-register-settings-for-oscillators/262635
// file:///home/mikem/Downloads/es0506-stm32wle5xx-stm32wle4xx-device-errata-stmicroelectronics.pdf
// Author: Mike McCauley (mikem@airspayce.com)
// Copyright (C) 2023 Mike McCauley
//

#ifndef RH_STM32WLx_h
#define RH_STM32WLx_h

#include <RH_SX126x.h>

/////////////////////////////////////////////////////////////////////
/*! \class RH_STM32WLx RH_STM32WLx.h <RH_STM32WLx.h>
\brief Driver to send and receive unaddressed, unreliable datagrams via a
SX1261 LoRa capable radio transceiver embedded in a ST Microelectronics STM32WLE5xx and
STM32WLE4xx processors.

\par Overview

The ST Microelectronics STM32WLE5xx and STM32WLE4xx Arm Cortex
processors contain a SX1261 radio with 2 power amplifiers (PAs), a bit
like a combination of a SX1261 and SX1262. There is a dedicated SPI
interface (called the SubGhzSPI that is used only to communicate with
the radio.

The Seeed LoRa-E5-HF and LoRa-E5-LF modules encapsulate a STM32WLE5JC
processor, along with an antenna switch, Temperature Controlled
Crystal Oscillator (TCXO) and some support components. The Seeed
Wio-E5 mini developement board (described below) has a LoRa-E5-HF
along with voltage regulator, USB interface, antenna connector etc.

This class is a subclass of RH_SX126x. See the documentation for that
class for more details about configuring and using this class.

The stmx32wl_* examples provided should compile out of the box on Arduino 2.1 or later
and run with either modules equipped with
LoRa-E4 or on the NUCLEO_WL55JC1. Make sure you select the appropriate board in Arduino.

\par The Seeed Wio-E5 mini

One example of a development board that includes a STM32WLE5xx and
SX1261 radio is the Wio-E5 mini.

The Wio-E5 mini is powered by the ST Microelectronics STM32WLE5JC,
which contains an ARM Cortex-M4 core and Semtech SX126X LoRa capable
radio. The core and radio are enclosed in a sealed Seeed LoRa-E5-HF
module, and which includes an antenna switch.

https://wiki.seeedstudio.com/LoRa_E5_mini/
https://files.seeedstudio.com/products/317990687/res/LoRa-E5+module+datasheet_V1.0.pdf

It comes with a 868 to 915MHz compatible antenna and USB-C cable.

As shipped, the Wio-E5 mini contains firmware that provides a serial AT
command set that allows the device to connect to join a LoRaWAN
network (via a LoRaWAN gateway) using LoRA radio modulation.
https://files.seeedstudio.com/products/317990687/res/LoRa-E5+AT+Command+Specification_V1.0+.pdf

You can plug the WiO E5 into a USB port, and use a serial port
program (gtkterm, putty, Hyperterm or whatever) to communicate (using
AT commands) with the Wio-E5, configure it and then use it to send
LoRaWAN messages to a LoRaWAN server.

However, what we want to do is to upload RadioHead based firmware into
the Wio-E5 and use the built in SX126X radio to communicate via LoRa
radio modulation with other similar LoRA radio nodes.

Note: LoRa is NOT the same thing as LoRaWAN. LoRaWAN is a complete
networking system that uses LoRa radio modulation as the transport
layer. If you want to work with a LoRaWAN network, you should be using
LoRaWAN software and libraries, not RadioHead. LoRa is a much simpler
and lower level transport layer that can he used to send short
messages over significant distances withglow power.

So, we want to upload RadioHead based software (containing the
RadioHead SX126X driver) to the STM32WLE5JC on the Wio-E5. Before we
can do that we have to disable the firmware write protection on the
STM32WLE5JC.

The Wio-E5 (as shipped) AT command firmware is protected by the ARM
processor's Read Out Protection (RDP) byte, meaning you cant read out
the AT command set firmware, nor can you upload new firmware, The RDP
byte has to be reset first.

But CAUTION: resetting the RDP byte will permanently ERASE the
as-shipped AT command firmware, and THERE IS NO WAY TO GET IT BACK. So
make sure thats what you want to do.

In order to be able to upload our own firmware we must reset the RDP
byte from the as-shipped value of 0xBB (Level 1 read protection) to
0xAA (Level 0, no protection). In order to do this you will need:

- Wio-E5 mini as shipped
- STLink/V2 or STLink/V2 emulator (we used the Adafruit 2548
    https://www.adafruit.com/product/2548). Its an emulator (a red cased USB dongle)
- Host computer with USB ports (Linux, Mac or Windows)
- STM32CubeProgrammer software
  https://www.st.com/en/development-tools/stm32cubeprog.html
  2.15.0 or later for Linux Mac or Windows. Registration required.

1. Install STM32CubeProgrammer
2. Wire the STLink/V2 to the Wio-E5 pins with the jumper wires
   provided. This will provide both power and
   communications with the STM32WLE5JC:
   Wio-E5      STLink/V2 (writing on the end shows the pinout)   
   3.3V	       3.3V         
   GND	       GND          
   DIO	       SWDIO        
   CLK	       SWCLK        
3. Plug the STLink/V2 into a USB port. Red light should appear on the STLink/V2.
4. Run STM32CubeProgrammer (appears in Ubuntu Applications menu if installed on Linux with wine)
5. At the top right you should see ST-LINK selected, and below that,
in the ST-LINK Configuration box, the Serial number of your STLink/V2.
6a. If this is the first time you have used the STLink/V2, you will need
   to upgrade its firmware. Click on 'Firmware upgrade' in the ST-LINK
   Configuration box, get the Upgrade dialog.
6b. Click on 'Open in update mode'. If it complains the STlink is not
in DFU mode, unplug and replug the STlink, click on 'Refresh device
list', then 'Open in update mode'
6c Click on Upgrade. Wait 20 seconds. See success.
6d Close the dialog.
7a Press RST reset pin on the Wio-E5 and then immediately:
7b Click on 'Connect' top right of STM32CubeProgrammer. You should see
   some blue progress lines in the Log, maybe finally a red 'Data read
   failed'. Thats OK, because the RDP byte is still set.
8. Click on the 'OB' icon, 3rd from the top in the far left margin.
9. Select Read Out Protection item, see RDP value set to 'BB'. Change to
   'AA', click Apply. See pop-up 'Option Bytes successfully
   programmed'.

Good! The STM32WLE5JC firmware can now be written and read as we need (note
that the factory supplied LoRaWan firmware has now been erased), by
either STM32CubeProgrammer or Arduino IDE (2.1.1 or later) via the
STLink/V2, so leave it connected

In order to use the Arduino IDE to program the Wio-E5, you must
install the stm32duino package using these instructions:
   https://community.st.com/t5/stm32-mcus/stm32-arduino-stm32duino-tutorial/ta-p/49649
We installed and tested with 2.7.1

Leave STLink/V2 connected but close STM32CubeProgrammer
In Arduino IDE: select the following menu options:
Tools -> Board -> STM32 MCU based boards -> LoRa boards
Tools -> Board part number -> LoRa-E5 mini
Tools -> U(S)ART support -> Enabled (generic Serial)
You will then be able to edit and upload directly from Arduino IDE through the STLink/V2
You can connect the Wio-E5 USB-C to your hosts USB port and use
Arduino IDE to monitor the serial output to Serial
(Note: programming does not occur over the USB connection).

There are other options for programming the STM32WLE5JC,
including STM32CubeIDE from ST. We did not test them.

https://www.rfsolutions.co.uk/downloads/1537522406DS_SX1261-2_V1.1_SEMTECH.pdf

Lora-net software:
https://github.com/Lora-net/SWL2001/releases/tag/v3.3.0

\par Interrupts

This driver uses the SubGhzClass class (part of the Arduino stm32duino
board support) to interface with the internal SX1261/2 radio. That
class implements the dedicated radio SPI interface. The radio also has a
dedicated internal pin for the radio interrupt and the SPI device select
pin and the radio reset pin. The SubGhzClass class includes methods
for using all those pins.

Nevertheless it should be possible to use this processor with the
normal external SPI interfaces to also interface with another SPI
based RadioHEad supported radio, in oprder to make a radio gateway
etc.

*/
class RH_STM32WLx : public RH_SX126x
{
public:
    // Contructor
    RH_STM32WLx();

    // Override the init() function becaue we need to adjust some things afterwards to suit this radio module
    virtual bool init();

protected:
    // Override this because waiting is built in to the SUBGhz driver
    virtual bool waitUntilNotBusy() { return true;};
    
    /// Do whatever is necessary to establish the interrupt handler.
    /// This device has special requirements for setting up the interupt handler
    /// through the SUBGHZSPI interface so we override
    bool setupInterruptHandler();
};


#endif

