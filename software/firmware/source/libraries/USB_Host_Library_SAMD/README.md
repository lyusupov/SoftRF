# USB Host Library SAMD
USB Host Library for Arduino SAMD boards

This library is based on the [USB Host Shield Library
2.0](https://github.com/felis/USB_Host_Shield_2.0) with modifications to work
on Arduino and Arduino compatible SAMD boards. Be sure to see the README for
details on Bluetooth pairing, etc. because the information is not repeated
here.

This library would not be possible without the work of the USB Host Shield
Library developers. Thank you!

The example programs should work on Zero, M0, and MKR family on the native USB
port in USB host mode. The USB host shield with MAX3421E is not supported by
this library.

Early versions of this library required some changes to the SAMD board package.
The current version does not, so it works like a regular Arduino library.

![Image of MKR Zero connected to Xbox One controller](./images/mkrxbox.jpg)

## USB Host Co-Processor

The related [USB Host Co-Processor](https://github.com/gdsports/usbhostcopro)
project encapsulates some of the USB host drivers of this project (MIDI and
keyboard so far) in an Adafruit Trinket M0. This allows boards without USB
hardware or without USB host software support to use USB host mode by
connecting via UART to a Trinket M0.

To make using the firmware even easier, the repo includes UF2 binary releases
that can be burned into a Trinket M0 by dragging and dropping the UF2 file on
the TRINKETBOOT USB drive. It is not necessary to install the Arduino IDE,
libraries, or USB serial driver.

## Xbox and MKR board example

Components from left to right:

* USB phone/tablet charger, 5V
* Arduino MKR Zero (but any MKR board should work)
* Adafruit CP2104 USB to UART (but any USB 3.3V serial should work)
* Xbox One controller with batteries

USB OTG to host cable and 2 X USB micro cables are also shown. Search any
shopping website for "usb otg to usb host".

The Xbox controller must have working batteries to ensure the controller works
when both rumble motors are turned on.

When using a MKR board, in the XBOXONE.ino example program, change "Serial." to
"Serial1." to get serial console on the TX/RX pins.

When using USB host mode on MKR boards, the IDE automatic upload does not work.
When the IDE says "Uploading", double click on the MKR board reset button.

Be sure the USB serial board uses 3.3V logic levels on Tx and Rx and 5V on the
power pin.

CP2104  | MKR board
------  | ---------
5V      | VIN
GND     | GND
RXD     | 14<-TX  (Serial1)
TXD     | 13->RX  (Serial1)

The CP2104 board passes through the USB 5V to the MKR VIN power input pin. The
MKR board powers the Xbox controller via its USB port.

## Arduino MKR Boards

The Zero and M0 boards have two USB ports. The programming port is used for
sketch uploads and serial console. The native USB port with a USB OTG to host
cable/adapter is used for the USB device (for example, Xbox controller). When
using a MKR board, the native USB port is used for both functions.

The IDE automatic upload does not work on MKR boards when using USB host. The
solution is simple. Unplug the USB OTG to host cable/adaper, plug in the MKR
board to computer running the IDE as usual. Press the IDE Upload icon. When the
IDE status shows "Uploading", double click the MKR board reset button.

## Adafruit M0 and M4 Boards

The Adafruit Metro M4 (SAMD51) and Trinket M0 (SAMD21) work with this library.
Follow Adafruit's tutorial to install the Adafruit SAMD board package. Other
Arduino compatible (for example Adafruit, SparkFun, etc.) SAMD boards might
work.

![Image of Metro M4 connected to PS3 controller clone via Bluetooth](./images/ps3bt.jpg)

Components starting from the Metro M4 board then moving clockwise.

* Adafruit Metro M4 with USB OTG to host adapter and USB Bluetooth plug
* USB FTDI serial board for serial console and power
* USB phone/tablet charger, 5V
* Playstation 3 controller (clone) with Bluetooth

FTDI    | Metro M4 board
------  | ---------
5V      | VIN
GND     | GND
RXD     | TX-1  (Serial1)
TXD     | RX-0  (Serial1)

## Related projects

* [Xbox Adaptive Controller Joystick Splitter](https://github.com/gdsports/xac-joystick-splitter)
* [MIDI DIN to MIDI USB Host Converter](https://github.com/gdsports/midiuartusbh)
* [Portable MIDI dsp-G1 Synth](https://github.com/gdsports/dspg1)
* [Control DC motor using an Xbox One controller](https://github.com/gdsports/xbox1motor)
* [Control DC motor with PS3 game controller](https://github.com/gdsports/ps3motor)
* [RFM69 Wireless USB Keyboard](https://github.com/gdsports/rfm69-usb-devices)
* [USB Host Co-Processor](https://github.com/gdsports/usbhostcopro)
* [CircuitPython USB Host MIDI](https://github.com/gdsports/circuitpython_usb_host_midi)

## Testing

This is unstable and may break at any time. It works for me but may not work
you. Use At Your Own Risk. Your Mileage May Vary. Batteries Not Included.

I do not have the hardware to test all the drivers. Also I do not have time to
do extensive testing. Hardware I have connected and tested minimally:

* USB keyboard and mouse
* Xbox One USB (genuine)
* Xbox 360 USB (clone)
* PS3 USB (cheap clone)
* PS4 USB (genuine)
* ftdi serial
* pl2303 serial
* CDC ACM serial (Uno, Leonardo, Zero, MKR, CircuitPlayground Express)
* MIDI USB Korg NanoKontrol2
* Bluetooth SPP, BTHID (BT keyboard), and PS3BT (Playstation 3 controller)
* ADK ArduinoBlinkLED
* USB hubs work but not reliable yet
* Logitech Extreme 3D Pro joystick
* Thrustmaster T.16000M FCS joystick
* Dymo M10 postage scale using the scaleEasy example
* USB ESC POS receipt printer (https://github.com/gdsports/USBPrinter_uhls)

## Building the development environment

Early versions of this library required patches to the SAMD board package.
The current version does not, so it works like a regular Arduino library.

The following script works for a Linux system. The enviroment uses the [Arduino
IDE portable](https://www.arduino.cc/en/Guide/PortableIDE) feature to insulate
this project from the standard Arduino directories. This is no longer needed
since it no longer patches the SAMD board package.

The script use the IDE command line interface (CLI) which is documented
[here](https://github.com/arduino/Arduino/blob/master/build/shared/manpage.adoc).


```
IDEVER="1.8.7"
# Change to home directory
cd
# Create work directory
mkdir arduino_samd_usb_host
cd arduino_samd_usb_host
WORKDIR=`pwd`
# Install Ardino IDE in work directory
wget -O arduino.tar.xz https://downloads.arduino.cc/arduino-${IDEVER}-linux64.tar.xz
tar xf arduino.tar.xz -C ${WORKDIR}
rm arduino.tar.xz
# Create portable sketchbook and library directories
# Using portable prevents these changes from affecting other Arduino projects.
IDEDIR="${WORKDIR}/arduino-${IDEVER}"
LIBDIR="${IDEDIR}/portable/sketchbook/libraries"
mkdir -p "${LIBDIR}"
cd ${IDEDIR}
# Install board package
./arduino --pref "compiler.warning_level=default" --save-prefs
./arduino --install-boards "arduino:samd"
BOARD="arduino:samd:arduino_zero_edbg"
./arduino --board "${BOARD}" --save-prefs
# Install MIDI library for USBH_MIDI examples
./arduino --install-library "MIDI Library"
cd ${LIBDIR}
# Install TinyGPS for pl2303 example
git clone https://github.com/mikalhart/TinyGPS.git
# Install USB host library for SAMD
git clone https://github.com/gdsports/USB_Host_Library_SAMD
cd ${IDEDIR}
./arduino &
````
