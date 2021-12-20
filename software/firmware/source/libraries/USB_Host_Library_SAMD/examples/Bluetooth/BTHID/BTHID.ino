/*
 Example sketch for the HID Bluetooth library - developed by Kristian Lauszus
 For more information visit my blog: http://blog.tkjelectronics.dk/ or
 send me an e-mail:  kristianl@tkjelectronics.com
 */

#include <BTHID.h>
#include <usbhub.h>
#include "KeyboardParser.h"
#include "MouseParser.h"

// On SAMD boards where the native USB port is also the serial console, use
// Serial1 for the serial console. This applies to all SAMD boards except for
// Arduino Zero and M0 boards.
#if (USB_VID==0x2341 && defined(ARDUINO_SAMD_ZERO)) || (USB_VID==0x2a03 && defined(ARDUINO_SAM_ZERO))
#define SerialDebug SERIAL_PORT_MONITOR
#else
#define SerialDebug Serial1
#endif

USBHost UsbH;
//USBHub Hub1(&UsbH); // Some dongles have a hub inside
BTD Btd(&UsbH); // You have to create the Bluetooth Dongle instance like so

/* You can create the instance of the class in two ways */
// This will start an inquiry and then pair with your device - you only have to do this once
// If you are using a Bluetooth keyboard, then you should type in the password on the keypad and then press enter
BTHID bthid(&Btd, PAIR, "0000");

// After that you can simply create the instance like so and then press any button on the device
//BTHID hid(&Btd);

KbdRptParser keyboardPrs;
MouseRptParser mousePrs;

void setup() {
  SerialDebug.begin(115200);
  if (UsbH.Init()) {
    SerialDebug.print(F("\r\nUSB host did not start"));
    while (1); // Halt
  }

  bthid.SetReportParser(KEYBOARD_PARSER_ID, &keyboardPrs);
  bthid.SetReportParser(MOUSE_PARSER_ID, &mousePrs);

  // If "Boot Protocol Mode" does not work, then try "Report Protocol Mode"
  // If that does not work either, then uncomment PRINTREPORT in BTHID.cpp to see the raw report
  bthid.setProtocolMode(HID_BOOT_PROTOCOL); // Boot Protocol Mode
  //bthid.setProtocolMode(HID_RPT_PROTOCOL); // Report Protocol Mode

  SerialDebug.print(F("\r\nHID Bluetooth Library Started"));
}
void loop() {
  UsbH.Task();
}
