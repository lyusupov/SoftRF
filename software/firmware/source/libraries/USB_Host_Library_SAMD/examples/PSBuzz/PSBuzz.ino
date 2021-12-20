/*
 Example sketch for the Playstation Buzz library - developed by Kristian Lauszus
 For more information visit my blog: http://blog.tkjelectronics.dk/ or
 send me an e-mail:  kristianl@tkjelectronics.com
 */

#include <PSBuzz.h>

// On SAMD boards where the native USB port is also the serial console, use
// Serial1 for the serial console. This applies to all SAMD boards except for
// Arduino Zero and M0 boards.
#if (USB_VID==0x2341 && defined(ARDUINO_SAMD_ZERO)) || (USB_VID==0x2a03 && defined(ARDUINO_SAM_ZERO))
#define SerialDebug SERIAL_PORT_MONITOR
#else
#define SerialDebug Serial1
#endif

USBHost UsbH;
PSBuzz Buzz(&UsbH);

void setup() {
  SerialDebug.begin(115200);
  if (UsbH.Init()) {
    SerialDebug.print(F("\r\nUSB host did not start"));
    while (1); // Halt
  }
  SerialDebug.println(F("\r\nPS Buzz Library Started"));
}

void loop() {
  UsbH.Task();

  if (Buzz.connected()) {
    for (uint8_t i = 0; i < 4; i++) {
      if (Buzz.getButtonClick(RED, i)) {
        Buzz.setLedToggle(i); // Toggle the LED
        SerialDebug.println(F("RED"));
      }
      if (Buzz.getButtonClick(YELLOW, i))
        SerialDebug.println(F("YELLOW"));
      if (Buzz.getButtonClick(GREEN, i))
        SerialDebug.println(F("GREEN"));
      if (Buzz.getButtonClick(ORANGE, i))
        SerialDebug.println(F("ORANGE"));
      if (Buzz.getButtonClick(BLUE, i))
        SerialDebug.println(F("BLUE"));
    }
  }
}
