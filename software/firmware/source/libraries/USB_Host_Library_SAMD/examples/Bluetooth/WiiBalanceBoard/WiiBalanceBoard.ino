/*
 Example sketch for the Wii Balance Board Bluetooth library - developed by Kristian Lauszus
 For more information visit my blog: http://blog.tkjelectronics.dk/ or
 send me an e-mail:  kristianl@tkjelectronics.com
 */

#include <Wii.h>
#include <usbhub.h>

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
WII Wii(&Btd, PAIR); // This will start an inquiry and then pair with your Wii Balance Board - you only have to do this once
//WII Wii(&Btd); // After that you can simply create the instance like so and then press the power button on the Wii Balance Board

void setup() {
  SerialDebug.begin(115200);
  if (UsbH.Init()) {
    SerialDebug.print(F("\r\nUSB host did not start"));
    while (1); //halt
  }
  SerialDebug.print(F("\r\nWii Balance Board Bluetooth Library Started"));
}
void loop() {
  UsbH.Task();
  if (Wii.wiiBalanceBoardConnected) {
    SerialDebug.print(F("\r\nWeight: "));
    for (uint8_t i = 0; i < 4; i++) {
      SerialDebug.print(Wii.getWeight((BalanceBoardEnum)i));
      SerialDebug.print(F("\t"));
    }
    SerialDebug.print(F("Total Weight: "));
    SerialDebug.print(Wii.getTotalWeight());
    if (Wii.getButtonClick(A)) {
      SerialDebug.print(F("\r\nA"));
      //Wii.setLedToggle(LED1); // The Wii Balance Board has one LED as well
      Wii.disconnect();
    }
  }
}
