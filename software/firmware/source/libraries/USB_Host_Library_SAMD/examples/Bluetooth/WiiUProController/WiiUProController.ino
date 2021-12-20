/*
 Example sketch for the Wiimote Bluetooth library - developed by Kristian Lauszus
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
WII Wii(&Btd, PAIR); // This will start an inquiry and then pair with your Wiimote - you only have to do this once
//WII Wii(&Btd); // After that you can simply create the instance like so and then press any button on the Wiimote

void setup() {
  SerialDebug.begin(115200);
  if (UsbH.Init()) {
    SerialDebug.print(F("\r\nUSB host did not start"));
    while (1); //halt
  }
  SerialDebug.print(F("\r\nWiimote Bluetooth Library Started"));
}
void loop() {
  UsbH.Task();
  if (Wii.wiiUProControllerConnected) {
    if (Wii.getButtonClick(HOME)) { // You can use getButtonPress to see if the button is held down
      SerialDebug.print(F("\r\nHome"));
      Wii.disconnect();
    }
    else {
      if (Wii.getButtonClick(LEFT)) {
        Wii.setLedOff();
        Wii.setLedOn(LED1);
        SerialDebug.print(F("\r\nLeft"));
      }
      if (Wii.getButtonClick(RIGHT)) {
        Wii.setLedOff();
        Wii.setLedOn(LED3);
        SerialDebug.print(F("\r\nRight"));
      }
      if (Wii.getButtonClick(DOWN)) {
        Wii.setLedOff();
        Wii.setLedOn(LED4);
        SerialDebug.print(F("\r\nDown"));
      }
      if (Wii.getButtonClick(UP)) {
        Wii.setLedOff();
        Wii.setLedOn(LED2);
        SerialDebug.print(F("\r\nUp"));
      }

      if (Wii.getButtonClick(PLUS))
        SerialDebug.print(F("\r\nPlus"));
      if (Wii.getButtonClick(MINUS))
        SerialDebug.print(F("\r\nMinus"));

      if (Wii.getButtonClick(A))
        SerialDebug.print(F("\r\nA"));
      if (Wii.getButtonClick(B)) {
        Wii.setRumbleToggle();
        SerialDebug.print(F("\r\nB"));
      }
      if (Wii.getButtonClick(X))
        SerialDebug.print(F("\r\nX"));
      if (Wii.getButtonClick(Y))
        SerialDebug.print(F("\r\nY"));

      if (Wii.getButtonClick(L))
        SerialDebug.print(F("\r\nL"));
      if (Wii.getButtonClick(R))
        SerialDebug.print(F("\r\nR"));
      if (Wii.getButtonClick(ZL))
        SerialDebug.print(F("\r\nZL"));
      if (Wii.getButtonClick(ZR))
        SerialDebug.print(F("\r\nZR"));
      if (Wii.getButtonClick(L3))
        SerialDebug.print(F("\r\nL3"));
      if (Wii.getButtonClick(R3))
        SerialDebug.print(F("\r\nR3"));
    }
    if (Wii.getAnalogHat(LeftHatX) > 2200 || Wii.getAnalogHat(LeftHatX) < 1800 || Wii.getAnalogHat(LeftHatY) > 2200 || Wii.getAnalogHat(LeftHatY) < 1800 || Wii.getAnalogHat(RightHatX) > 2200 ||  Wii.getAnalogHat(RightHatX) < 1800 || Wii.getAnalogHat(RightHatY) > 2200 || Wii.getAnalogHat(RightHatY) < 1800) {
      SerialDebug.print(F("\r\nLeftHatX: "));
      SerialDebug.print(Wii.getAnalogHat(LeftHatX));
      SerialDebug.print(F("\tLeftHatY: "));
      SerialDebug.print(Wii.getAnalogHat(LeftHatY));
      SerialDebug.print(F("\tRightHatX: "));
      SerialDebug.print(Wii.getAnalogHat(RightHatX));
      SerialDebug.print(F("\tRightHatY: "));
      SerialDebug.print(Wii.getAnalogHat(RightHatY));
    }
  }
}
