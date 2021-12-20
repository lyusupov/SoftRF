/*
 Example sketch for the original Xbox library - developed by Kristian Lauszus
 For more information visit my blog: http://blog.tkjelectronics.dk/ or
 send me an e-mail:  kristianl@tkjelectronics.com
 */

#include <XBOXOLD.h>
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
USBHub  Hub1(&UsbH); // The controller has a built in hub, so this instance is needed
XBOXOLD Xbox(&UsbH);

void setup() {
  SerialDebug.begin(115200);
  if (UsbH.Init()) {
    SerialDebug.print(F("\r\nUSB host did not start"));
    while (1); // halt
  }
  SerialDebug.print(F("\r\nXBOX Library Started"));
}
void loop() {
  UsbH.Task();
  if (Xbox.XboxConnected) {
    if (Xbox.getButtonPress(BLACK) || Xbox.getButtonPress(WHITE)) {
      SerialDebug.print("BLACK: ");
      SerialDebug.print(Xbox.getButtonPress(BLACK));
      SerialDebug.print("\tWHITE: ");
      SerialDebug.println(Xbox.getButtonPress(WHITE));
      Xbox.setRumbleOn(Xbox.getButtonPress(BLACK), Xbox.getButtonPress(WHITE));
    } else
      Xbox.setRumbleOn(0, 0);

    if (Xbox.getAnalogHat(LeftHatX) > 7500 || Xbox.getAnalogHat(LeftHatX) < -7500 || Xbox.getAnalogHat(LeftHatY) > 7500 || Xbox.getAnalogHat(LeftHatY) < -7500 || Xbox.getAnalogHat(RightHatX) > 7500 || Xbox.getAnalogHat(RightHatX) < -7500 || Xbox.getAnalogHat(RightHatY) > 7500 || Xbox.getAnalogHat(RightHatY) < -7500) {
      if (Xbox.getAnalogHat(LeftHatX) > 7500 || Xbox.getAnalogHat(LeftHatX) < -7500) {
        SerialDebug.print(F("LeftHatX: "));
        SerialDebug.print(Xbox.getAnalogHat(LeftHatX));
        SerialDebug.print("\t");
      }
      if (Xbox.getAnalogHat(LeftHatY) > 7500 || Xbox.getAnalogHat(LeftHatY) < -7500) {
        SerialDebug.print(F("LeftHatY: "));
        SerialDebug.print(Xbox.getAnalogHat(LeftHatY));
        SerialDebug.print("\t");
      }
      if (Xbox.getAnalogHat(RightHatX) > 7500 || Xbox.getAnalogHat(RightHatX) < -7500) {
        SerialDebug.print(F("RightHatX: "));
        SerialDebug.print(Xbox.getAnalogHat(RightHatX));
        SerialDebug.print("\t");
      }
      if (Xbox.getAnalogHat(RightHatY) > 7500 || Xbox.getAnalogHat(RightHatY) < -7500) {
        SerialDebug.print(F("RightHatY: "));
        SerialDebug.print(Xbox.getAnalogHat(RightHatY));
      }
      SerialDebug.println();
    }

    if (Xbox.getButtonClick(UP))
      SerialDebug.println(F("Up"));
    if (Xbox.getButtonClick(DOWN))
      SerialDebug.println(F("Down"));
    if (Xbox.getButtonClick(LEFT))
      SerialDebug.println(F("Left"));
    if (Xbox.getButtonClick(RIGHT))
      SerialDebug.println(F("Right"));

    if (Xbox.getButtonClick(START))
      SerialDebug.println(F("Start"));
    if (Xbox.getButtonClick(BACK))
      SerialDebug.println(F("Back"));
    if (Xbox.getButtonClick(L3))
      SerialDebug.println(F("L3"));
    if (Xbox.getButtonClick(R3))
      SerialDebug.println(F("R3"));

    if (Xbox.getButtonPress(A)) {
      SerialDebug.print(F("A: "));
      SerialDebug.println(Xbox.getButtonPress(A));
    }
    if (Xbox.getButtonPress(B)) {
      SerialDebug.print(F("B: "));
      SerialDebug.println(Xbox.getButtonPress(B));
    }
    if (Xbox.getButtonPress(X)) {
      SerialDebug.print(F("X: "));
      SerialDebug.println(Xbox.getButtonPress(X));
    }
    if (Xbox.getButtonPress(Y)) {
      SerialDebug.print(F("Y: "));
      SerialDebug.println(Xbox.getButtonPress(Y));
    }
    if (Xbox.getButtonPress(L1)) {
      SerialDebug.print(F("L1: "));
      SerialDebug.println(Xbox.getButtonPress(L1));
    }
    if (Xbox.getButtonPress(R1)) {
      SerialDebug.print(F("R1: "));
      SerialDebug.println(Xbox.getButtonPress(R1));
    }
  }
  delay(1);
}
