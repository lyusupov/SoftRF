/*
 Example sketch for the Xbox Wireless Reciver library - developed by Kristian Lauszus
 It supports up to four controllers wirelessly
 For more information see the blog post: http://blog.tkjelectronics.dk/2012/12/xbox-360-receiver-added-to-the-usb-host-library/ or
 send me an e-mail:  kristianl@tkjelectronics.com
 */

#include <XBOXRECV.h>

// On SAMD boards where the native USB port is also the serial console, use
// Serial1 for the serial console. This applies to all SAMD boards except for
// Arduino Zero and M0 boards.
#if (USB_VID==0x2341 && defined(ARDUINO_SAMD_ZERO)) || (USB_VID==0x2a03 && defined(ARDUINO_SAM_ZERO))
#define SerialDebug SERIAL_PORT_MONITOR
#else
#define SerialDebug Serial1
#endif

USBHost UsbH;
XBOXRECV Xbox(&UsbH);

void setup() {
  SerialDebug.begin(115200);
  if (UsbH.Init()) {
    SerialDebug.print(F("\r\nUSB host did not start"));
    while (1); //halt
  }
  SerialDebug.print(F("\r\nXbox Wireless Receiver Library Started"));
}
void loop() {
  UsbH.Task();
  if (Xbox.XboxReceiverConnected) {
    for (uint8_t i = 0; i < 4; i++) {
      if (Xbox.Xbox360Connected[i]) {
        if (Xbox.getButtonPress(L2, i) || Xbox.getButtonPress(R2, i)) {
          SerialDebug.print("L2: ");
          SerialDebug.print(Xbox.getButtonPress(L2, i));
          SerialDebug.print("\tR2: ");
          SerialDebug.println(Xbox.getButtonPress(R2, i));
          Xbox.setRumbleOn(Xbox.getButtonPress(L2, i), Xbox.getButtonPress(R2, i), i);
        }

        if (Xbox.getAnalogHat(LeftHatX, i) > 7500 || Xbox.getAnalogHat(LeftHatX, i) < -7500 || Xbox.getAnalogHat(LeftHatY, i) > 7500 || Xbox.getAnalogHat(LeftHatY, i) < -7500 || Xbox.getAnalogHat(RightHatX, i) > 7500 || Xbox.getAnalogHat(RightHatX, i) < -7500 || Xbox.getAnalogHat(RightHatY, i) > 7500 || Xbox.getAnalogHat(RightHatY, i) < -7500) {
          if (Xbox.getAnalogHat(LeftHatX, i) > 7500 || Xbox.getAnalogHat(LeftHatX, i) < -7500) {
            SerialDebug.print(F("LeftHatX: "));
            SerialDebug.print(Xbox.getAnalogHat(LeftHatX, i));
            SerialDebug.print("\t");
          }
          if (Xbox.getAnalogHat(LeftHatY, i) > 7500 || Xbox.getAnalogHat(LeftHatY, i) < -7500) {
            SerialDebug.print(F("LeftHatY: "));
            SerialDebug.print(Xbox.getAnalogHat(LeftHatY, i));
            SerialDebug.print("\t");
          }
          if (Xbox.getAnalogHat(RightHatX, i) > 7500 || Xbox.getAnalogHat(RightHatX, i) < -7500) {
            SerialDebug.print(F("RightHatX: "));
            SerialDebug.print(Xbox.getAnalogHat(RightHatX, i));
            SerialDebug.print("\t");
          }
          if (Xbox.getAnalogHat(RightHatY, i) > 7500 || Xbox.getAnalogHat(RightHatY, i) < -7500) {
            SerialDebug.print(F("RightHatY: "));
            SerialDebug.print(Xbox.getAnalogHat(RightHatY, i));
          }
          SerialDebug.println();
        }

        if (Xbox.getButtonClick(UP, i)) {
          Xbox.setLedOn(LED1, i);
          SerialDebug.println(F("Up"));
        }
        if (Xbox.getButtonClick(DOWN, i)) {
          Xbox.setLedOn(LED4, i);
          SerialDebug.println(F("Down"));
        }
        if (Xbox.getButtonClick(LEFT, i)) {
          Xbox.setLedOn(LED3, i);
          SerialDebug.println(F("Left"));
        }
        if (Xbox.getButtonClick(RIGHT, i)) {
          Xbox.setLedOn(LED2, i);
          SerialDebug.println(F("Right"));
        }

        if (Xbox.getButtonClick(START, i)) {
          Xbox.setLedMode(ALTERNATING, i);
          SerialDebug.println(F("Start"));
        }
        if (Xbox.getButtonClick(BACK, i)) {
          Xbox.setLedBlink(ALL, i);
          SerialDebug.println(F("Back"));
        }
        if (Xbox.getButtonClick(L3, i))
          SerialDebug.println(F("L3"));
        if (Xbox.getButtonClick(R3, i))
          SerialDebug.println(F("R3"));

        if (Xbox.getButtonClick(L1, i))
          SerialDebug.println(F("L1"));
        if (Xbox.getButtonClick(R1, i))
          SerialDebug.println(F("R1"));
        if (Xbox.getButtonClick(XBOX, i)) {
          Xbox.setLedMode(ROTATING, i);
          SerialDebug.print(F("Xbox (Battery: "));
          SerialDebug.print(Xbox.getBatteryLevel(i)); // The battery level in the range 0-3
          SerialDebug.println(F(")"));
        }
        if (Xbox.getButtonClick(SYNC, i)) {
          SerialDebug.println(F("Sync"));
          Xbox.disconnect(i);
        }

        if (Xbox.getButtonClick(A, i))
          SerialDebug.println(F("A"));
        if (Xbox.getButtonClick(B, i))
          SerialDebug.println(F("B"));
        if (Xbox.getButtonClick(X, i))
          SerialDebug.println(F("X"));
        if (Xbox.getButtonClick(Y, i))
          SerialDebug.println(F("Y"));
      }
    }
  }
}
