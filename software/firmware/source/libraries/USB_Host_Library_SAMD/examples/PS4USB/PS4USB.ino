/*
 Example sketch for the PS4 USB library - developed by Kristian Lauszus
 For more information visit my blog: http://blog.tkjelectronics.dk/ or
 send me an e-mail:  kristianl@tkjelectronics.com
 */

#include <PS4USB.h>

// On SAMD boards where the native USB port is also the serial console, use
// Serial1 for the serial console. This applies to all SAMD boards except for
// Arduino Zero and M0 boards.
#if (USB_VID==0x2341 && defined(ARDUINO_SAMD_ZERO)) || (USB_VID==0x2a03 && defined(ARDUINO_SAM_ZERO))
#define SerialDebug SERIAL_PORT_MONITOR
#else
#define SerialDebug Serial1
#endif

USBHost UsbH;
PS4USB PS4(&UsbH);

bool printAngle, printTouch;
uint8_t oldL2Value, oldR2Value;

void setup() {
  SerialDebug.begin(115200);
  if (UsbH.Init()) {
    SerialDebug.print(F("\r\nUSB host did not start"));
    while (1); // Halt
  }
  SerialDebug.print(F("\r\nPS4 USB Library Started"));
}

void loop() {
  UsbH.Task();

  if (PS4.connected()) {
    if (PS4.getAnalogHat(LeftHatX) > 137 || PS4.getAnalogHat(LeftHatX) < 117 || PS4.getAnalogHat(LeftHatY) > 137 || PS4.getAnalogHat(LeftHatY) < 117 || PS4.getAnalogHat(RightHatX) > 137 || PS4.getAnalogHat(RightHatX) < 117 || PS4.getAnalogHat(RightHatY) > 137 || PS4.getAnalogHat(RightHatY) < 117) {
      SerialDebug.print(F("\r\nLeftHatX: "));
      SerialDebug.print(PS4.getAnalogHat(LeftHatX));
      SerialDebug.print(F("\tLeftHatY: "));
      SerialDebug.print(PS4.getAnalogHat(LeftHatY));
      SerialDebug.print(F("\tRightHatX: "));
      SerialDebug.print(PS4.getAnalogHat(RightHatX));
      SerialDebug.print(F("\tRightHatY: "));
      SerialDebug.print(PS4.getAnalogHat(RightHatY));
    }

    if (PS4.getAnalogButton(L2) || PS4.getAnalogButton(R2)) { // These are the only analog buttons on the PS4 controller
      SerialDebug.print(F("\r\nL2: "));
      SerialDebug.print(PS4.getAnalogButton(L2));
      SerialDebug.print(F("\tR2: "));
      SerialDebug.print(PS4.getAnalogButton(R2));
    }
    if (PS4.getAnalogButton(L2) != oldL2Value || PS4.getAnalogButton(R2) != oldR2Value) // Only write value if it's different
      PS4.setRumbleOn(PS4.getAnalogButton(L2), PS4.getAnalogButton(R2));
    oldL2Value = PS4.getAnalogButton(L2);
    oldR2Value = PS4.getAnalogButton(R2);

    if (PS4.getButtonClick(PS))
      SerialDebug.print(F("\r\nPS"));
    if (PS4.getButtonClick(TRIANGLE)) {
      SerialDebug.print(F("\r\nTriangle"));
      PS4.setRumbleOn(RumbleLow);
    }
    if (PS4.getButtonClick(CIRCLE)) {
      SerialDebug.print(F("\r\nCircle"));
      PS4.setRumbleOn(RumbleHigh);
    }
    if (PS4.getButtonClick(CROSS)) {
      SerialDebug.print(F("\r\nCross"));
      PS4.setLedFlash(10, 10); // Set it to blink rapidly
    }
    if (PS4.getButtonClick(SQUARE)) {
      SerialDebug.print(F("\r\nSquare"));
      PS4.setLedFlash(0, 0); // Turn off blinking
    }

    if (PS4.getButtonClick(UP)) {
      SerialDebug.print(F("\r\nUp"));
      PS4.setLed(Red);
    } if (PS4.getButtonClick(RIGHT)) {
      SerialDebug.print(F("\r\nRight"));
      PS4.setLed(Blue);
    } if (PS4.getButtonClick(DOWN)) {
      SerialDebug.print(F("\r\nDown"));
      PS4.setLed(Yellow);
    } if (PS4.getButtonClick(LEFT)) {
      SerialDebug.print(F("\r\nLeft"));
      PS4.setLed(Green);
    }

    if (PS4.getButtonClick(L1))
      SerialDebug.print(F("\r\nL1"));
    if (PS4.getButtonClick(L3))
      SerialDebug.print(F("\r\nL3"));
    if (PS4.getButtonClick(R1))
      SerialDebug.print(F("\r\nR1"));
    if (PS4.getButtonClick(R3))
      SerialDebug.print(F("\r\nR3"));

    if (PS4.getButtonClick(SHARE))
      SerialDebug.print(F("\r\nShare"));
    if (PS4.getButtonClick(OPTIONS)) {
      SerialDebug.print(F("\r\nOptions"));
      printAngle = !printAngle;
    }
    if (PS4.getButtonClick(TOUCHPAD)) {
      SerialDebug.print(F("\r\nTouchpad"));
      printTouch = !printTouch;
    }

    if (printAngle) { // Print angle calculated using the accelerometer only
      SerialDebug.print(F("\r\nPitch: "));
      SerialDebug.print(PS4.getAngle(Pitch));
      SerialDebug.print(F("\tRoll: "));
      SerialDebug.print(PS4.getAngle(Roll));
    }

    if (printTouch) { // Print the x, y coordinates of the touchpad
      if (PS4.isTouching(0) || PS4.isTouching(1)) // Print newline and carriage return if any of the fingers are touching the touchpad
        SerialDebug.print(F("\r\n"));
      for (uint8_t i = 0; i < 2; i++) { // The touchpad track two fingers
        if (PS4.isTouching(i)) { // Print the position of the finger if it is touching the touchpad
          SerialDebug.print(F("X")); SerialDebug.print(i + 1); SerialDebug.print(F(": "));
          SerialDebug.print(PS4.getX(i));
          SerialDebug.print(F("\tY")); SerialDebug.print(i + 1); SerialDebug.print(F(": "));
          SerialDebug.print(PS4.getY(i));
          SerialDebug.print(F("\t"));
        }
      }
    }
  }
}
