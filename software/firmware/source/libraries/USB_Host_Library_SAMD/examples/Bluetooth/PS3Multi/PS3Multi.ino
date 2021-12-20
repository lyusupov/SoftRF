/*
 Example sketch for the PS3 Bluetooth library - developed by Kristian Lauszus
 This example show how one can use multiple controllers with the library
 For more information visit my blog: http://blog.tkjelectronics.dk/ or
 send me an e-mail:  kristianl@tkjelectronics.com
 */

#include <PS3BT.h>
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
PS3BT *PS3[2]; // We will use this pointer to store the two instance, you can easily make it larger if you like, but it will use a lot of RAM!
const uint8_t length = sizeof(PS3) / sizeof(PS3[0]); // Get the lenght of the array
bool printAngle[length];
bool oldControllerState[length];

void setup() {
  for (uint8_t i = 0; i < length; i++) {
    PS3[i] = new PS3BT(&Btd); // Create the instances
    PS3[i]->attachOnInit(onInit); // onInit() is called upon a new connection - you can call the function whatever you like
  }

  SerialDebug.begin(115200);
  if (UsbH.Init()) {
    SerialDebug.print(F("\r\nUSB host did not start"));
    while (1); //halt
  }
  SerialDebug.print(F("\r\nPS3 Bluetooth Library Started"));
}
void loop() {
  UsbH.Task();

  for (uint8_t i = 0; i < length; i++) {
    if (PS3[i]->PS3Connected || PS3[i]->PS3NavigationConnected) {
      if (PS3[i]->getAnalogHat(LeftHatX) > 137 || PS3[i]->getAnalogHat(LeftHatX) < 117 || PS3[i]->getAnalogHat(LeftHatY) > 137 || PS3[i]->getAnalogHat(LeftHatY) < 117 || PS3[i]->getAnalogHat(RightHatX) > 137 || PS3[i]->getAnalogHat(RightHatX) < 117 || PS3[i]->getAnalogHat(RightHatY) > 137 || PS3[i]->getAnalogHat(RightHatY) < 117) {
        SerialDebug.print(F("\r\nLeftHatX: "));
        SerialDebug.print(PS3[i]->getAnalogHat(LeftHatX));
        SerialDebug.print(F("\tLeftHatY: "));
        SerialDebug.print(PS3[i]->getAnalogHat(LeftHatY));
        if (PS3[i]->PS3Connected) { // The Navigation controller only have one joystick
          SerialDebug.print(F("\tRightHatX: "));
          SerialDebug.print(PS3[i]->getAnalogHat(RightHatX));
          SerialDebug.print(F("\tRightHatY: "));
          SerialDebug.print(PS3[i]->getAnalogHat(RightHatY));
        }
      }
      //Analog button values can be read from almost all buttons
      if (PS3[i]->getAnalogButton(L2) || PS3[i]->getAnalogButton(R2)) {
        SerialDebug.print(F("\r\nL2: "));
        SerialDebug.print(PS3[i]->getAnalogButton(L2));
        if (PS3[i]->PS3Connected) {
          SerialDebug.print(F("\tR2: "));
          SerialDebug.print(PS3[i]->getAnalogButton(R2));
        }
      }
      if (PS3[i]->getButtonClick(PS)) {
        SerialDebug.print(F("\r\nPS"));
        PS3[i]->disconnect();
        oldControllerState[i] = false; // Reset value
      }
      else {
        if (PS3[i]->getButtonClick(TRIANGLE))
          SerialDebug.print(F("\r\nTraingle"));
        if (PS3[i]->getButtonClick(CIRCLE))
          SerialDebug.print(F("\r\nCircle"));
        if (PS3[i]->getButtonClick(CROSS))
          SerialDebug.print(F("\r\nCross"));
        if (PS3[i]->getButtonClick(SQUARE))
          SerialDebug.print(F("\r\nSquare"));

        if (PS3[i]->getButtonClick(UP)) {
          SerialDebug.print(F("\r\nUp"));
          if (PS3[i]->PS3Connected) {
            PS3[i]->setLedOff();
            PS3[i]->setLedOn(LED4);
          }
        }
        if (PS3[i]->getButtonClick(RIGHT)) {
          SerialDebug.print(F("\r\nRight"));
          if (PS3[i]->PS3Connected) {
            PS3[i]->setLedOff();
            PS3[i]->setLedOn(LED1);
          }
        }
        if (PS3[i]->getButtonClick(DOWN)) {
          SerialDebug.print(F("\r\nDown"));
          if (PS3[i]->PS3Connected) {
            PS3[i]->setLedOff();
            PS3[i]->setLedOn(LED2);
          }
        }
        if (PS3[i]->getButtonClick(LEFT)) {
          SerialDebug.print(F("\r\nLeft"));
          if (PS3[i]->PS3Connected) {
            PS3[i]->setLedOff();
            PS3[i]->setLedOn(LED3);
          }
        }

        if (PS3[i]->getButtonClick(L1))
          SerialDebug.print(F("\r\nL1"));
        if (PS3[i]->getButtonClick(L3))
          SerialDebug.print(F("\r\nL3"));
        if (PS3[i]->getButtonClick(R1))
          SerialDebug.print(F("\r\nR1"));
        if (PS3[i]->getButtonClick(R3))
          SerialDebug.print(F("\r\nR3"));

        if (PS3[i]->getButtonClick(SELECT)) {
          SerialDebug.print(F("\r\nSelect - "));
          PS3[i]->printStatusString();
        }
        if (PS3[i]->getButtonClick(START)) {
          SerialDebug.print(F("\r\nStart"));
          printAngle[i] = !printAngle[i];
        }
      }
      if (printAngle[i]) {
        SerialDebug.print(F("\r\nPitch: "));
        SerialDebug.print(PS3[i]->getAngle(Pitch));
        SerialDebug.print(F("\tRoll: "));
        SerialDebug.print(PS3[i]->getAngle(Roll));
      }
    }
    /* I have removed the PS3 Move code as an Uno will run out of RAM if it's included */
    //else if(PS3[i]->PS3MoveConnected) {
  }
}

void onInit() {
  for (uint8_t i = 0; i < length; i++) {
    if ((PS3[i]->PS3Connected || PS3[i]->PS3NavigationConnected) && !oldControllerState[i]) {
      oldControllerState[i] = true; // Used to check which is the new controller
      PS3[i]->setLedOn((LEDEnum)(i + 1)); // Cast directly to LEDEnum - see: "controllerEnums.h"
    }
  }
}
