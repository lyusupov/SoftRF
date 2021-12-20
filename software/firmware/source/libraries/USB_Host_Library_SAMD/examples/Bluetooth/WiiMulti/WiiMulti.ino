/*
 Example sketch for the Wiimote Bluetooth library - developed by Kristian Lauszus
 This example show how one can use multiple controllers with the library
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
WII *Wii[2]; // We will use this pointer to store the two instance, you can easily make it larger if you like, but it will use a lot of RAM!
const uint8_t length = sizeof(Wii) / sizeof(Wii[0]); // Get the lenght of the array
bool printAngle[length];
bool oldControllerState[length];

void setup() {
  for (uint8_t i = 0; i < length; i++) {
    Wii[i] = new WII(&Btd); // You will have to pair each controller with the dongle before you can define the instances like so, just add PAIR as the second argument
    Wii[i]->attachOnInit(onInit); // onInit() is called upon a new connection - you can call the function whatever you like
  }

  SerialDebug.begin(115200);
  if (UsbH.Init()) {
    SerialDebug.print(F("\r\nUSB host did not start"));
    while (1); //halt
  }
  SerialDebug.print(F("\r\nWiimote Bluetooth Library Started"));
}
void loop() {
  UsbH.Task();

  for (uint8_t i = 0; i < length; i++) {
    if (Wii[i]->wiimoteConnected) {
      if (Wii[i]->getButtonClick(HOME)) { // You can use getButtonPress to see if the button is held down
        SerialDebug.print(F("\r\nHOME"));
        Wii[i]->disconnect();
        oldControllerState[i] = false; // Reset value
      }
      else {
        if (Wii[i]->getButtonClick(LEFT)) {
          Wii[i]->setLedOff();
          Wii[i]->setLedOn(LED1);
          SerialDebug.print(F("\r\nLeft"));
        }
        if (Wii[i]->getButtonClick(RIGHT)) {
          Wii[i]->setLedOff();
          Wii[i]->setLedOn(LED3);
          SerialDebug.print(F("\r\nRight"));
        }
        if (Wii[i]->getButtonClick(DOWN)) {
          Wii[i]->setLedOff();
          Wii[i]->setLedOn(LED4);
          SerialDebug.print(F("\r\nDown"));
        }
        if (Wii[i]->getButtonClick(UP)) {
          Wii[i]->setLedOff();
          Wii[i]->setLedOn(LED2);
          SerialDebug.print(F("\r\nUp"));
        }

        if (Wii[i]->getButtonClick(PLUS))
          SerialDebug.print(F("\r\nPlus"));
        if (Wii[i]->getButtonClick(MINUS))
          SerialDebug.print(F("\r\nMinus"));

        if (Wii[i]->getButtonClick(ONE))
          SerialDebug.print(F("\r\nOne"));
        if (Wii[i]->getButtonClick(TWO))
          SerialDebug.print(F("\r\nTwo"));

        if (Wii[i]->getButtonClick(A)) {
          printAngle[i] = !printAngle[i];
          SerialDebug.print(F("\r\nA"));
        }
        if (Wii[i]->getButtonClick(B)) {
          Wii[i]->setRumbleToggle();
          SerialDebug.print(F("\r\nB"));
        }
      }
      if (printAngle[i]) {
        SerialDebug.print(F("\r\nPitch: "));
        SerialDebug.print(Wii[i]->getPitch());
        SerialDebug.print(F("\tRoll: "));
        SerialDebug.print(Wii[i]->getRoll());
        if (Wii[i]->motionPlusConnected) {
          SerialDebug.print(F("\tYaw: "));
          SerialDebug.print(Wii[i]->getYaw());
        }
        if (Wii[i]->nunchuckConnected) {
          SerialDebug.print(F("\tNunchuck Pitch: "));
          SerialDebug.print(Wii[i]->getNunchuckPitch());
          SerialDebug.print(F("\tNunchuck Roll: "));
          SerialDebug.print(Wii[i]->getNunchuckRoll());
        }
      }
    }
    if (Wii[i]->nunchuckConnected) {
      if (Wii[i]->getButtonClick(Z))
        SerialDebug.print(F("\r\nZ"));
      if (Wii[i]->getButtonClick(C))
        SerialDebug.print(F("\r\nC"));
      if (Wii[i]->getAnalogHat(HatX) > 137 ||  Wii[i]->getAnalogHat(HatX) < 117 || Wii[i]->getAnalogHat(HatY) > 137 || Wii[i]->getAnalogHat(HatY) < 117) {
        SerialDebug.print(F("\r\nHatX: "));
        SerialDebug.print(Wii[i]->getAnalogHat(HatX));
        SerialDebug.print(F("\tHatY: "));
        SerialDebug.print(Wii[i]->getAnalogHat(HatY));
      }
    }
  }
}

void onInit() {
  for (uint8_t i = 0; i < length; i++) {
    if (Wii[i]->wiimoteConnected && !oldControllerState[i]) {
      oldControllerState[i] = true; // Used to check which is the new controller
      Wii[i]->setLedOn((LEDEnum)(i + 1)); // Cast directly to LEDEnum - see: "controllerEnums.h"
    }
  }
}
