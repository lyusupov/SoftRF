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

bool printAngle;

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
  if (Wii.wiimoteConnected) {
    if (Wii.getButtonClick(HOME)) { // You can use getButtonPress to see if the button is held down
      SerialDebug.print(F("\r\nHOME"));
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

      if (Wii.getButtonClick(ONE))
        SerialDebug.print(F("\r\nOne"));
      if (Wii.getButtonClick(TWO))
        SerialDebug.print(F("\r\nTwo"));

      if (Wii.getButtonClick(A)) {
        printAngle = !printAngle;
        SerialDebug.print(F("\r\nA"));
      }
      if (Wii.getButtonClick(B)) {
        Wii.setRumbleToggle();
        SerialDebug.print(F("\r\nB"));
      }
    }
#if 0 // Set this to 1 in order to see the angle of the controllers
    if (printAngle) {
      SerialDebug.print(F("\r\nPitch: "));
      SerialDebug.print(Wii.getPitch());
      SerialDebug.print(F("\tRoll: "));
      SerialDebug.print(Wii.getRoll());
      if (Wii.motionPlusConnected) {
        SerialDebug.print(F("\tYaw: "));
        SerialDebug.print(Wii.getYaw());
      }
      if (Wii.nunchuckConnected) {
        SerialDebug.print(F("\tNunchuck Pitch: "));
        SerialDebug.print(Wii.getNunchuckPitch());
        SerialDebug.print(F("\tNunchuck Roll: "));
        SerialDebug.print(Wii.getNunchuckRoll());
      }
    }
#endif
  }
#if 0 // Set this to 1 if you are using a Nunchuck controller
  if (Wii.nunchuckConnected) {
    if (Wii.getButtonClick(Z))
      SerialDebug.print(F("\r\nZ"));
    if (Wii.getButtonClick(C))
      SerialDebug.print(F("\r\nC"));
    if (Wii.getAnalogHat(HatX) > 137 ||  Wii.getAnalogHat(HatX) < 117 || Wii.getAnalogHat(HatY) > 137 || Wii.getAnalogHat(HatY) < 117) {
      SerialDebug.print(F("\r\nHatX: "));
      SerialDebug.print(Wii.getAnalogHat(HatX));
      SerialDebug.print(F("\tHatY: "));
      SerialDebug.print(Wii.getAnalogHat(HatY));
    }
  }
#endif
}
