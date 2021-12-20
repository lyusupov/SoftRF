/*
Example sketch for the Wii libary showing the IR camera functionality. This example
is for the Bluetooth Wii library developed for the USB shield from Circuits@Home

Created by Allan Glover and Kristian Lauszus.
Contact Kristian:  http://blog.tkjelectronics.dk/ or send an email at kristianl@tkjelectronics.com.
Contact Allan at adglover9.81@gmail.com

To test the Wiimote IR camera, you will need access to an IR source. Sunlight will work but is not ideal.
The simpleist solution is to use the Wii sensor bar, i.e. emitter bar, supplied by the Wii system.
Otherwise, wire up a IR LED yourself.
*/

// On SAMD boards where the native USB port is also the serial console, use
// Serial1 for the serial console. This applies to all SAMD boards except for
// Arduino Zero and M0 boards.
#if (USB_VID==0x2341 && defined(ARDUINO_SAMD_ZERO)) || (USB_VID==0x2a03 && defined(ARDUINO_SAM_ZERO))
#define SerialDebug SERIAL_PORT_MONITOR
#else
#define SerialDebug Serial1
#endif

#include <Wii.h>
#include <usbhub.h>

#ifndef WIICAMERA // Used to check if WIICAMERA is defined
#error "Please set ENABLE_WII_IR_CAMERA to 1 in settings.h"
#endif

USBHost UsbH;
//USBHub Hub1(&UsbH); // Some dongles have a hub inside

BTD Btd(&UsbH); // You have to create the Bluetooth Dongle instance like so
/* You can create the instance of the class in two ways */
WII Wii(&Btd, PAIR); // This will start an inquiry and then pair with your Wiimote - you only have to do this once
//WII Wii(&Btd); // After the Wiimote pairs once with the line of code above, you can simply create the instance like so and re upload and then press any button on the Wiimote

bool printAngle;
uint8_t printObjects;

void setup() {
  SerialDebug.begin(115200);
  if (UsbH.Init() == -1) {
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
      if (Wii.getButtonClick(ONE))
        Wii.IRinitialize(); // Run the initialisation sequence
      if (Wii.getButtonClick(MINUS) || Wii.getButtonClick(PLUS)) {
        if (!Wii.isIRCameraEnabled())
          SerialDebug.print(F("\r\nEnable IR camera first"));
        else {
          if (Wii.getButtonPress(MINUS)) { // getButtonClick will only return true once
            if (printObjects > 0)
              printObjects--;
          }
          else {
            if (printObjects < 4)
              printObjects++;
          }
          SerialDebug.print(F("\r\nTracking "));
          SerialDebug.print(printObjects);
          SerialDebug.print(F(" objects"));
        }
      }
      if (Wii.getButtonClick(A)) {
        printAngle = !printAngle;
        SerialDebug.print(F("\r\nA"));
      }
      if (Wii.getButtonClick(B)) {
        SerialDebug.print(F("\r\nBattery level: "));
        SerialDebug.print(Wii.getBatteryLevel()); // You can get the battery level as well
      }
    }
    if (printObjects > 0) {
      if (Wii.getIRx1() != 0x3FF || Wii.getIRy1() != 0x3FF || Wii.getIRs1() != 0) { // Only print if the IR camera is actually seeing something
        SerialDebug.print(F("\r\nx1: "));
        SerialDebug.print(Wii.getIRx1());
        SerialDebug.print(F("\ty1: "));
        SerialDebug.print(Wii.getIRy1());
        SerialDebug.print(F("\ts1:"));
        SerialDebug.print(Wii.getIRs1());
      }
      if (printObjects > 1) {
        if (Wii.getIRx2() != 0x3FF || Wii.getIRy2() != 0x3FF || Wii.getIRs2() != 0) {
          SerialDebug.print(F("\r\nx2: "));
          SerialDebug.print(Wii.getIRx2());
          SerialDebug.print(F("\ty2: "));
          SerialDebug.print(Wii.getIRy2());
          SerialDebug.print(F("\ts2:"));
          SerialDebug.print(Wii.getIRs2());
        }
        if (printObjects > 2) {
          if (Wii.getIRx3() != 0x3FF || Wii.getIRy3() != 0x3FF || Wii.getIRs3() != 0) {
            SerialDebug.print(F("\r\nx3: "));
            SerialDebug.print(Wii.getIRx3());
            SerialDebug.print(F("\ty3: "));
            SerialDebug.print(Wii.getIRy3());
            SerialDebug.print(F("\ts3:"));
            SerialDebug.print(Wii.getIRs3());
          }
          if (printObjects > 3) {
            if (Wii.getIRx4() != 0x3FF || Wii.getIRy4() != 0x3FF || Wii.getIRs4() != 0) {
              SerialDebug.print(F("\r\nx4: "));
              SerialDebug.print(Wii.getIRx4());
              SerialDebug.print(F("\ty4: "));
              SerialDebug.print(Wii.getIRy4());
              SerialDebug.print(F("\ts4:"));
              SerialDebug.print(Wii.getIRs4());
            }
          }
        }
      }
    }
    if (printAngle) { // There is no extension bytes available, so the MotionPlus or Nunchuck can't be read
      SerialDebug.print(F("\r\nPitch: "));
      SerialDebug.print(Wii.getPitch());
      SerialDebug.print(F("\tRoll: "));
      SerialDebug.print(Wii.getRoll());
    }
  }
}
