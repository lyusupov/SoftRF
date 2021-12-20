/*
  Example sketch for the SteelSeries SRW-S1 Steering Wheel - developed by Kristian Lauszus
  For more information visit my blog: http://blog.tkjelectronics.dk/ or
  send me an e-mail:  kristianl@tkjelectronics.com
*/

#include "SRWS1.h"
// On SAMD boards where the native USB port is also the serial console, use
// Serial1 for the serial console. This applies to all SAMD boards except for
// Arduino Zero and M0 boards.
#if (USB_VID==0x2341 && defined(ARDUINO_SAMD_ZERO)) || (USB_VID==0x2a03 && defined(ARDUINO_SAM_ZERO))
#define SerialDebug SERIAL_PORT_MONITOR
#else
#define SerialDebug Serial1
#endif

USBHost UsbH;
SRWS1 srw1(&UsbH);

bool printTilt;

void setup() {
  SerialDebug.begin(115200);
  if (UsbH.Init()) {
    SerialDebug.print(F("\r\nUSB host did not start"));
    while (1); // Halt
  }
  SerialDebug.println(F("\r\nSteelSeries SRW-S1 Steering Wheel example started"));
}

void loop() {
  UsbH.Task();

  if (srw1.connected()) {
    if (printTilt) { // Show tilt angle using the LEDs
      srw1.setLeds(1 << map(srw1.srws1Data.tilt, -1800, 1800, 0, 14)); // Turn on a LED according to tilt value
      SerialDebug.println(srw1.srws1Data.tilt);
    } else { // Show strobe light effect
      static uint32_t timer;
      if ((int32_t)((uint32_t)millis() - timer) > 12) {
        timer = (uint32_t)millis(); // Reset timer

        static uint16_t leds = 0;
        //PrintHex<uint16_t > (leds, 0x80); SerialDebug.println();
        srw1.setLeds(leds); // Update LEDs

        static bool dirUp = true;
        if (dirUp) {
          leds <<= 1;
          if (leds == 0x8000) // All are actually turned off, as there is only 15 LEDs
            dirUp = false; // If we have reached the end i.e. all LEDs are off, then change direction
          else if (!(leds & 0x8000)) // If last bit is not set, then set the lowest bit
            leds |= 1; // Set lowest bit
        } else {
          leds >>= 1;
          if (leds == 0) // Check if all LEDs are off
            dirUp = true; // If all LEDs are off, then repeat the sequence
          else if (!(leds & 0x1)) // If last bit is not set, then set the top bit
            leds |= 1 << 15; // Set top bit
        }
      }
    }

    if (srw1.srws1Data.leftTrigger) {
      SerialDebug.print(F("L2: "));
      SerialDebug.println(srw1.srws1Data.leftTrigger);
    }
    if (srw1.srws1Data.rightTrigger) {
      SerialDebug.print(F("R2: "));
      SerialDebug.println(srw1.srws1Data.rightTrigger);
    }

    if (srw1.buttonClickState.select) {
      srw1.buttonClickState.select = 0; // Clear event
      SerialDebug.println(F("Select"));
      printTilt = !printTilt; // Print tilt value & show it using the LEDs as well
    }

    if (srw1.buttonClickState.back) {
      srw1.buttonClickState.back = 0; // Clear event
      SerialDebug.println(F("Back"));
    }
    if (srw1.buttonClickState.lookLeft) {
      srw1.buttonClickState.lookLeft = 0; // Clear event
      SerialDebug.println(F("Look Left"));
    }
    if (srw1.buttonClickState.lights) {
      srw1.buttonClickState.lights = 0; // Clear event
      SerialDebug.println(F("Lights"));
    }
    if (srw1.buttonClickState.lookBack) {
      srw1.buttonClickState.lookBack = 0; // Clear event
      SerialDebug.println(F("Look Back"));
    }
    if (srw1.buttonClickState.rearBrakeBalance) {
      srw1.buttonClickState.rearBrakeBalance = 0; // Clear event
      SerialDebug.println(F("R. Brake Balance"));
    }
    if (srw1.buttonClickState.frontBrakeBalance) {
      srw1.buttonClickState.frontBrakeBalance = 0; // Clear event
      SerialDebug.println(F("F. Brake Balance"));
    }
    if (srw1.buttonClickState.requestPit) {
      srw1.buttonClickState.requestPit = 0; // Clear event
      SerialDebug.println(F("Request Pit"));
    }
    if (srw1.buttonClickState.leftGear) {
      srw1.buttonClickState.leftGear = 0; // Clear event
      SerialDebug.println(F("Left Gear"));
    }

    if (srw1.buttonClickState.camera) {
      srw1.buttonClickState.camera = 0; // Clear event
      SerialDebug.println(F("Camera"));
    }
    if (srw1.buttonClickState.lookRight) {
      srw1.buttonClickState.lookRight = 0; // Clear event
      SerialDebug.println(F("Look right"));
    }
    if (srw1.buttonClickState.boost) {
      srw1.buttonClickState.boost = 0; // Clear event
      SerialDebug.println(F("Boost"));
    }
    if (srw1.buttonClickState.horn) {
      srw1.buttonClickState.horn = 0; // Clear event
      SerialDebug.println(F("Horn"));
    }
    if (srw1.buttonClickState.hud) {
      srw1.buttonClickState.hud = 0; // Clear event
      SerialDebug.println(F("HUD"));
    }
    if (srw1.buttonClickState.launchControl) {
      srw1.buttonClickState.launchControl = 0; // Clear event
      SerialDebug.println(F("Launch Control"));
    }
    if (srw1.buttonClickState.speedLimiter) {
      srw1.buttonClickState.speedLimiter = 0; // Clear event
      SerialDebug.println(F("Speed Limiter"));
    }
    if (srw1.buttonClickState.rightGear) {
      srw1.buttonClickState.rightGear = 0; // Clear event
      SerialDebug.println(F("Right gear"));
    }

    if (srw1.srws1Data.assists) SerialDebug.println(srw1.srws1Data.assists);
    if (srw1.srws1Data.steeringSensitivity) SerialDebug.println(srw1.srws1Data.steeringSensitivity);
    if (srw1.srws1Data.assistValues) SerialDebug.println(srw1.srws1Data.assistValues);

    switch (srw1.srws1Data.btn.dpad) {
      case DPAD_UP:
        SerialDebug.println(F("Up"));
        break;
      case DPAD_UP_RIGHT:
        SerialDebug.println(F("UP & right"));
        break;
      case DPAD_RIGHT:
        SerialDebug.println(F("Right"));
        break;
      case DPAD_RIGHT_DOWN:
        SerialDebug.println(F("Right & down"));
        break;
      case DPAD_DOWN:
        SerialDebug.println(F("Down"));
        break;
      case DPAD_DOWN_LEFT:
        SerialDebug.println(F("Down & left"));
        break;
      case DPAD_LEFT:
        SerialDebug.println(F("Left"));
        break;
      case DPAD_LEFT_UP:
        SerialDebug.println(F("Left & up"));
        break;
      case DPAD_OFF:
        break;
      default:
        SerialDebug.print(F("Unknown state: "));
        PrintHex<uint8_t > (srw1.srws1Data.btn.dpad, 0x80);
        SerialDebug.println();
        break;
    }
  }
}

