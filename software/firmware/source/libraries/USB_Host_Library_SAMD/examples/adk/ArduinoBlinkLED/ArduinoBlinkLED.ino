// The source for the Android application can be found at the following link: https://github.com/Lauszus/ArduinoBlinkLED
// The code for the Android application is heavily based on this guide: http://allaboutee.com/2011/12/31/arduino-adk-board-blink-an-led-with-your-phone-code-and-explanation/ by Miguel
#include <adk.h>

//
// CAUTION! WARNING! ATTENTION! VORSICHT! ADVARSEL! ¡CUIDADO! ВНИМАНИЕ!
//
// Pin 13 is occupied by the SCK pin on various Arduino boards,
// including Uno, Duemilanove, etc., so use a different pin for those boards.
//
// CAUTION! WARNING! ATTENTION! VORSICHT! ADVARSEL! ¡CUIDADO! ВНИМАНИЕ!
//
#if defined(LED_BUILTIN)
#define LED LED_BUILTIN // Use built in LED
#else
#define LED 9 // Set to something here that makes sense for your board.
#endif


// On SAMD boards where the native USB port is also the serial console, use
// Serial1 for the serial console. This applies to all SAMD boards except for
// Arduino Zero and M0 boards.
#if (USB_VID==0x2341 && defined(ARDUINO_SAMD_ZERO)) || (USB_VID==0x2a03 && defined(ARDUINO_SAM_ZERO))
#define SerialDebug SERIAL_PORT_MONITOR
#else
#define SerialDebug Serial1
#endif

USBHost UsbH;
ADK adk(&UsbH, "TKJElectronics", // Manufacturer Name
              "ArduinoBlinkLED", // Model Name
              "Example sketch for the USB Host Shield", // Description (user-visible string)
              "1.0", // Version
              "http://www.tkjelectronics.dk/uploads/ArduinoBlinkLED.apk", // URL (web page to visit if no installed apps support the accessory)
              "123456789"); // Serial Number (optional)

uint32_t timer;
bool connected;

void setup() {
  SerialDebug.begin(115200);
  if (UsbH.Init()) {
    SerialDebug.print("\r\nUSB host failed to assert");
    while (1); // halt
  }
  pinMode(LED, OUTPUT);
  SerialDebug.print("\r\nArduino Blink LED Started");
}

void loop() {
  UsbH.Task();

  if (adk.isReady()) {
    if (!connected) {
      connected = true;
      SerialDebug.print(F("\r\nConnected to accessory"));
    }

    uint8_t msg[1];
    uint16_t len = sizeof(msg);
    uint8_t rcode = adk.RcvData(&len, msg);
    if (rcode && rcode != USB_ERRORFLOW) {
      SerialDebug.print(F("\r\nData rcv: "));
      SerialDebug.print(rcode, HEX);
    } else if (len > 0) {
      SerialDebug.print(F("\r\nData Packet: "));
      SerialDebug.print(msg[0]);
      digitalWrite(LED, msg[0] ? HIGH : LOW);
    }

    if ((int32_t)((uint32_t)millis() - timer) >= 1000) { // Send data every 1s
      timer = (uint32_t)millis();
      rcode = adk.SndData(sizeof(timer), (uint8_t*)&timer);
      if (rcode && rcode != USB_ERRORFLOW) {
        SerialDebug.print(F("\r\nData send: "));
        SerialDebug.print(rcode, HEX);
      } else if (rcode != USB_ERRORFLOW) {
        SerialDebug.print(F("\r\nTimer: "));
        SerialDebug.print(timer);
      }
    }
  } else {
    if (connected) {
      connected = false;
      SerialDebug.print(F("\r\nDisconnected from accessory"));
      digitalWrite(LED, LOW);
    }
  }
}
