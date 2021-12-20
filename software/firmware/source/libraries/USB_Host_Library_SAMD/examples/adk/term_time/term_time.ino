#include <adk.h>
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

ADK adk(&UsbH, "Circuits@Home, ltd.",
              "USB Host Shield",
              "Arduino Terminal for Android",
              "1.0",
              "http://www.circuitsathome.com",
              "0000000000000001");

void setup()
{
  SerialDebug.begin(115200);
  SerialDebug.println("\r\nADK demo start");

  if (UsbH.Init()) {
    SerialDebug.println("USB host failed to assert");
    while (1); //halt
  }//if (UsbH.Init() == -1...
}

void loop()
{
  uint8_t buf[ 12 ] = { 0 }; //buffer to convert unsigned long to ASCII
  const char* sec_ela = " seconds elapsed\r";
  uint8_t rcode;

  UsbH.Task();
  if ( adk.isReady() == false ) {
    return;
  }

  ultoa((uint32_t)millis() / 1000, (char *)buf, 10 );

  rcode = adk.SndData( strlen((char *)buf), buf );
  if (rcode && rcode != USB_ERRORFLOW) {
    SerialDebug.print(F("\r\nData send: "));
    SerialDebug.print(rcode, HEX);
  }
  rcode = adk.SndData( strlen( sec_ela), (uint8_t *)sec_ela );
  if (rcode && rcode != USB_ERRORFLOW) {
    SerialDebug.print(F("\r\nData send: "));
    SerialDebug.print(rcode, HEX);
  }

  delay( 1000 );
}
