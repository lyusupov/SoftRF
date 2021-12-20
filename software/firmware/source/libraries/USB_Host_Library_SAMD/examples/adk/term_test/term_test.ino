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
//USBHub     Hub(&UsbH);

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
  uint8_t rcode;
  uint8_t msg[64] = { 0x00 };
  const char* recv = "Received: ";

  UsbH.Task();

  if ( adk.isReady() == false ) {
    return;
  }
  uint16_t len = 64;

  rcode = adk.RcvData(&len, msg);
  if ( rcode & ( rcode != USB_ERRORFLOW )) {
    USBTRACE2("Data rcv. :", rcode );
  }
  if (len > 0) {
    USBTRACE("\r\nData Packet.");

    for ( uint8_t i = 0; i < len; i++ ) {
      SerialDebug.print((char)msg[i]);
    }
    /* sending back what was received */
    rcode = adk.SndData( strlen( recv ), (uint8_t *)recv );
    if (rcode && rcode != USB_ERRORFLOW) {
      SerialDebug.print(F("\r\nData send: "));
      SerialDebug.print(rcode, HEX);
    }
    rcode = adk.SndData( strlen(( char * )msg ), msg );
    if (rcode && rcode != USB_ERRORFLOW) {
      SerialDebug.print(F("\r\nData send: "));
      SerialDebug.print(rcode, HEX);
    }

  }//if( len > 0 )...

  delay( 1000 );
}

