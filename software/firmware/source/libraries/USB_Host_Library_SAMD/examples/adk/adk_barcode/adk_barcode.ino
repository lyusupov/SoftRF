/**/
/* A sketch demonstrating data exchange between two USB devices - a HID barcode scanner and ADK-compatible Android phone */
/**/
#include <adk.h>
#include <hidboot.h>
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
USBHub Hub1(&UsbH);
USBHub Hub2(&UsbH);
HIDBoot<HID_PROTOCOL_KEYBOARD> HidKeyboard(&UsbH);

ADK adk(&UsbH,"Circuits@Home, ltd.",
            "USB Host Shield",
            "Arduino Terminal for Android",
            "1.0",
            "http://www.circuitsathome.com",
            "0000000000000001");


class KbdRptParser : public KeyboardReportParser
{

protected:
	void OnKeyDown	(uint8_t mod, uint8_t key);
	void OnKeyPressed(uint8_t key);
};

void KbdRptParser::OnKeyDown(uint8_t mod, uint8_t key)
{
    uint8_t c = OemToAscii(mod, key);

    if (c)
        OnKeyPressed(c);
}

/* what to do when symbol arrives */
void KbdRptParser::OnKeyPressed(uint8_t key)
{
const char* new_line = "\n";
uint8_t rcode;
uint8_t keylcl;

 if( adk.isReady() == false ) {
   return;
 }

  keylcl = key;

  if( keylcl == 0x13 ) {
    rcode = adk.SndData( strlen( new_line ), (uint8_t *)new_line );
    if (rcode && rcode != USB_ERRORFLOW) {
      SerialDebug.print(F("\r\nData send: "));
      SerialDebug.print(rcode, HEX);
    }
  }
  else {
    rcode = adk.SndData( 1, &keylcl );
    if (rcode && rcode != USB_ERRORFLOW) {
      SerialDebug.print(F("\r\nData send: "));
      SerialDebug.print(rcode, HEX);
    }
  }

  SerialDebug.print((char) keylcl );
  SerialDebug.print(" : ");
  SerialDebug.println( keylcl, HEX );
};

KbdRptParser Prs;

void setup()
{
  SerialDebug.begin(115200);
  SerialDebug.println("\r\nADK demo start");

  if (UsbH.Init()) {
    SerialDebug.println("USB host failed to assert");
    while(1); //halt
  }//if (UsbH.Init() == -1...

  HidKeyboard.SetReportParser(0, &Prs);

  delay( 200 );
}

void loop()
{
  UsbH.Task();
}
