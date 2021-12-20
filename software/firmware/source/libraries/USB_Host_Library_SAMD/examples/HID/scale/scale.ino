/* Digital Scale Output. Written for Stamps.com Model 510  */
/* 5lb Digital Scale; any HID scale with Usage page 0x8d should work */

#include <hid.h>
#include <hiduniversal.h>
#include <usbhub.h>

#include "scale_rptparser.h"

// On SAMD boards where the native USB port is also the serial console, use
// Serial1 for the serial console. This applies to all SAMD boards except for
// Arduino Zero and M0 boards.
#if (USB_VID==0x2341 && defined(ARDUINO_SAMD_ZERO)) || (USB_VID==0x2a03 && defined(ARDUINO_SAM_ZERO))
#define SerialDebug SERIAL_PORT_MONITOR
#else
#define SerialDebug Serial1
#endif

USBHost                                             UsbH;
USBHub                                          Hub(&UsbH);
HIDUniversal                                    Hid(&UsbH);
Max_LCD                                       LCD(&UsbH);
ScaleEvents                                  ScaleEvents(&LCD);
ScaleReportParser                            Scale(&ScaleEvents);

void setup()
{
  SerialDebug.begin( 115200 );
  SerialDebug.println("Start");

  if (UsbH.Init())
      SerialDebug.println("USB host did not start.");

    // set up the LCD's number of rows and columns:
    LCD.begin(16, 2);
    LCD.clear();
    LCD.home();
    LCD.setCursor(0,0);
    LCD.write('R');

  delay( 200 );

  if (!Hid.SetReportParser(0, &Scale))
      ErrorMessage<uint8_t>(PSTR("SetReportParser"), 1  );
}

void loop()
{
    UsbH.Task();
}

