#include <hid.h>
#include <hiduniversal.h>
#include <hidescriptorparser.h>
#include <usbhub.h>
#include "pgmstrings.h"

// On SAMD boards where the native USB port is also the serial console, use
// Serial1 for the serial console. This applies to all SAMD boards except for
// Arduino Zero and M0 boards.
#if (USB_VID==0x2341 && defined(ARDUINO_SAMD_ZERO)) || (USB_VID==0x2a03 && defined(ARDUINO_SAM_ZERO))
#define SerialDebug SERIAL_PORT_MONITOR
#else
#define SerialDebug Serial1
#endif

class HIDUniversal2 : public HIDUniversal
{
public:
    HIDUniversal2(USBHost *usb) : HIDUniversal(usb) {};

protected:
    uint32_t OnInitSuccessful();
};

uint32_t HIDUniversal2::OnInitSuccessful()
{
    uint8_t    rcode;

    SerialDebug.println("HIDUniversal2::OnInitSuccessful");
    HexDumper<USBReadParser, uint32_t, uint32_t>    Hex;
    ReportDescParser                                Rpt;

    if ((rcode = GetReportDescr(0, &Hex)))
        goto FailGetReportDescr1;

    if ((rcode = GetReportDescr(0, &Rpt)))
	goto FailGetReportDescr2;

    return 0;

FailGetReportDescr1:
    SerialDebug.println("HIDUniversal2::OnInitSuccessful fail1");
    USBTRACE("GetReportDescr1:");
    goto Fail;

FailGetReportDescr2:
    SerialDebug.println("HIDUniversal2::OnInitSuccessful fail2");
    USBTRACE("GetReportDescr2:");
    goto Fail;

Fail:
    SerialDebug.println(rcode, HEX);
    Release();
    return rcode;
}

USBHost UsbH;
//USBHub Hub(&UsbH);
HIDUniversal2 Hid(&UsbH);
UniversalReportParser Uni;

void setup()
{
  SerialDebug.begin( 115200 );
  SerialDebug.println("Start");

  if (UsbH.Init())
      SerialDebug.println("USB host did not start.");

  delay( 200 );

  if (!Hid.SetReportParser(0, &Uni))
      ErrorMessage<uint8_t>(PSTR("SetReportParser"), 1  );
}

void loop()
{
    UsbH.Task();
}

