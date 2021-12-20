#include <cdcftdi.h>
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

class FTDIAsync : public FTDIAsyncOper
{
public:
    uint8_t OnInit(FTDI *pftdi);
};

uint8_t FTDIAsync::OnInit(FTDI *pftdi)
{
    uint8_t rcode = 0;

    rcode = pftdi->SetBaudRate(115200);

    if (rcode)
    {
        ErrorMessage<uint8_t>(PSTR("SetBaudRate"), rcode);
        return rcode;
    }
    rcode = pftdi->SetFlowControl(FTDI_SIO_DISABLE_FLOW_CTRL);

    if (rcode)
        ErrorMessage<uint8_t>(PSTR("SetFlowControl"), rcode);

    return rcode;
}

USBHost          UsbH;
//USBHub         Hub(&UsbH);
FTDIAsync        FtdiAsync;
FTDI             Ftdi(&UsbH, &FtdiAsync);

void setup()
{
  SerialDebug.begin( 115200 );
  SerialDebug.println("Start");

  if (UsbH.Init())
      SerialDebug.println("USB host did not start.");

  delay( 200 );
}

void loop()
{
    UsbH.Task();

    if( UsbH.getUsbTaskState() == USB_STATE_RUNNING )
    {
        uint32_t  rcode;
        char strbuf[] = "DEADBEEF";
        //char strbuf[] = "The quick brown fox jumps over the lazy dog";
        //char strbuf[] = "This string contains 61 character to demonstrate FTDI buffers"; //add one symbol to it to see some garbage
        SerialDebug.print(".");

        rcode = Ftdi.SndData(strlen(strbuf), (uint8_t*)strbuf);

	if (rcode)
            ErrorMessage<uint8_t>(PSTR("SndData"), rcode);

        delay(50);

        uint8_t  buf[64];

        for (uint8_t i=0; i<64; i++)
            buf[i] = 0;

        uint16_t rcvd = 64;
        rcode = Ftdi.RcvData(&rcvd, buf);

        if (rcode && rcode != USB_ERRORFLOW)
            ErrorMessage<uint8_t>(PSTR("Ret"), rcode);

        // The device reserves the first two bytes of data
        //   to contain the current values of the modem and line status registers.
        if (rcvd > 2)
            SerialDebug.print((char*)(buf+2));

        delay(10);
    }
}

