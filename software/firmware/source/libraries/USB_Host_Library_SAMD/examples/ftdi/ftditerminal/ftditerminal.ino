#include <cdcftdi.h>
#include <usbhub.h>

//#include "pgmstrings.h"

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

  if (( UsbH.getUsbTaskState() == USB_STATE_RUNNING ) && Ftdi.isReady() ) {
    uint8_t rcode;
    int bytesIn;
    char buf[64];

    /* reading the keyboard */
    if((bytesIn = SerialDebug.available()) > 0) {
      bytesIn = SerialDebug.readBytes(buf, min(bytesIn, sizeof(buf)));
      if (bytesIn > 0) {
        /* sending to USB serial */
        rcode = Ftdi.SndData(bytesIn, (uint8_t*)buf);
        if (rcode)
          ErrorMessage<uint8_t>(PSTR("SndData"), rcode);
      }
    }

    /* reading USB serial */
    /* buffer size must be greater or equal to max.packet size */
    /* it it set to 64 (largest possible max.packet size) here, can be tuned down
       for particular endpoint */
    uint16_t rcvd = sizeof(buf);
    rcode = Ftdi.RcvData(&rcvd, (uint8_t *)buf);
    if (rcode && rcode != USB_ERRORFLOW)
      ErrorMessage<uint8_t>(PSTR("Ret"), rcode);
    else {
      // The device reserves the first two bytes of data
      //   to contain the current values of the modem and line status registers.
      if (rcvd > 2) {
        SerialDebug.write(buf+2, rcvd-2);
      }
    }
  }
}
