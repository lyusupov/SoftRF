#include <cdc_XR21B1411.h>

// On SAMD boards where the native USB port is also the serial console, use
// Serial1 for the serial console. This applies to all SAMD boards except for
// Arduino Zero and M0 boards.
#if (USB_VID==0x2341 && defined(ARDUINO_SAMD_ZERO)) || (USB_VID==0x2a03 && defined(ARDUINO_SAM_ZERO))
#define SerialDebug SERIAL_PORT_MONITOR
#else
#define SerialDebug Serial1
#endif

class ACMAsyncOper : public CDCAsyncOper
{
public:
    uint8_t OnInit(ACM *pacm);
};

uint8_t ACMAsyncOper::OnInit(ACM *pacm)
{
    uint8_t rcode;
    // Set DTR = 1 RTS=1
    rcode = pacm->SetControlLineState(3);

    if (rcode)
    {
        ErrorMessage<uint8_t>(PSTR("SetControlLineState"), rcode);
        return rcode;
    }

    LINE_CODING	lc;
    lc.dwDTERate	= 115200;
    lc.bCharFormat	= 0;
    lc.bParityType	= 0;
    lc.bDataBits	= 8;

    rcode = pacm->SetLineCoding(&lc);

    if (rcode)
        ErrorMessage<uint8_t>(PSTR("SetLineCoding"), rcode);

    return rcode;
}

USBHost     UsbH;
ACMAsyncOper  AsyncOper;
XR21B1411     Acm(&UsbH, &AsyncOper);

void setup() {
        SerialDebug.begin( 115200 );
        SerialDebug.println("\r\n\r\nStart");

        if (UsbH.Init()) SerialDebug.println("USB host failed to assert");
}

void loop() {
        UsbH.Task();
        if( Acm.isReady()) {
                uint8_t rcode;
                uint8_t  buf[1];
                uint16_t rcvd = 1;

                /* read keyboard */
                if(SerialDebug.available()) {
                         uint8_t data = SerialDebug.read();
                         /* send */
                         rcode = Acm.SndData(1, &data);
                         if (rcode)
                                 ErrorMessage<uint8_t>(PSTR("SndData"), rcode);
                 }

                /* read XR serial */
                rcode = Acm.RcvData(&rcvd, buf);
                if (rcode && rcode != USB_ERRORFLOW)
                        ErrorMessage<uint8_t>(PSTR("Ret"), rcode);

                if( rcvd ) { //more than zero bytes received
                        for(uint16_t i=0; i < rcvd; i++ ) {
                                SerialDebug.print((char)buf[i]);
                        }
                }
        }
}

