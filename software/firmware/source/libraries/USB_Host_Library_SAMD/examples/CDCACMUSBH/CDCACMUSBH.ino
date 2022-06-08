#include <cdcacm.h>
#include <usbhub.h>

// On SAMD boards where the native USB port is also the serial console, use
// Serial1 for the serial console. This applies to all SAMD boards except for
// Arduino Zero and M0 boards.
#if (USB_VID==0x2341 && defined(ARDUINO_SAMD_ZERO)) || (USB_VID==0x2a03 && defined(ARDUINO_SAM_ZERO))
#define SerialDebug SERIAL_PORT_MONITOR
#else
#define SerialDebug Serial1
#endif

#define DEBUG_ON  0
#if DEBUG_ON
#define dbbegin(...)      SerialDebug.begin(__VA_ARGS__)
#define dbprint(...)      SerialDebug.print(__VA_ARGS__)
#define dbprintln(...)    SerialDebug.println(__VA_ARGS__)
#else
#define dbbegin(...)
#define dbprint(...)
#define dbprintln(...)
#endif

#ifdef ADAFRUIT_TRINKET_M0
// setup Dotstar LED on Trinket M0
#include <Adafruit_DotStar.h>
#define DATAPIN    7
#define CLOCKPIN   8
Adafruit_DotStar strip = Adafruit_DotStar(1, DATAPIN, CLOCKPIN, DOTSTAR_BRG);
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
    lc.dwDTERate	= 38400;
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
ACM           AcmSerial(&UsbH, &AsyncOper);

void setup()
{
  // Turn off built-in RED LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
#ifdef ADAFRUIT_TRINKET_M0
  // Turn off built-in Dotstar RGB LED
  strip.begin();
  strip.clear();
  strip.show();
#endif

  dbbegin( 38400 );
  dbprintln("Start");
  Serial1.begin(38400);

  if (UsbH.Init())
      dbprintln("USB host failed to initialize");

  delay( 200 );
  dbprintln("USB Host init OK");
}

void loop()
{
  UsbH.Task();

  if( AcmSerial.isReady()) {
    uint8_t rcode;
    int bytesIn;
    uint8_t data[64];

    if((bytesIn = Serial1.available()) > 0) {
      bytesIn = Serial1.readBytes(data, min(bytesIn, sizeof(data)));
      rcode = AcmSerial.SndData(bytesIn, data);
      if (rcode) {
        ErrorMessage<uint8_t>(PSTR("SndData"), rcode);
      }
    }

    uint16_t rcvd = sizeof(data);
    rcode = AcmSerial.RcvData(&rcvd, data);
    if (rcode && rcode != USB_ERRORFLOW) {
      ErrorMessage<uint8_t>(PSTR("RcvData"), rcode);
    }
    else {
      if( rcvd ) {
        Serial1.write(data, rcvd);
      }
    }
  }
}
