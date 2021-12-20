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
USBHub hub0(&UsbH);
USBHub hub1(&UsbH);
ADK adk(&UsbH, "Google, Inc.",
              "DemoKit",
              "DemoKit Arduino Board",
              "1.0",
              "http://www.android.com",
              "0000000012345678");
uint8_t  b, b1;


#define LED1_RED          3
#define BUTTON1           2

#ifdef ESP32
#define LED1_RED_CHANNEL  0
#endif

void init_buttons()
{
  pinMode(BUTTON1, INPUT);

  // enable the internal pullups
  digitalWrite(BUTTON1, HIGH);
}

void init_leds()
{
  digitalWrite(LED1_RED, 0);

#ifdef ESP32
  ledcAttachPin(LED1_RED, LED1_RED_CHANNEL); // Assign LED pin to channel 0
  ledcSetup(LED1_RED_CHANNEL, 12000, 8); // 12 kHz PWM, 8-bit resolution
#else
  pinMode(LED1_RED, OUTPUT);
#endif
}

void setup()
{
  SerialDebug.begin(115200);
  SerialDebug.println("\r\nADK demo start");

  if (UsbH.Init()) {
    SerialDebug.println("USB host failed to assert");
    while (1); //halt
  }//if (UsbH.Init() == -1...

  init_leds();
  init_buttons();
  b1 = digitalRead(BUTTON1);
}

void loop()
{
  uint8_t rcode;
  uint8_t msg[3] = { 0x00 };
  UsbH.Task();

  if ( adk.isReady() == false ) {
#ifdef ESP32
    ledcWrite(LED1_RED_CHANNEL, 255);
#else
    analogWrite(LED1_RED, 255);
#endif
    return;
  }
  uint16_t len = sizeof(msg);

  rcode = adk.RcvData(&len, msg);
  if ( rcode ) {
    USBTRACE2("Data rcv. :", rcode );
  }
  if (len > 0) {
    USBTRACE("\r\nData Packet.");
    // assumes only one command per packet
    if (msg[0] == 0x2) {
      switch ( msg[1] ) {
        case 0:
#ifdef ESP32
          ledcWrite(LED1_RED_CHANNEL, 255 - msg[2]);
#else
          analogWrite(LED1_RED, 255 - msg[2]);
#endif
          break;
      }//switch( msg[1]...
    }//if (msg[0] == 0x2...
  }//if( len > 0...

  msg[0] = 0x1;

  b = digitalRead(BUTTON1);
  if (b != b1) {
    USBTRACE("\r\nButton state changed");
    msg[1] = 0;
    msg[2] = b ? 0 : 1;
    rcode = adk.SndData( 3, msg );
    if ( rcode ) {
      USBTRACE2("Button send: ", rcode );
    }
    b1 = b;
  }//if (b != b1...


  delay( 10 );
}
