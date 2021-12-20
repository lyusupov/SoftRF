/*
 *******************************************************************************
 * USB-MIDI dump utility
 * Copyright (C) 2013-2017 Yuuichi Akagawa
 *
 * for use with USB Host Shield 2.0 from Circuitsathome.com
 * https://github.com/felis/USB_Host_Shield_2.0
 *
 * This is sample program. Do not expect perfect behavior.
 *******************************************************************************
 */

#include <usbh_midi.h>
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
USBHub Hub(&UsbH);
USBH_MIDI  Midi(&UsbH);

void MIDI_poll();
void doDelay(uint32_t t1, uint32_t t2, uint32_t delayTime);

boolean bFirst;
uint16_t pid, vid;

void setup()
{
  bFirst = true;
  vid = pid = 0;
  SerialDebug.begin(115200);

  if (UsbH.Init()) {
    SerialDebug.println("USB host did not start");
    while (1); //halt
  }
  delay( 200 );
}

void loop()
{
  UsbH.Task();
  //uint32_t t1 = (uint32_t)micros();
  if ( Midi ) {
    MIDI_poll();
  }
}

// Poll USB MIDI Controler and send to serial MIDI
void MIDI_poll()
{
  char buf[20];
  uint8_t bufMidi[64];
  uint16_t  rcvd;

  if (Midi.idVendor() != vid || Midi.idProduct() != pid) {
    vid = Midi.idVendor();
    pid = Midi.idProduct();
    sprintf(buf, "VID:%04X, PID:%04X", vid, pid);
    SerialDebug.println(buf);
  }
  if (Midi.RecvData(&rcvd,  bufMidi) == 0 ) {
    uint32_t time = (uint32_t)millis();
    sprintf(buf, "%04X%04X: ", (uint16_t)(time >> 16), (uint16_t)(time & 0xFFFF)); // Split variable to prevent warnings on the ESP8266 platform
    SerialDebug.print(buf);
    SerialDebug.print(rcvd);
    SerialDebug.print(':');
    for (int i = 0; i < 64; i++) {
      sprintf(buf, " %02X", bufMidi[i]);
      SerialDebug.print(buf);
    }
    SerialDebug.println();
  }
}
