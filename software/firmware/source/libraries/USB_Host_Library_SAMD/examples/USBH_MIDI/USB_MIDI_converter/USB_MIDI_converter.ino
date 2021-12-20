/*
 *******************************************************************************
 * USB-MIDI to Legacy Serial MIDI converter
 * Copyright (C) 2012-2017 Yuuichi Akagawa
 *
 * Idea from LPK25 USB-MIDI to Serial MIDI converter
 *   by Collin Cunningham - makezine.com, narbotic.com
 *
 * This is sample program. Do not expect perfect behavior.
 *******************************************************************************
 */

#include <usbh_midi.h>
#include <usbhub.h>

#ifdef USBCON
#define _MIDI_SERIAL_PORT Serial1
#else
#define _MIDI_SERIAL_PORT Serial
#endif
//////////////////////////
// MIDI Pin assign
// 2 : GND
// 4 : +5V(Vcc) with 220ohm
// 5 : TX
//////////////////////////

USBHost UsbH;
USBH_MIDI  Midi(&UsbH);

void MIDI_poll();
void doDelay(uint32_t t1, uint32_t t2, uint32_t delayTime);

void setup()
{
  _MIDI_SERIAL_PORT.begin(31250);

  if (UsbH.Init()) {
    while (1); //halt
  }
  delay( 200 );
}

void loop()
{
  UsbH.Task();
  uint32_t t1 = (uint32_t)micros();
  if ( Midi ) {
    MIDI_poll();
  }
  //delay(1ms)
  doDelay(t1, (uint32_t)micros(), 1000);
}

// Poll USB MIDI Controler and send to serial MIDI
void MIDI_poll()
{
  uint8_t outBuf[ 3 ];
  uint8_t size;

  do {
    if ( (size = Midi.RecvData(outBuf)) > 0 ) {
      //MIDI Output
      _MIDI_SERIAL_PORT.write(outBuf, size);
    }
  } while (size > 0);
}

// Delay time (max 16383 us)
void doDelay(uint32_t t1, uint32_t t2, uint32_t delayTime)
{
  uint32_t t3;

    t3 = t2 - t1;
  if ( t3 < delayTime ) {
    delayMicroseconds(delayTime - t3);
  }
}
