/*
 * uatm.cpp
 * Copyright (C) 2019-2025 Linar Yusupov
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "../RF.h"

#if !defined(EXCLUDE_UATM)
/*
 * UATM-specific code
 *
 *
 */

#include <fec.h>
#include <uat.h>

#include "../EEPROM.h"

#define UAT_RINGBUF_SIZE  (sizeof(Stratux_frame_t) * 2)

static bool uatm_probe(void);
static void uatm_setup(void);
static void uatm_channel(int8_t);
static bool uatm_receive(void);
static bool uatm_transmit(void);
static void uatm_shutdown(void);

const rfchip_ops_t uatm_ops = {
  RF_IC_UATM,
  "UATM",
  uatm_probe,
  uatm_setup,
  uatm_channel,
  uatm_receive,
  uatm_transmit,
  uatm_shutdown
};

static unsigned char uat_ringbuf[UAT_RINGBUF_SIZE];
static unsigned int uatbuf_head = 0;
Stratux_frame_t uatradio_frame;

const char UAT_ident[] PROGMEM = SOFTRF_IDENT;

static bool uatm_probe()
{
  bool success = false;
  unsigned long startTime;
  unsigned int uatbuf_tail;
  u1_t keylen = strlen_P(UAT_ident);
  u1_t i=0;

  /* Do not probe on itself and ESP8266 */
  if (SoC->id == SOC_CC13X0 ||
      SoC->id == SOC_CC13X2 ||
      SoC->id == SOC_ESP8266) {
    return success;
  }

  SoC->UATSerial_begin(UAT_RECEIVER_BR);

  SoC->UATModule_restart();

  startTime = millis();

  // Timeout if no valid response in 1 second
  while (millis() - startTime < 1000) {

    if (UATSerial.available() > 0) {
      unsigned char c = UATSerial.read();
#if DEBUG
      Serial.println(c, HEX);
#endif
      uat_ringbuf[uatbuf_head % UAT_RINGBUF_SIZE] = c;

      uatbuf_tail = uatbuf_head - keylen;
      uatbuf_head++;

      for (i=0; i < keylen; i++) {
        if (pgm_read_byte(&UAT_ident[i]) != uat_ringbuf[(uatbuf_tail + i) % UAT_RINGBUF_SIZE]) {
          break;
        }
      }

      if (i >= keylen) {
        success = true;
        break;
      }
    }
  }

  /* cleanup UAT data buffer */
  uatbuf_head = 0;
  memset(uat_ringbuf, 0, sizeof(uat_ringbuf));

  /* Current ESP32 Core has a bug with Serial2.end()+Serial2.begin() cycle */
  if (SoC->id != SOC_ESP32) {
    UATSerial.end();
  }

  return success;
}

static void uatm_channel(int8_t channel)
{
  /* Nothing to do */
}

static void uatm_setup()
{
  /* Current ESP32 Core has a bug with Serial2.end()+Serial2.begin() cycle */
  if (SoC->id != SOC_ESP32) {
    SoC->UATSerial_begin(UAT_RECEIVER_BR);
  }

  init_fec();

  /* Enforce radio settings to follow UAT978 protocol's RF specs */
  settings->rf_protocol = RF_PROTOCOL_ADSB_UAT;

  RF_FreqPlan.setPlan(settings->band, settings->rf_protocol);

  protocol_encode = &uat978_encode;
  protocol_decode = &uat978_decode;
}

static bool uatm_receive()
{
  bool success = false;
  unsigned int uatbuf_tail;
  int rs_errors;

  while (UATSerial.available()) {
    unsigned char c = UATSerial.read();

    uat_ringbuf[uatbuf_head % UAT_RINGBUF_SIZE] = c;

    uatbuf_tail = uatbuf_head - sizeof(Stratux_frame_t);
    uatbuf_head++;

    if (uat_ringbuf[ uatbuf_tail      % UAT_RINGBUF_SIZE] == STRATUX_UATRADIO_MAGIC_1 &&
        uat_ringbuf[(uatbuf_tail + 1) % UAT_RINGBUF_SIZE] == STRATUX_UATRADIO_MAGIC_2 &&
        uat_ringbuf[(uatbuf_tail + 2) % UAT_RINGBUF_SIZE] == STRATUX_UATRADIO_MAGIC_3 &&
        uat_ringbuf[(uatbuf_tail + 3) % UAT_RINGBUF_SIZE] == STRATUX_UATRADIO_MAGIC_4) {

      unsigned char *pre_fec_buf = (unsigned char *) &uatradio_frame;
      for (u1_t i=0; i < sizeof(Stratux_frame_t); i++) {
          pre_fec_buf[i] = uat_ringbuf[(uatbuf_tail + i) % UAT_RINGBUF_SIZE];
      }

      int frame_type = correct_adsb_frame(uatradio_frame.data, &rs_errors);

      if (frame_type == -1) {
        continue;
      }

      u1_t size = 0;

      if (frame_type == 1) {
        size = SHORT_FRAME_DATA_BYTES;
      } else if (frame_type == 2) {
        size = LONG_FRAME_DATA_BYTES;
      }

      if (size > sizeof(RxBuffer)) {
        size = sizeof(RxBuffer);
      }

      if (size > 0) {
        memcpy(RxBuffer, uatradio_frame.data, size);

        RF_last_rssi = uatradio_frame.rssi;
        rx_packets_counter++;
        success = true;

        break;
      }
    }
  }

  return success;
}

static bool uatm_transmit()
{
  /* Nothing to do */
  return false;
}

static void uatm_shutdown()
{
  /* Nothing to do */
}
#endif /* EXCLUDE_UATM */
