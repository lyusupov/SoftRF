/*
 * RFHelper.cpp
 * Copyright (C) 2016-2025 Linar Yusupov
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

#include "RF.h"
#include "EEPROM.h"
#if !defined(EXCLUDE_MAVLINK)
#include "../protocol/data/MAVLink.h"
#endif /* EXCLUDE_MAVLINK */

byte RxBuffer[MAX_PKT_SIZE] __attribute__((aligned(sizeof(uint32_t))));

unsigned long TxTimeMarker = 0;
byte TxBuffer[MAX_PKT_SIZE] __attribute__((aligned(sizeof(uint32_t))));

uint32_t tx_packets_counter = 0;
uint32_t rx_packets_counter = 0;

int8_t RF_last_rssi = 0;

FreqPlan RF_FreqPlan;

size_t RF_tx_size = 0;

const rfchip_ops_t *rf_chip = NULL;
bool RF_SX12XX_RST_is_connected = true;

const char *Protocol_ID[] = {
  [RF_PROTOCOL_LEGACY]    = "LEG",
  [RF_PROTOCOL_OGNTP]     = "OGN",
  [RF_PROTOCOL_P3I]       = "P3I",
  [RF_PROTOCOL_ADSB_1090] = "ES",
  [RF_PROTOCOL_ADSB_UAT]  = "UAT",
  [RF_PROTOCOL_FANET]     = "FAN",
  [RF_PROTOCOL_APRS]      = "HAM",
  [RF_PROTOCOL_ADSL_860]  = "ADL",
};

size_t (*protocol_encode)(void *, ufo_t *);
bool   (*protocol_decode)(void *, ufo_t *, ufo_t *);

static Slots_descr_t Time_Slots, *ts;
static uint8_t       RF_timing = RF_TIMING_INTERVAL;

extern const gnss_chip_ops_t *gnss_chip;

String Bin2Hex(byte *buffer, size_t size)
{
  String str = "";
  for (int i=0; i < size; i++) {
    byte c = buffer[i];
    str += (c < 0x10 ? "0" : "") + String(c, HEX);
  }
  return str;
}

uint8_t parity(uint32_t x) {
    uint8_t parity=0;
    while (x > 0) {
      if (x & 0x1) {
          parity++;
      }
      x >>= 1;
    }
    return (parity % 2);
}
 
byte RF_setup(void)
{

  if (rf_chip == NULL) {
#if !defined(USE_OGN_RF_DRIVER)
#if !defined(EXCLUDE_SX12XX)
#if !defined(EXCLUDE_SX1276)
    if (sx1276_ops.probe()) {
      rf_chip = &sx1276_ops;
#else
    if (false) {
#endif
#if defined(USE_BASICMAC)
#if !defined(EXCLUDE_SX1276)
      SX12XX_LL = &sx127x_ll_ops;
#endif
    } else if (sx1262_ops.probe()) {
      rf_chip = &sx1262_ops;
      SX12XX_LL = &sx126x_ll_ops;
#endif /* USE_BASICMAC */
#else
    if (false) {
#endif /* EXCLUDE_SX12XX */
#if defined(USE_RADIOLIB)
#if !defined(EXCLUDE_LR11XX)
    } else if (lr1110_ops.probe()) {
      rf_chip = &lr1110_ops;
    } else if (lr1121_ops.probe()) {
      rf_chip = &lr1121_ops;
#endif /* EXCLUDE_LR11XX */
#if !defined(EXCLUDE_CC1101)
    } else if (cc1101_ops.probe()) {
      rf_chip = &cc1101_ops;
#endif /* EXCLUDE_CC1101 */
#endif /* USE_RADIOLIB */
#if !defined(EXCLUDE_NRF905)
    } else if (nrf905_ops.probe()) {
      rf_chip = &nrf905_ops;
#endif /* EXCLUDE_NRF905 */
#if !defined(EXCLUDE_UATM)
    } else if (uatm_ops.probe()) {
      rf_chip = &uatm_ops;
#endif /* EXCLUDE_UATM */
#if !defined(EXCLUDE_CC13XX)
    } else if (cc13xx_ops.probe()) {
      rf_chip = &cc13xx_ops;
#endif /* EXCLUDE_CC13XX */
#if defined(USE_SA8X8)
    } else if (sa8x8_ops.probe()) {
      rf_chip = &sa8x8_ops;
#endif /* USE_SA8X8 */
    }
    if (rf_chip && rf_chip->name) {
      Serial.print(rf_chip->name);
      Serial.println(F(" RFIC is detected."));
    } else {
      Serial.println(F("WARNING! None of supported RFICs is detected!"));
    }
#else /* USE_OGN_RF_DRIVER */
    if (ognrf_ops.probe()) {
      rf_chip = &ognrf_ops;
      Serial.println(F("OGN_DRV: RFIC is detected."));
    } else {
      Serial.println(F("WARNING! RFIC is NOT detected."));
    }
#endif /* USE_OGN_RF_DRIVER */
  }

  if (rf_chip) {
    rf_chip->setup();

    const rf_proto_desc_t *p;

    switch (settings->rf_protocol)
    {
      case RF_PROTOCOL_OGNTP:     p = &ogntp_proto_desc;  break;
      case RF_PROTOCOL_P3I:       p = &p3i_proto_desc;    break;
      case RF_PROTOCOL_FANET:     p = &fanet_proto_desc;  break;
      case RF_PROTOCOL_ADSB_UAT:  p = &uat978_proto_desc; break;
      case RF_PROTOCOL_ADSB_1090: p = &es1090_proto_desc; break;
#if defined(ENABLE_ADSL)
      case RF_PROTOCOL_ADSL_860:  p = &adsl_proto_desc;   break;
#endif /* ENABLE_ADSL */
      case RF_PROTOCOL_APRS:
#if defined(ENABLE_PROL)
        if (rf_chip->type == RF_IC_SX1276 || rf_chip->type == RF_IC_SX1262) {
          p = &prol_proto_desc;
        } else
#endif /* ENABLE_PROL */
        {
          p = &aprs_proto_desc;
        }
        break;
      case RF_PROTOCOL_LEGACY:
      default:                    p = &legacy_proto_desc; break;
    }

    RF_timing         = p->tm_type;

    ts                = &Time_Slots;
    ts->air_time      = p->air_time;
    ts->interval_min  = p->tx_interval_min;
    ts->interval_max  = p->tx_interval_max;
    ts->interval_mid  = (p->tx_interval_max + p->tx_interval_min) / 2;
    ts->s0.begin      = p->slot0.begin;
    ts->s1.begin      = p->slot1.begin;
    ts->s0.duration   = p->slot0.end - p->slot0.begin;
    ts->s1.duration   = p->slot1.end - p->slot1.begin;

    uint16_t duration = ts->s0.duration + ts->s1.duration;
    ts->adj = duration > ts->interval_mid ? 0 : (ts->interval_mid - duration) / 2;

    return rf_chip->type;
  } else {
    return RF_IC_NONE;
  }
}

void RF_SetChannel(void)
{
  tmElements_t  tm;
  time_t        Time;
  unsigned long pps_btime_ms, ref_time_ms;

  switch (settings->mode)
  {
  case SOFTRF_MODE_TXRX_TEST:
    Time = now();
    RF_timing = RF_timing == RF_TIMING_2SLOTS_PPS_SYNC ?
                RF_TIMING_INTERVAL : RF_timing;
    break;
#if !defined(EXCLUDE_MAVLINK)
  case SOFTRF_MODE_UAV:
    Time = the_aircraft.location.gps_time_stamp / 1000000;
    RF_timing = RF_timing == RF_TIMING_2SLOTS_PPS_SYNC ?
                RF_TIMING_INTERVAL : RF_timing;
    break;
#endif /* EXCLUDE_MAVLINK */
  case SOFTRF_MODE_NORMAL:
  default:
    pps_btime_ms = SoC->get_PPS_TimeMarker();
    unsigned long time_corr_neg;
    unsigned long ms_since_boot = millis();

    if (pps_btime_ms) {
      unsigned long last_Commit_Time = ms_since_boot - gnss.time.age();
      if (pps_btime_ms <= last_Commit_Time) {
        time_corr_neg = (last_Commit_Time - pps_btime_ms) % 1000;
      } else {
        time_corr_neg = 1000 - ((pps_btime_ms - last_Commit_Time) % 1000);
      }
      ref_time_ms = (ms_since_boot - pps_btime_ms) <= 1010 ?
                    pps_btime_ms :
                    ms_since_boot-(ms_since_boot % 1000)+(pps_btime_ms % 1000);
    } else {
      unsigned long last_RMC_Commit = ms_since_boot - gnss.date.age();
      time_corr_neg = gnss_chip ? gnss_chip->rmc_ms : 100;
      ref_time_ms = last_RMC_Commit - time_corr_neg;
    }

    int yr    = gnss.date.year();
    if( yr > 99)
        yr    = yr - 1970;
    else
        yr    += 30;
    tm.Year   = yr;
    tm.Month  = gnss.date.month();
    tm.Day    = gnss.date.day();
    tm.Hour   = gnss.time.hour();
    tm.Minute = gnss.time.minute();
    tm.Second = gnss.time.second();

    Time = makeTime(tm) + (gnss.time.age() - time_corr_neg) / 1000;
    break;
  }

  uint8_t OGN  = (settings->rf_protocol == RF_PROTOCOL_OGNTP    ? 1 : 0);
  uint8_t ADSL = (settings->rf_protocol == RF_PROTOCOL_ADSL_860 ? 1 : 0);
  int8_t chan  = -1;

  switch (RF_timing)
  {
  case RF_TIMING_2SLOTS_PPS_SYNC:
    {
      unsigned long ms_since_boot = millis();
      if ((ms_since_boot - ts->s0.tmarker) >= ts->interval_mid) {
        ts->s0.tmarker = ref_time_ms + ts->s0.begin - ts->adj;
        ts->current = 0;
        chan = (int8_t) RF_FreqPlan.getChannel(Time, ts->current, OGN | ADSL);
      }
      if ((ms_since_boot - ts->s1.tmarker) >= ts->interval_mid) {
        ts->s1.tmarker = ref_time_ms + ts->s1.begin;
        ts->current = 1;
        chan = (int8_t) RF_FreqPlan.getChannel(Time, ts->current, OGN | ADSL);
      }
    }
    break;
  case RF_TIMING_INTERVAL:
  default:
    chan = (int8_t) RF_FreqPlan.getChannel(Time, 0, OGN | ADSL);
    break;
  }

#if DEBUG
  Serial.print("Plan: "); Serial.println(RF_FreqPlan.Plan);
  Serial.print("OGN: "); Serial.println(OGN);
  Serial.print("ADSL: "); Serial.println(ADSL);
  Serial.print("Channel: "); Serial.println(chan);
#endif

  if (rf_chip) {
    rf_chip->channel(chan);
  }
}

void RF_loop()
{
  RF_SetChannel();
}

size_t RF_Encode(ufo_t *fop)
{
  size_t size = 0;
  if (protocol_encode) {

    if (settings->txpower == RF_TX_POWER_OFF ) {
      return size;
    }

    if (millis() > TxTimeMarker) {
      size = (*protocol_encode)((void *) &TxBuffer[0], fop);
    }
  }
  return size;
}

bool RF_Transmit(size_t size, bool wait)
{
  if (rf_chip && (size > 0)) {
    RF_tx_size = size;

    if (settings->txpower == RF_TX_POWER_OFF ) {
      return true;
    }

    if (!wait || millis() > TxTimeMarker) {

      time_t timestamp = now();

      if (memcmp(TxBuffer, RxBuffer, RF_tx_size) != 0) {

        if (rf_chip->transmit()) {
          if (settings->nmea_p) {
            StdOut.print(F("$PSRFO,"));
            StdOut.print((unsigned long) timestamp);
            StdOut.print(F(","));
            StdOut.println(Bin2Hex((byte *) &TxBuffer[0],
                                   RF_Payload_Size(settings->rf_protocol)));
          }
          tx_packets_counter++;
        }
      } else {

        if (settings->nmea_p) {
          StdOut.println(F("$PSRFE,RF loopback is detected on Tx"));
        }
      }

      RF_tx_size = 0;

      Slot_descr_t *next;
      unsigned long adj;

      switch (RF_timing)
      {
      case RF_TIMING_2SLOTS_PPS_SYNC:
        next = RF_FreqPlan.Channels == 1 ? &(ts->s0) :
               ts->current          == 1 ? &(ts->s0) : &(ts->s1);
        adj  = ts->current ? ts->adj   : 0;
        TxTimeMarker = next->tmarker    +
                       ts->interval_mid +
                       SoC->random(adj, next->duration - ts->air_time);
        break;
      case RF_TIMING_INTERVAL:
      default:
        TxTimeMarker = millis() + SoC->random(ts->interval_min, ts->interval_max) - ts->air_time;
        break;
      }

      return true;
    }
  }
  return false;
}

bool RF_Receive(void)
{
  bool rval = false;

  if (rf_chip) {
    rval = rf_chip->receive();
  }
  
  return rval;
}

void RF_Shutdown(void)
{
  if (rf_chip) {
    rf_chip->shutdown();
  }
}

uint8_t RF_Payload_Size(uint8_t protocol)
{
  switch (protocol)
  {
    case RF_PROTOCOL_LEGACY:    return legacy_proto_desc.payload_size;
    case RF_PROTOCOL_OGNTP:     return ogntp_proto_desc.payload_size;
    case RF_PROTOCOL_P3I:       return p3i_proto_desc.payload_size;
    case RF_PROTOCOL_FANET:     return fanet_proto_desc.payload_size;
    case RF_PROTOCOL_ADSB_UAT:  return uat978_proto_desc.payload_size;
    case RF_PROTOCOL_ADSB_1090: return es1090_proto_desc.payload_size;
#if defined(ENABLE_ADSL)
    case RF_PROTOCOL_ADSL_860:  return adsl_proto_desc.payload_size;
#endif /* ENABLE_ADSL */
    case RF_PROTOCOL_APRS:
#if defined(ENABLE_PROL)
      if (rf_chip->type == RF_IC_SX1276 || rf_chip->type == RF_IC_SX1262) {
        return prol_proto_desc.payload_size;
      } else
#endif /* ENABLE_PROL */
      {
        return aprs_proto_desc.payload_size;
      }
    default:                    return 0;
  }
}
