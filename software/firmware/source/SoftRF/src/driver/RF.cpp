/*
 * RFHelper.cpp
 * Copyright (C) 2016-2022 Linar Yusupov
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
#if defined(ARDUINO)
#include <SPI.h>
#endif /* ARDUINO */

#include "RF.h"
#include "../system/SoC.h"
#include "EEPROM.h"
#include "Battery.h"
#include "../ui/Web.h"
#if !defined(EXCLUDE_MAVLINK)
#include "../protocol/data/MAVLink.h"
#endif /* EXCLUDE_MAVLINK */
#include <fec.h>

#if LOGGER_IS_ENABLED
#include "../system/Log.h"
#endif /* LOGGER_IS_ENABLED */

byte RxBuffer[MAX_PKT_SIZE] __attribute__((aligned(sizeof(uint32_t))));

unsigned long TxTimeMarker = 0;
byte TxBuffer[MAX_PKT_SIZE] __attribute__((aligned(sizeof(uint32_t))));

uint32_t tx_packets_counter = 0;
uint32_t rx_packets_counter = 0;

int8_t RF_last_rssi = 0;

FreqPlan RF_FreqPlan;
static bool RF_ready = false;

static size_t RF_tx_size = 0;

const rfchip_ops_t *rf_chip = NULL;
bool RF_SX12XX_RST_is_connected = true;

const char *Protocol_ID[] = {
  [RF_PROTOCOL_LEGACY]    = "LEG",
  [RF_PROTOCOL_OGNTP]     = "OGN",
  [RF_PROTOCOL_P3I]       = "P3I",
  [RF_PROTOCOL_ADSB_1090] = "ADS",
  [RF_PROTOCOL_ADSB_UAT]  = "UAT",
  [RF_PROTOCOL_FANET]     = "FAN"
};

size_t (*protocol_encode)(void *, ufo_t *);
bool   (*protocol_decode)(void *, ufo_t *, ufo_t *);

static Slots_descr_t Time_Slots, *ts;
static uint8_t       RF_timing = RF_TIMING_INTERVAL;

extern const gnss_chip_ops_t *gnss_chip;

static bool nrf905_probe(void);
static void nrf905_setup(void);
static void nrf905_channel(int8_t);
static bool nrf905_receive(void);
static void nrf905_transmit(void);
static void nrf905_shutdown(void);

static bool sx1276_probe(void);
static bool sx1262_probe(void);
static void sx12xx_setup(void);
static void sx12xx_channel(int8_t);
static bool sx12xx_receive(void);
static void sx12xx_transmit(void);
static void sx1276_shutdown(void);
static void sx1262_shutdown(void);

static bool uatm_probe(void);
static void uatm_setup(void);
static void uatm_channel(int8_t);
static bool uatm_receive(void);
static void uatm_transmit(void);
static void uatm_shutdown(void);

static bool cc13xx_probe(void);
static void cc13xx_setup(void);
static void cc13xx_channel(int8_t);
static bool cc13xx_receive(void);
static void cc13xx_transmit(void);
static void cc13xx_shutdown(void);

static bool ognrf_probe(void);
static void ognrf_setup(void);
static void ognrf_channel(int8_t);
static bool ognrf_receive(void);
static void ognrf_transmit(void);
static void ognrf_shutdown(void);

#if !defined(EXCLUDE_NRF905)
const rfchip_ops_t nrf905_ops = {
  RF_IC_NRF905,
  "NRF905",
  nrf905_probe,
  nrf905_setup,
  nrf905_channel,
  nrf905_receive,
  nrf905_transmit,
  nrf905_shutdown
};
#endif
#if !defined(EXCLUDE_SX12XX)
const rfchip_ops_t sx1276_ops = {
  RF_IC_SX1276,
  "SX127x",
  sx1276_probe,
  sx12xx_setup,
  sx12xx_channel,
  sx12xx_receive,
  sx12xx_transmit,
  sx1276_shutdown
};
#if defined(USE_BASICMAC)
const rfchip_ops_t sx1262_ops = {
  RF_IC_SX1262,
  "SX126x",
  sx1262_probe,
  sx12xx_setup,
  sx12xx_channel,
  sx12xx_receive,
  sx12xx_transmit,
  sx1262_shutdown
};
#endif /* USE_BASICMAC */
#endif /*EXCLUDE_SX12XX */
#if !defined(EXCLUDE_UATM)
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
#endif /* EXCLUDE_UATM */
#if !defined(EXCLUDE_CC13XX)
const rfchip_ops_t cc13xx_ops = {
  RF_IC_CC13XX,
  "CC13XX",
  cc13xx_probe,
  cc13xx_setup,
  cc13xx_channel,
  cc13xx_receive,
  cc13xx_transmit,
  cc13xx_shutdown
};
#endif /* EXCLUDE_CC13XX */
#if defined(USE_OGN_RF_DRIVER)

#define vTaskDelay  delay

#if defined(WITH_SI4X32)
#include <rf/si4x32/rfm.h>
#else
#include <rf/combo/rfm.h>
#endif /* WITH_SI4X32 */

const rfchip_ops_t ognrf_ops = {
  RF_DRV_OGN,
  "OGNDRV",
  ognrf_probe,
  ognrf_setup,
  ognrf_channel,
  ognrf_receive,
  ognrf_transmit,
  ognrf_shutdown
};
#endif /* USE_OGN_RF_DRIVER */

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

  /* "AUTO" freq. will set the plan upon very first valid GNSS fix */
  if (settings->band == RF_BAND_AUTO) {
    /* Supersede EU plan with UK when PAW is selected */
    if (rf_chip                &&
#if !defined(EXCLUDE_NRF905)
        rf_chip != &nrf905_ops &&
#endif
        settings->rf_protocol == RF_PROTOCOL_P3I) {
      RF_FreqPlan.setPlan(RF_BAND_UK);
    }
  } else {
    RF_FreqPlan.setPlan(settings->band);
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

  uint8_t OGN = (settings->rf_protocol == RF_PROTOCOL_OGNTP ? 1 : 0);
  int8_t chan = -1;

  switch (RF_timing)
  {
  case RF_TIMING_2SLOTS_PPS_SYNC:
    {
      unsigned long ms_since_boot = millis();
      if ((ms_since_boot - ts->s0.tmarker) >= ts->interval_mid) {
        ts->s0.tmarker = ref_time_ms + ts->s0.begin - ts->adj;
        ts->current = 0;
        chan = (int8_t) RF_FreqPlan.getChannel(Time, ts->current, OGN);
      }
      if ((ms_since_boot - ts->s1.tmarker) >= ts->interval_mid) {
        ts->s1.tmarker = ref_time_ms + ts->s1.begin;
        ts->current = 1;
        chan = (int8_t) RF_FreqPlan.getChannel(Time, ts->current, OGN);
      }
    }
    break;
  case RF_TIMING_INTERVAL:
  default:
    chan = (int8_t) RF_FreqPlan.getChannel(Time, 0, OGN);
    break;
  }

#if DEBUG
  Serial.print("Plan: "); Serial.println(RF_FreqPlan.Plan);
  Serial.print("OGN: "); Serial.println(OGN);
  Serial.print("Channel: "); Serial.println(chan);
#endif

  if (RF_ready && rf_chip) {
    rf_chip->channel(chan);
  }
}

void RF_loop()
{
  if (!RF_ready) {
    if (RF_FreqPlan.Plan == RF_BAND_AUTO) {
      if (ThisAircraft.latitude || ThisAircraft.longitude) {
        RF_FreqPlan.setPlan((int32_t)(ThisAircraft.latitude  * 600000),
                            (int32_t)(ThisAircraft.longitude * 600000));
        RF_ready = true;
      }
    } else {
      RF_ready = true;
    }
  }

  if (RF_ready) {
    RF_SetChannel();
  }
}

size_t RF_Encode(ufo_t *fop)
{
  size_t size = 0;
  if (RF_ready && protocol_encode) {

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
  if (RF_ready && rf_chip && (size > 0)) {
    RF_tx_size = size;

    if (settings->txpower == RF_TX_POWER_OFF ) {
      return true;
    }

    if (!wait || millis() > TxTimeMarker) {

      time_t timestamp = now();

      if (memcmp(TxBuffer, RxBuffer, RF_tx_size) != 0) {

        rf_chip->transmit();

        if (settings->nmea_p) {
          StdOut.print(F("$PSRFO,"));
          StdOut.print((unsigned long) timestamp);
          StdOut.print(F(","));
          StdOut.println(Bin2Hex((byte *) &TxBuffer[0],
                                 RF_Payload_Size(settings->rf_protocol)));
        }
        tx_packets_counter++;

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

  if (RF_ready && rf_chip) {
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
    default:                    return 0;
  }
}

#if !defined(EXCLUDE_NRF905)
/*
 * NRF905-specific code
 *
 *
 */

static int8_t nrf905_channel_prev = (int8_t) -1;
static bool nrf905_receive_active  = false;

static bool nrf905_probe()
{
  uint8_t addr[4];
  uint8_t ref[] = TXADDR;

  digitalWrite(CS_N, HIGH);
  pinMode(CS_N, OUTPUT);

  SoC->SPI_begin();

#if defined(ARDUINO) && !defined(RASPBERRY_PI)
  SPI.setClockDivider(SPI_CLOCK_DIV2);
#endif /* ARDUINO */

  digitalWrite(CS_N, LOW);

  SPI.transfer(NRF905_CMD_R_TX_ADDRESS);
  for(uint8_t i=4;i--;) {
    addr[i] = SPI.transfer(NRF905_CMD_NOP);
  }

  digitalWrite(CS_N, HIGH);
  pinMode(CS_N, INPUT);

  SPI.end();

#if 0
  delay(3000);
  Serial.print("NRF905 probe: ");
  Serial.print(addr[0], HEX); Serial.print(" ");
  Serial.print(addr[1], HEX); Serial.print(" ");
  Serial.print(addr[2], HEX); Serial.print(" ");
  Serial.print(addr[3], HEX); Serial.print(" ");
  Serial.println();
#endif

  /* Cold start state */
  if ((addr[0] == 0xE7) && (addr[1] == 0xE7) && (addr[2] == 0xE7) && (addr[3] == 0xE7)) {
    return true;
  }

  /* Warm restart state */
  if ((addr[0] == 0xE7) && (addr[1] == ref[0]) && (addr[2] == ref[1]) && (addr[3] == ref[2])) {
    return true;
  }

  return false;
}

static void nrf905_channel(int8_t channel)
{
  if (channel != -1 && channel != nrf905_channel_prev) {

    uint32_t frequency;
    nRF905_band_t band;

    frequency = RF_FreqPlan.getChanFrequency((uint8_t) channel);
    band = (frequency >= 844800000UL ? NRF905_BAND_868 : NRF905_BAND_433);

    nRF905_setFrequency(band , frequency);

    nrf905_channel_prev = channel;
    /* restart Rx upon a channel switch */
    nrf905_receive_active = false;
  }
}

static void nrf905_setup()
{
  SoC->SPI_begin();

  // Start up
  nRF905_init();

  /* Channel selection is now part of RF_loop() */
//  nrf905_channel(channel);

  //nRF905_setTransmitPower(NRF905_PWR_10);
  //nRF905_setTransmitPower(NRF905_PWR_n10);

  switch(settings->txpower)
  {
  case RF_TX_POWER_FULL:

    /*
     * NRF905 is unable to give more than 10 dBm
     * 10 dBm is legal everywhere in the world
     */

    nRF905_setTransmitPower((nRF905_pwr_t)NRF905_PWR_10);
    break;
  case RF_TX_POWER_OFF:
  case RF_TX_POWER_LOW:
  default:
    nRF905_setTransmitPower((nRF905_pwr_t)NRF905_PWR_n10);
    break;
  }

  nRF905_setCRC(NRF905_CRC_16);
  //nRF905_setCRC(NRF905_CRC_DISABLE);

  // Set address of this device
  byte addr[] = RXADDR;
  nRF905_setRXAddress(addr);

  /* Enforce radio settings to follow "Legacy" protocol's RF specs */
  settings->rf_protocol = RF_PROTOCOL_LEGACY;

  /* Enforce encoder and decoder to process "Legacy" frames only */
  protocol_encode = &legacy_encode;
  protocol_decode = &legacy_decode;

  /* Put IC into receive mode */
  nRF905_receive();
}

static bool nrf905_receive()
{
  bool success = false;

  // Put into receive mode
  if (!nrf905_receive_active) {
    nRF905_receive();
    nrf905_receive_active = true;
  }

  success = nRF905_getData(RxBuffer, LEGACY_PAYLOAD_SIZE);
  if (success) { // Got data
    rx_packets_counter++;
  }

  return success;
}

static void nrf905_transmit()
{
    nrf905_receive_active = false;

    // Set address of device to send to
    byte addr[] = TXADDR;
    nRF905_setTXAddress(addr);

    // Set payload data
    nRF905_setData(&TxBuffer[0], LEGACY_PAYLOAD_SIZE );

    // Send payload (send fails if other transmissions are going on, keep trying until success)
    while (!nRF905_send()) {
      yield();
    } ;
}

static void nrf905_shutdown()
{
  nRF905_powerDown();
  SPI.end();
}

#endif /* EXCLUDE_NRF905 */

#if !defined(EXCLUDE_SX12XX)
/*
 * SX12XX-specific code
 *
 *
 */

osjob_t sx12xx_txjob;
osjob_t sx12xx_timeoutjob;

static void sx12xx_tx_func (osjob_t* job);
static void sx12xx_rx_func (osjob_t* job);
static void sx12xx_rx(osjobcb_t func);

static bool sx12xx_receive_complete = false;
bool sx12xx_receive_active = false;
static bool sx12xx_transmit_complete = false;

static int8_t sx12xx_channel_prev = (int8_t) -1;

#if defined(USE_BASICMAC)
void os_getDevEui (u1_t* buf) { }
u1_t os_getRegion (void) { return REGCODE_EU868; }
#else
#if !defined(DISABLE_INVERT_IQ_ON_RX)
#error This example requires DISABLE_INVERT_IQ_ON_RX to be set. Update \
       config.h in the lmic library to set it.
#endif
#endif

#define SX1276_RegVersion          0x42 // common

static u1_t sx1276_readReg (u1_t addr) {
#if defined(USE_BASICMAC)
    hal_spi_select(1);
#else
    hal_pin_nss(0);
#endif
    hal_spi(addr & 0x7F);
    u1_t val = hal_spi(0x00);
#if defined(USE_BASICMAC)
    hal_spi_select(0);
#else
    hal_pin_nss(1);
#endif
    return val;
}

static bool sx1276_probe()
{
  u1_t v, v_reset;

  SoC->SPI_begin();

  lmic_hal_init (nullptr);

  // manually reset radio
  hal_pin_rst(0); // drive RST pin low
  hal_waitUntil(os_getTime()+ms2osticks(1)); // wait >100us

  v_reset = sx1276_readReg(SX1276_RegVersion);

  hal_pin_rst(2); // configure RST pin floating!
  hal_waitUntil(os_getTime()+ms2osticks(5)); // wait 5ms

  v = sx1276_readReg(SX1276_RegVersion);

  pinMode(lmic_pins.nss, INPUT);
  SPI.end();

  if (v == 0x12 || v == 0x13) {

    if (v_reset == 0x12 || v_reset == 0x13) {
      RF_SX12XX_RST_is_connected = false;
    }

    return true;
  } else {
    return false;
  }
}

#if defined(USE_BASICMAC)

#define CMD_READREGISTER            0x1D
#define REG_LORASYNCWORDLSB         0x0741
#define SX126X_DEF_LORASYNCWORDLSB  0x24

static void sx1262_ReadRegs (uint16_t addr, uint8_t* data, uint8_t len) {
    hal_spi_select(1);
    hal_pin_busy_wait();
    hal_spi(CMD_READREGISTER);
    hal_spi(addr >> 8);
    hal_spi(addr);
    hal_spi(0x00); // NOP
    for (uint8_t i = 0; i < len; i++) {
        data[i] = hal_spi(0x00);
    }
    hal_spi_select(0);
}

static uint8_t sx1262_ReadReg (uint16_t addr) {
    uint8_t val;
    sx1262_ReadRegs(addr, &val, 1);
    return val;
}

static bool sx1262_probe()
{
  u1_t v, v_reset;

  SoC->SPI_begin();

  lmic_hal_init (nullptr);

  // manually reset radio
  hal_pin_rst(0); // drive RST pin low
  hal_waitUntil(os_getTime()+ms2osticks(1)); // wait >100us

  v_reset = sx1262_ReadReg(REG_LORASYNCWORDLSB);

  hal_pin_rst(2); // configure RST pin floating!
  hal_waitUntil(os_getTime()+ms2osticks(5)); // wait 5ms

  v = sx1262_ReadReg(REG_LORASYNCWORDLSB);

  pinMode(lmic_pins.nss, INPUT);
  SPI.end();

  u1_t fanet_sw_lsb = ((fanet_proto_desc.syncword[0]  & 0x0F) << 4) | 0x04;
  if (v == SX126X_DEF_LORASYNCWORDLSB || v == fanet_sw_lsb) {

    if (v_reset == SX126X_DEF_LORASYNCWORDLSB || v == fanet_sw_lsb) {
      RF_SX12XX_RST_is_connected = false;
    }

    return true;
  } else {
    return false;
  }
}
#endif

static void sx12xx_channel(int8_t channel)
{
  if (channel != -1 && channel != sx12xx_channel_prev) {
    uint32_t frequency = RF_FreqPlan.getChanFrequency((uint8_t) channel);
    int8_t fc = settings->freq_corr;

#if defined(FANET_ZONE2_ENABLE)
    /*
     * NEEDS WORK
     * https://github.com/3s1d/fanet-stm32/commit/0671ce65e5faa06e0e34abce7f35a5b95c1e19db
     *
     * The quick and diry patch below is Ok to apply
     * when (if) FCC certified Skytraxx 'zone #2' devices will appear on the market
     *
     * Better solution is to advance
     * RF_FreqPlan.setPlan(band) -> RF_FreqPlan.setPlan(proto, band)
     */
    if (LMIC.protocol                            &&
        LMIC.protocol->type == RF_PROTOCOL_FANET &&
        settings->band == RF_BAND_US) {
      frequency = 920800000UL; /* 920.8 MHz */
    }
#endif /* FANET_ZONE2_ENABLE */

    //Serial.print("frequency: "); Serial.println(frequency);

    if (sx12xx_receive_active) {
      os_radio(RADIO_RST);
      sx12xx_receive_active = false;
    }

    if (rf_chip->type == RF_IC_SX1276) {
      /* correction of not more than 30 kHz is allowed */
      if (fc > 30) {
        fc = 30;
      } else if (fc < -30) {
        fc = -30;
      };
    } else {
      /* Most of SX1262 designs use TCXO */
      fc = 0;
    }

    /* Actual RF chip's channel registers will be updated before each Tx or Rx session */
    LMIC.freq = frequency + (fc * 1000);
    //LMIC.freq = 868200000UL;

    sx12xx_channel_prev = channel;
  }
}

static void sx12xx_setup()
{
  SoC->SPI_begin();

  // initialize runtime env
  os_init (nullptr);

  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  switch (settings->rf_protocol)
  {
  case RF_PROTOCOL_OGNTP:
    LMIC.protocol = &ogntp_proto_desc;
    protocol_encode = &ogntp_encode;
    protocol_decode = &ogntp_decode;
    break;
  case RF_PROTOCOL_P3I:
    LMIC.protocol = &p3i_proto_desc;
    protocol_encode = &p3i_encode;
    protocol_decode = &p3i_decode;
    break;
  case RF_PROTOCOL_FANET:
    LMIC.protocol = &fanet_proto_desc;
    protocol_encode = &fanet_encode;
    protocol_decode = &fanet_decode;
    break;
  case RF_PROTOCOL_LEGACY:
  default:
    LMIC.protocol = &legacy_proto_desc;
    protocol_encode = &legacy_encode;
    protocol_decode = &legacy_decode;
    /*
     * Enforce legacy protocol setting for SX1276
     * if other value (UAT) left in EEPROM from other (UATM) radio
     */
    settings->rf_protocol = RF_PROTOCOL_LEGACY;
    break;
  }

  switch(settings->txpower)
  {
  case RF_TX_POWER_FULL:

    /* Load regional max. EIRP at first */
    LMIC.txpow = RF_FreqPlan.MaxTxPower;

    if (rf_chip->type == RF_IC_SX1262) {
      /* SX1262 is unable to give more than 22 dBm */
      if (LMIC.txpow > 22)
        LMIC.txpow = 22;
    } else {
      /* SX1276 is unable to give more than 20 dBm */
      if (LMIC.txpow > 20)
        LMIC.txpow = 20;
    }

#if 1
    /*
     * Enforce Tx power limit until confirmation
     * that RFM95W is doing well
     * when antenna is not connected
     */
    if (LMIC.txpow > 17)
      LMIC.txpow = 17;
#endif
    break;
  case RF_TX_POWER_OFF:
  case RF_TX_POWER_LOW:
  default:
    LMIC.txpow = 2; /* 2 dBm is minimum for RFM95W on PA_BOOST pin */
    break;
  }
}

static void sx12xx_setvars()
{
  if (LMIC.protocol && LMIC.protocol->modulation_type == RF_MODULATION_TYPE_LORA) {
    LMIC.datarate = LMIC.protocol->bitrate;
    LMIC.syncword = LMIC.protocol->syncword[0];
  } else {
    LMIC.datarate = DR_FSK;
  }

#if defined(USE_BASICMAC)

#define updr2rps  LMIC_updr2rps

  // LMIC.rps = MAKERPS(sf, BW250, CR_4_5, 0, 0);

  LMIC.noRXIQinversion = true;
  LMIC.rxsyms = 100;

#endif /* USE_BASICMAC */

  // This sets CR 4/5, BW125 (except for DR_SF7B, which uses BW250)
  LMIC.rps = updr2rps(LMIC.datarate);


  if (LMIC.protocol && LMIC.protocol->type == RF_PROTOCOL_FANET) {
    /* for only a few nodes around, increase the coding rate to ensure a more robust transmission */
    LMIC.rps = setCr(LMIC.rps, CR_4_8);
#if defined(FANET_ZONE2_ENABLE)
    if (settings->band == RF_BAND_US) {
      LMIC.rps = setBw(LMIC.rps, BW500);
    }
#endif /* FANET_ZONE2_ENABLE */
  }
}

static bool sx12xx_receive()
{
  bool success = false;

  sx12xx_receive_complete = false;

  if (!sx12xx_receive_active) {
    if (settings->power_save & POWER_SAVE_NORECEIVE) {
      LMIC_shutdown();
    } else {
      sx12xx_setvars();
      sx12xx_rx(sx12xx_rx_func);
    }
    sx12xx_receive_active = true;
  }

  if (sx12xx_receive_complete == false) {
    // execute scheduled jobs and events
    os_runstep();
  };

  if (sx12xx_receive_complete == true) {

    u1_t size = LMIC.dataLen - LMIC.protocol->payload_offset - LMIC.protocol->crc_size;

    if (size >sizeof(RxBuffer)) {
      size = sizeof(RxBuffer);
    }

    for (u1_t i=0; i < size; i++) {
        RxBuffer[i] = LMIC.frame[i + LMIC.protocol->payload_offset];
    }

    RF_last_rssi = LMIC.rssi;
    rx_packets_counter++;
    success = true;
  }

  return success;
}

static void sx12xx_transmit()
{
    sx12xx_transmit_complete = false;
    sx12xx_receive_active = false;

    sx12xx_setvars();
    os_setCallback(&sx12xx_txjob, sx12xx_tx_func);

    unsigned long tx_timeout = LMIC.protocol ? (LMIC.protocol->air_time + 25) : 60;
    unsigned long tx_start   = millis();

    while (sx12xx_transmit_complete == false) {
      if ((millis() - tx_start) > tx_timeout) {
        os_radio(RADIO_RST);
        //Serial.println("TX timeout");
        break;
      }

      // execute scheduled jobs and events
      os_runstep();

      yield();
    };
}

static void sx1276_shutdown()
{
  LMIC_shutdown();

  SPI.end();
}

#if defined(USE_BASICMAC)
static void sx1262_shutdown()
{
  os_init (nullptr);
  sx126x_ll_ops.radio_sleep();
  delay(1);

  SPI.end();
}
#endif /* USE_BASICMAC */

// Enable rx mode and call func when a packet is received
static void sx12xx_rx(osjobcb_t func) {
  LMIC.osjob.func = func;
  LMIC.rxtime = os_getTime(); // RX _now_
  // Enable "continuous" RX for LoRa only (e.g. without a timeout,
  // still stops after receiving a packet)
  os_radio(LMIC.protocol &&
           LMIC.protocol->modulation_type == RF_MODULATION_TYPE_LORA ?
           RADIO_RXON : RADIO_RX);
  //Serial.println("RX");
}

static void sx12xx_rx_func (osjob_t* job) {

  u1_t crc8, pkt_crc8;
  u2_t crc16, pkt_crc16;
  u1_t i;

  // SX1276 is in SLEEP after IRQ handler, Force it to enter RX mode
  sx12xx_receive_active = false;

  /* FANET (LoRa) LMIC IRQ handler may deliver empty packets here when CRC is invalid. */
  if (LMIC.dataLen == 0) {
    return;
  }

  switch (LMIC.protocol->crc_type)
  {
  case RF_CHECKSUM_TYPE_GALLAGER:
  case RF_CHECKSUM_TYPE_NONE:
     /* crc16 left not initialized */
    break;
  case RF_CHECKSUM_TYPE_CRC8_107:
    crc8 = 0x71;     /* seed value */
    break;
  case RF_CHECKSUM_TYPE_CCITT_0000:
    crc16 = 0x0000;  /* seed value */
    break;
  case RF_CHECKSUM_TYPE_CCITT_FFFF:
  default:
    crc16 = 0xffff;  /* seed value */
    break;
  }

  //Serial.print("Got ");
  //Serial.print(LMIC.dataLen);
  //Serial.println(" bytes");

  switch (LMIC.protocol->type)
  {
  case RF_PROTOCOL_LEGACY:
    /* take in account NRF905/FLARM "address" bytes */
    crc16 = update_crc_ccitt(crc16, 0x31);
    crc16 = update_crc_ccitt(crc16, 0xFA);
    crc16 = update_crc_ccitt(crc16, 0xB6);
    break;
  case RF_PROTOCOL_P3I:
  case RF_PROTOCOL_OGNTP:
  default:
    break;
  }

  for (i = LMIC.protocol->payload_offset;
       i < (LMIC.dataLen - LMIC.protocol->crc_size);
       i++)
  {

    switch (LMIC.protocol->crc_type)
    {
    case RF_CHECKSUM_TYPE_GALLAGER:
    case RF_CHECKSUM_TYPE_NONE:
      break;
    case RF_CHECKSUM_TYPE_CRC8_107:
      update_crc8(&crc8, (u1_t)(LMIC.frame[i]));
      break;
    case RF_CHECKSUM_TYPE_CCITT_FFFF:
    case RF_CHECKSUM_TYPE_CCITT_0000:
    default:
      crc16 = update_crc_ccitt(crc16, (u1_t)(LMIC.frame[i]));
      break;
    }

    switch (LMIC.protocol->whitening)
    {
    case RF_WHITENING_NICERF:
      LMIC.frame[i] ^= pgm_read_byte(&whitening_pattern[i - LMIC.protocol->payload_offset]);
      break;
    case RF_WHITENING_MANCHESTER:
    case RF_WHITENING_NONE:
    default:
      break;
    }

#if DEBUG
    Serial.printf("%02x", (u1_t)(LMIC.frame[i]));
#endif
  }

  switch (LMIC.protocol->crc_type)
  {
  case RF_CHECKSUM_TYPE_NONE:
    sx12xx_receive_complete = true;
    break;
  case RF_CHECKSUM_TYPE_GALLAGER:
    if (LDPC_Check((uint8_t  *) &LMIC.frame[0])) {
#if DEBUG
      Serial.printf(" %02x%02x%02x%02x%02x%02x is wrong FEC",
        LMIC.frame[i], LMIC.frame[i+1], LMIC.frame[i+2],
        LMIC.frame[i+3], LMIC.frame[i+4], LMIC.frame[i+5]);
#endif
      sx12xx_receive_complete = false;
    } else {
      sx12xx_receive_complete = true;
    }
    break;
  case RF_CHECKSUM_TYPE_CRC8_107:
    pkt_crc8 = LMIC.frame[i];
#if DEBUG
    if (crc8 == pkt_crc8 ) {
      Serial.printf(" %02x is valid crc", pkt_crc8);
    } else {
      Serial.printf(" %02x is wrong crc", pkt_crc8);
    }
#endif
    if (crc8 == pkt_crc8) {
      sx12xx_receive_complete = true;
    } else {
      sx12xx_receive_complete = false;
    }
    break;
  case RF_CHECKSUM_TYPE_CCITT_FFFF:
  case RF_CHECKSUM_TYPE_CCITT_0000:
  default:
    pkt_crc16 = (LMIC.frame[i] << 8 | LMIC.frame[i+1]);
#if DEBUG
    if (crc16 == pkt_crc16 ) {
      Serial.printf(" %04x is valid crc", pkt_crc16);
    } else {
      Serial.printf(" %04x is wrong crc", pkt_crc16);
    }
#endif
    if (crc16 == pkt_crc16) {
      sx12xx_receive_complete = true;
    } else {
      sx12xx_receive_complete = false;
    }
    break;
  }

#if DEBUG
  Serial.println();
#endif

}

// Transmit the given string and call the given function afterwards
static void sx12xx_tx(unsigned char *buf, size_t size, osjobcb_t func) {

  u1_t crc8;
  u2_t crc16;

  switch (LMIC.protocol->crc_type)
  {
  case RF_CHECKSUM_TYPE_GALLAGER:
  case RF_CHECKSUM_TYPE_NONE:
     /* crc16 left not initialized */
    break;
  case RF_CHECKSUM_TYPE_CRC8_107:
    crc8 = 0x71;     /* seed value */
    break;
  case RF_CHECKSUM_TYPE_CCITT_0000:
    crc16 = 0x0000;  /* seed value */
    break;
  case RF_CHECKSUM_TYPE_CCITT_FFFF:
  default:
    crc16 = 0xffff;  /* seed value */
    break;
  }
  
  os_radio(RADIO_RST); // Stop RX first
  delay(1); // Wait a bit, without this os_radio below asserts, apparently because the state hasn't changed yet

  LMIC.dataLen = 0;

  switch (LMIC.protocol->type)
  {
  case RF_PROTOCOL_LEGACY:
    /* take in account NRF905/FLARM "address" bytes */
    crc16 = update_crc_ccitt(crc16, 0x31);
    crc16 = update_crc_ccitt(crc16, 0xFA);
    crc16 = update_crc_ccitt(crc16, 0xB6);
    break;
  case RF_PROTOCOL_P3I:
    /* insert Net ID */
    LMIC.frame[LMIC.dataLen++] = (u1_t) ((LMIC.protocol->net_id >> 24) & 0x000000FF);
    LMIC.frame[LMIC.dataLen++] = (u1_t) ((LMIC.protocol->net_id >> 16) & 0x000000FF);
    LMIC.frame[LMIC.dataLen++] = (u1_t) ((LMIC.protocol->net_id >>  8) & 0x000000FF);
    LMIC.frame[LMIC.dataLen++] = (u1_t) ((LMIC.protocol->net_id >>  0) & 0x000000FF);
    /* insert byte with payload size */
    LMIC.frame[LMIC.dataLen++] = LMIC.protocol->payload_size;

    /* insert byte with CRC-8 seed value when necessary */
    if (LMIC.protocol->crc_type == RF_CHECKSUM_TYPE_CRC8_107) {
      LMIC.frame[LMIC.dataLen++] = crc8;
    }

    break;
  case RF_PROTOCOL_OGNTP:
  default:
    break;
  }

  for (u1_t i=0; i < size; i++) {

    switch (LMIC.protocol->whitening)
    {
    case RF_WHITENING_NICERF:
      LMIC.frame[LMIC.dataLen] = buf[i] ^ pgm_read_byte(&whitening_pattern[i]);
      break;
    case RF_WHITENING_MANCHESTER:
    case RF_WHITENING_NONE:
    default:
      LMIC.frame[LMIC.dataLen] = buf[i];
      break;
    }

    switch (LMIC.protocol->crc_type)
    {
    case RF_CHECKSUM_TYPE_GALLAGER:
    case RF_CHECKSUM_TYPE_NONE:
      break;
    case RF_CHECKSUM_TYPE_CRC8_107:
      update_crc8(&crc8, (u1_t)(LMIC.frame[LMIC.dataLen]));
      break;
    case RF_CHECKSUM_TYPE_CCITT_FFFF:
    case RF_CHECKSUM_TYPE_CCITT_0000:
    default:
      crc16 = update_crc_ccitt(crc16, (u1_t)(LMIC.frame[LMIC.dataLen]));
      break;
    }

    LMIC.dataLen++;
  }

  switch (LMIC.protocol->crc_type)
  {
  case RF_CHECKSUM_TYPE_GALLAGER:
  case RF_CHECKSUM_TYPE_NONE:
    break;
  case RF_CHECKSUM_TYPE_CRC8_107:
    LMIC.frame[LMIC.dataLen++] = crc8;
    break;
  case RF_CHECKSUM_TYPE_CCITT_FFFF:
  case RF_CHECKSUM_TYPE_CCITT_0000:
  default:
    LMIC.frame[LMIC.dataLen++] = (crc16 >>  8) & 0xFF;
    LMIC.frame[LMIC.dataLen++] = (crc16      ) & 0xFF;
    break;
  }

  LMIC.osjob.func = func;
  os_radio(RADIO_TX);
  //Serial.println("TX");
}

static void sx12xx_txdone_func (osjob_t* job) {
  sx12xx_transmit_complete = true;
}

static void sx12xx_tx_func (osjob_t* job) {

  if (RF_tx_size > 0) {
    sx12xx_tx((unsigned char *) &TxBuffer[0], RF_tx_size, sx12xx_txdone_func);
  }
}
#endif /* EXCLUDE_SX12XX */

#if !defined(EXCLUDE_UATM)
/*
 * UATM-specific code
 *
 *
 */

#include <uat.h>

#define UAT_RINGBUF_SIZE  (sizeof(Stratux_frame_t) * 2)

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

static void uatm_transmit()
{
  /* Nothing to do */
}

static void uatm_shutdown()
{
  /* Nothing to do */
}
#endif /* EXCLUDE_UATM */

#if !defined(EXCLUDE_CC13XX)
/*
 * CC13XX-specific code
 *
 *
 */

#include "EasyLink.h"

#include <uat.h>
#include <fec/char.h>
#include <fec.h>
#include <uat_decode.h>
#include <manchester.h>

#define MAX_SYNCWORD_SIZE       4

const rf_proto_desc_t  *cc13xx_protocol = &uat978_proto_desc;

EasyLink myLink;
#if !defined(EXCLUDE_OGLEP3)
EasyLink_TxPacket txPacket;
#endif /* EXCLUDE_OGLEP3 */

static int8_t cc13xx_channel_prev = (int8_t) -1;

static bool cc13xx_receive_complete  = false;
static bool cc13xx_receive_active    = false;
static bool cc13xx_transmit_complete = false;

void cc13xx_Receive_callback(EasyLink_RxPacket *rxPacket_ptr, EasyLink_Status status)
{
  cc13xx_receive_active = false;
  bool success = false;

  if (status == EasyLink_Status_Success) {

    size_t size = 0;
    uint8_t offset;

    u1_t crc8, pkt_crc8;
    u2_t crc16, pkt_crc16;

#if !defined(EXCLUDE_OGLEP3)
    switch (cc13xx_protocol->crc_type)
    {
    case RF_CHECKSUM_TYPE_GALLAGER:
    case RF_CHECKSUM_TYPE_NONE:
       /* crc16 left not initialized */
      break;
    case RF_CHECKSUM_TYPE_CRC8_107:
      crc8 = 0x71;     /* seed value */
      break;
    case RF_CHECKSUM_TYPE_CCITT_0000:
      crc16 = 0x0000;  /* seed value */
      break;
    case RF_CHECKSUM_TYPE_CCITT_FFFF:
    default:
      crc16 = 0xffff;  /* seed value */
      break;
    }

    switch (cc13xx_protocol->type)
    {
    case RF_PROTOCOL_LEGACY:
      /* take in account NRF905/FLARM "address" bytes */
      crc16 = update_crc_ccitt(crc16, 0x31);
      crc16 = update_crc_ccitt(crc16, 0xFA);
      crc16 = update_crc_ccitt(crc16, 0xB6);
      break;
    case RF_PROTOCOL_P3I:
    case RF_PROTOCOL_OGNTP:
    default:
      break;
    }
#endif /* EXCLUDE_OGLEP3 */

    switch (cc13xx_protocol->type)
    {
#if !defined(EXCLUDE_OGLEP3)
    case RF_PROTOCOL_P3I:
      uint8_t i;
      offset = cc13xx_protocol->payload_offset;
      for (i = 0; i < cc13xx_protocol->payload_size; i++)
      {
        update_crc8(&crc8, (u1_t)(rxPacket_ptr->payload[i + offset]));
        if (i < sizeof(RxBuffer)) {
          RxBuffer[i] = rxPacket_ptr->payload[i + offset] ^
                        pgm_read_byte(&whitening_pattern[i]);
        }
      }

      pkt_crc8 = rxPacket_ptr->payload[i + offset];

      if (crc8 == pkt_crc8) {

        success = true;
      }
      break;
    case RF_PROTOCOL_OGNTP:
    case RF_PROTOCOL_LEGACY:
      offset = cc13xx_protocol->syncword_size - 4;
      size =  cc13xx_protocol->payload_offset +
              cc13xx_protocol->payload_size +
              cc13xx_protocol->payload_size +
              cc13xx_protocol->crc_size +
              cc13xx_protocol->crc_size;
      if (rxPacket_ptr->len >= size + offset &&
          rxPacket_ptr->payload[0] == cc13xx_protocol->syncword[4] &&
          rxPacket_ptr->payload[1] == cc13xx_protocol->syncword[5] &&
          rxPacket_ptr->payload[2] == cc13xx_protocol->syncword[6] &&
          (offset > 3 ? (rxPacket_ptr->payload[3] == cc13xx_protocol->syncword[7]) : true)) {

        uint8_t i, val1, val2;
        for (i = 0; i < size; i++) {
          val1 = pgm_read_byte(&ManchesterDecode[rxPacket_ptr->payload[i + offset]]);
          i++;
          val2 = pgm_read_byte(&ManchesterDecode[rxPacket_ptr->payload[i + offset]]);
          if ((i>>1) < sizeof(RxBuffer)) {
            RxBuffer[i>>1] = ((val1 & 0x0F) << 4) | (val2 & 0x0F);

            if (i < size - (cc13xx_protocol->crc_size + cc13xx_protocol->crc_size)) {
              switch (cc13xx_protocol->crc_type)
              {
              case RF_CHECKSUM_TYPE_GALLAGER:
              case RF_CHECKSUM_TYPE_NONE:
                break;
              case RF_CHECKSUM_TYPE_CCITT_FFFF:
              case RF_CHECKSUM_TYPE_CCITT_0000:
              default:
                crc16 = update_crc_ccitt(crc16, (u1_t)(RxBuffer[i>>1]));
                break;
              }
            }
          }
        }

        switch (cc13xx_protocol->crc_type)
        {
        case RF_CHECKSUM_TYPE_GALLAGER:
          if (LDPC_Check((uint8_t  *) &RxBuffer[0]) == 0) {

            success = true;
          }
          break;
        case RF_CHECKSUM_TYPE_CCITT_FFFF:
        case RF_CHECKSUM_TYPE_CCITT_0000:
          offset = cc13xx_protocol->payload_offset + cc13xx_protocol->payload_size;
          if (offset + 1 < sizeof(RxBuffer)) {
            pkt_crc16 = (RxBuffer[offset] << 8 | RxBuffer[offset+1]);
            if (crc16 == pkt_crc16) {

              success = true;
            }
          }
          break;
        default:
          break;
        }
      }
      break;
#endif /* EXCLUDE_OGLEP3 */
    case RF_PROTOCOL_ADSB_UAT:
    default:
      int rs_errors;
      int frame_type;
      frame_type = correct_adsb_frame(rxPacket_ptr->payload, &rs_errors);

      if (frame_type != -1) {

        if (frame_type == 1) {
          size = SHORT_FRAME_DATA_BYTES;
        } else if (frame_type == 2) {
          size = LONG_FRAME_DATA_BYTES;
        }

        if (size > sizeof(RxBuffer)) {
          size = sizeof(RxBuffer);
        }

        if (size > 0) {
          memcpy(RxBuffer, rxPacket_ptr->payload, size);

          success = true;
        }
      }
      break;
    }

    if (success) {
      RF_last_rssi = rxPacket_ptr->rssi;
      rx_packets_counter++;

      cc13xx_receive_complete  = true;
    }
  }
}

void cc13xx_Transmit_callback(EasyLink_Status status)
{
  if (status == EasyLink_Status_Success) {
    cc13xx_transmit_complete = true;
  }
}

static bool cc13xx_Receive_Async()
{
  bool success = false;
  EasyLink_Status status;

  if (!cc13xx_receive_active) {
    status = myLink.receive(&cc13xx_Receive_callback);

    if (status == EasyLink_Status_Success) {
      cc13xx_receive_active = true;
    }
  }

  if (cc13xx_receive_complete == true) {
    success = true;
    cc13xx_receive_complete = false;
  }

  return success;
}

static bool cc13xx_probe()
{
  bool success = false;

  if (SoC->id == SOC_CC13X0 || SoC->id == SOC_CC13X2) {
    success = true;
  }

  return success;
}

static void cc13xx_channel(int8_t channel)
{
#if !defined(EXCLUDE_OGLEP3)
  if (settings->rf_protocol != RF_PROTOCOL_ADSB_UAT &&
      channel != -1                                 &&
      channel != cc13xx_channel_prev) {
    uint32_t frequency = RF_FreqPlan.getChanFrequency((uint8_t) channel);

    if (cc13xx_receive_active) {
      /* restart Rx upon a channel switch */
      EasyLink_abort();
      cc13xx_receive_active = false;
    }

    EasyLink_setFrequency(frequency);

    cc13xx_channel_prev = channel;
  }
#endif /* EXCLUDE_OGLEP3 */
}

static void cc13xx_setup()
{
  switch (settings->rf_protocol)
  {
#if !defined(EXCLUDE_OGLEP3)
  case RF_PROTOCOL_OGNTP:
    cc13xx_protocol = &ogntp_proto_desc;
    protocol_encode = &ogntp_encode;
    protocol_decode = &ogntp_decode;

    myLink.begin(EasyLink_Phy_100kbps2gfsk_ogntp);
    break;
  case RF_PROTOCOL_P3I:
    cc13xx_protocol = &p3i_proto_desc;
    protocol_encode = &p3i_encode;
    protocol_decode = &p3i_decode;

    myLink.begin(EasyLink_Phy_38400bps2gfsk_p3i);
    break;
  case RF_PROTOCOL_LEGACY:
    cc13xx_protocol = &legacy_proto_desc;
    protocol_encode = &legacy_encode;
    protocol_decode = &legacy_decode;

    myLink.begin(EasyLink_Phy_100kbps2gfsk_legacy);
    break;
#endif /* EXCLUDE_OGLEP3 */
  case RF_PROTOCOL_ADSB_UAT:
  default:
    cc13xx_protocol = &uat978_proto_desc;
    protocol_encode = &uat978_encode;
    protocol_decode = &uat978_decode;
    /*
     * Enforce UAT protocol setting
     * if other value (FANET) left in EEPROM from other (SX12XX) radio
     */
    settings->rf_protocol = RF_PROTOCOL_ADSB_UAT;

    init_fec();
    myLink.begin(EasyLink_Phy_Custom);
    break;
  }

  /* -10 dBm is a minumum for CC1310 ; CC1352 can operate down to -20 dBm */
  int8_t TxPower = -10;

  switch(settings->txpower)
  {
  case RF_TX_POWER_FULL:

    if (settings->rf_protocol != RF_PROTOCOL_ADSB_UAT) {
      /* Load regional max. EIRP at first */
      TxPower = RF_FreqPlan.MaxTxPower;
    }

    if (TxPower > 14)
      TxPower = 14; /* 'high power' CC13XXP (up to 20 dBm) is not supported yet */

    break;
  case RF_TX_POWER_OFF:
  case RF_TX_POWER_LOW:
  default:
    break;
  }

  EasyLink_setRfPwr(TxPower);
}

static bool cc13xx_receive()
{
  return cc13xx_Receive_Async();
}

static void cc13xx_transmit()
{
#if !defined(EXCLUDE_OGLEP3)
  EasyLink_Status status;

  u1_t crc8;
  u2_t crc16;
  u1_t i;

  if (RF_tx_size <= 0) {
    return;
  }

  if (cc13xx_protocol->type == RF_PROTOCOL_ADSB_UAT) {
    return; /* no transmit on UAT */
  }

  EasyLink_abort();

  cc13xx_receive_active = false;
  cc13xx_transmit_complete = false;

  size_t PayloadLen = 0;

  switch (cc13xx_protocol->crc_type)
  {
  case RF_CHECKSUM_TYPE_GALLAGER:
  case RF_CHECKSUM_TYPE_NONE:
     /* crc16 left not initialized */
    break;
  case RF_CHECKSUM_TYPE_CRC8_107:
    crc8 = 0x71;     /* seed value */
    break;
  case RF_CHECKSUM_TYPE_CCITT_0000:
    crc16 = 0x0000;  /* seed value */
    break;
  case RF_CHECKSUM_TYPE_CCITT_FFFF:
  default:
    crc16 = 0xffff;  /* seed value */
    break;
  }

  for (i = MAX_SYNCWORD_SIZE; i < cc13xx_protocol->syncword_size; i++)
  {
    txPacket.payload[PayloadLen++] = cc13xx_protocol->syncword[i];
  }

  switch (cc13xx_protocol->type)
  {
  case RF_PROTOCOL_LEGACY:
    /* take in account NRF905/FLARM "address" bytes */
    crc16 = update_crc_ccitt(crc16, 0x31);
    crc16 = update_crc_ccitt(crc16, 0xFA);
    crc16 = update_crc_ccitt(crc16, 0xB6);
    break;
  case RF_PROTOCOL_P3I:
    /* insert Net ID */
    txPacket.payload[PayloadLen++] = (u1_t) ((cc13xx_protocol->net_id >> 24) & 0x000000FF);
    txPacket.payload[PayloadLen++] = (u1_t) ((cc13xx_protocol->net_id >> 16) & 0x000000FF);
    txPacket.payload[PayloadLen++] = (u1_t) ((cc13xx_protocol->net_id >>  8) & 0x000000FF);
    txPacket.payload[PayloadLen++] = (u1_t) ((cc13xx_protocol->net_id >>  0) & 0x000000FF);
    /* insert byte with payload size */
    txPacket.payload[PayloadLen++] = cc13xx_protocol->payload_size;

    /* insert byte with CRC-8 seed value when necessary */
    if (cc13xx_protocol->crc_type == RF_CHECKSUM_TYPE_CRC8_107) {
      txPacket.payload[PayloadLen++] = crc8;
    }

    break;
  case RF_PROTOCOL_OGNTP:
  default:
    break;
  }

  for (i=0; i < RF_tx_size; i++) {

    switch (cc13xx_protocol->whitening)
    {
    case RF_WHITENING_NICERF:
      txPacket.payload[PayloadLen] = TxBuffer[i] ^ pgm_read_byte(&whitening_pattern[i]);
      break;
    case RF_WHITENING_MANCHESTER:
      txPacket.payload[PayloadLen] = pgm_read_byte(&ManchesterEncode[(TxBuffer[i] >> 4) & 0x0F]);
      PayloadLen++;
      txPacket.payload[PayloadLen] = pgm_read_byte(&ManchesterEncode[(TxBuffer[i]     ) & 0x0F]);
      break;
    case RF_WHITENING_NONE:
    default:
      txPacket.payload[PayloadLen] = TxBuffer[i];
      break;
    }

    switch (cc13xx_protocol->crc_type)
    {
    case RF_CHECKSUM_TYPE_GALLAGER:
    case RF_CHECKSUM_TYPE_NONE:
      break;
    case RF_CHECKSUM_TYPE_CRC8_107:
      update_crc8(&crc8, (u1_t)(txPacket.payload[PayloadLen]));
      break;
    case RF_CHECKSUM_TYPE_CCITT_FFFF:
    case RF_CHECKSUM_TYPE_CCITT_0000:
    default:
      if (cc13xx_protocol->whitening == RF_WHITENING_MANCHESTER) {
        crc16 = update_crc_ccitt(crc16, (u1_t)(TxBuffer[i]));
      } else {
        crc16 = update_crc_ccitt(crc16, (u1_t)(txPacket.payload[PayloadLen]));
      }
      break;
    }

    PayloadLen++;
  }

  switch (cc13xx_protocol->crc_type)
  {
  case RF_CHECKSUM_TYPE_GALLAGER:
  case RF_CHECKSUM_TYPE_NONE:
    break;
  case RF_CHECKSUM_TYPE_CRC8_107:
    txPacket.payload[PayloadLen++] = crc8;
    break;
  case RF_CHECKSUM_TYPE_CCITT_FFFF:
  case RF_CHECKSUM_TYPE_CCITT_0000:
  default:
    if (cc13xx_protocol->whitening == RF_WHITENING_MANCHESTER) {
      txPacket.payload[PayloadLen++] = pgm_read_byte(&ManchesterEncode[(((crc16 >>  8) & 0xFF) >> 4) & 0x0F]);
      txPacket.payload[PayloadLen++] = pgm_read_byte(&ManchesterEncode[(((crc16 >>  8) & 0xFF)     ) & 0x0F]);
      txPacket.payload[PayloadLen++] = pgm_read_byte(&ManchesterEncode[(((crc16      ) & 0xFF) >> 4) & 0x0F]);
      txPacket.payload[PayloadLen++] = pgm_read_byte(&ManchesterEncode[(((crc16      ) & 0xFF)     ) & 0x0F]);
      PayloadLen++;
    } else {
      txPacket.payload[PayloadLen++] = (crc16 >>  8) & 0xFF;
      txPacket.payload[PayloadLen++] = (crc16      ) & 0xFF;
    }
    break;
  }

  txPacket.len = PayloadLen;
  // Transmit immediately
  txPacket.absTime = EasyLink_ms_To_RadioTime(0);

#if 0
  status = myLink.transmit(&txPacket, &cc13xx_Transmit_callback);

  if (status == EasyLink_Status_Success) {
    while (cc13xx_transmit_complete == false) {
      yield();
    };
  }
#else
  myLink.transmit(&txPacket);
#endif

#endif /* EXCLUDE_OGLEP3 */
}

static void cc13xx_shutdown()
{
  EasyLink_abort();
}
#endif /* EXCLUDE_CC13XX */

#if defined(USE_OGN_RF_DRIVER)
/*
 * OGN driver specific code
 *
 *
 */

static RFM_TRX  TRX;

static int8_t ognrf_channel_prev  = (int8_t) -1;
static bool ognrf_receive_active   = false;

void RFM_Select  (void)                 { hal_pin_nss(0); }
void RFM_Deselect(void)                 { hal_pin_nss(1); }
uint8_t RFM_TransferByte(uint8_t Byte)  { return hal_spi(Byte); }

bool RFM_IRQ_isOn(void)   { return lmic_pins.dio[0] == LMIC_UNUSED_PIN ? \
                                  false : digitalRead(lmic_pins.dio[0]); }

#ifdef WITH_RFM95                     // RESET is active LOW
void RFM_RESET(uint8_t On)
{ if(On) hal_pin_rst(0);
    else hal_pin_rst(1); }
#endif

#if defined(WITH_RFM69) || defined(WITH_SX1272) // RESET is active HIGH
void RFM_RESET(uint8_t On)
{ if(On) hal_pin_rst(1);
    else hal_pin_rst(0); }
#endif

static bool ognrf_probe()
{
  bool success = false;

  TRX.Select       = RFM_Select;
  TRX.Deselect     = RFM_Deselect;
  TRX.TransferByte = RFM_TransferByte;
  TRX.RESET        = RFM_RESET;

  SoC->SPI_begin();
  lmic_hal_init (nullptr);

  TRX.RESET(1);                      // RESET active
  vTaskDelay(10);                    // wait 10ms
  TRX.RESET(0);                      // RESET released
  vTaskDelay(10);                    // wait 10ms

  uint8_t ChipVersion = TRX.ReadVersion();

  pinMode(lmic_pins.nss, INPUT);
  SPI.end();

#if defined(WITH_RFM95)
  if (ChipVersion == 0x12 || ChipVersion == 0x13) success = true;
#endif /* WITH_RFM95 */
#if defined(WITH_RFM69)
  if (ChipVersion == 0x24) success = true;
#endif /* WITH_RFM69 */
#if defined(WITH_SX1272)
  if (ChipVersion == 0x22) success = true;
#endif /* WITH_SX1272 */
#if defined(WITH_SI4X32)
  if (ChipVersion == 0x06 /* 4032 */ ||
      ChipVersion == 0x08 /* 4432 */ ) success = true;
#endif /* WITH_SI4X32 */

  return success;
}

static void ognrf_channel(int8_t channel)
{
  if (channel != -1 && channel != ognrf_channel_prev) {

    if (ognrf_receive_active) {

      TRX.WriteMode(RF_OPMODE_STANDBY);
      vTaskDelay(1);

      /* restart Rx upon a channel switch */
      ognrf_receive_active = false;
    }

    TRX.setChannel(channel & 0x7F);

    ognrf_channel_prev = channel;
  }
}

static void ognrf_setup()
{
  uint8_t TxPower = 0;

  /* Enforce radio settings to follow OGNTP protocol's RF specs */
  settings->rf_protocol = RF_PROTOCOL_OGNTP;

  LMIC.protocol = &ogntp_proto_desc;

  protocol_encode = &ogntp_encode;
  protocol_decode = &ogntp_decode;

  TRX.Select       = RFM_Select;
  TRX.Deselect     = RFM_Deselect;
  TRX.TransferByte = RFM_TransferByte;
  TRX.DIO0_isOn    = RFM_IRQ_isOn;
  TRX.RESET        = RFM_RESET;

  SoC->SPI_begin();
  lmic_hal_init (nullptr);

  TRX.RESET(1);                      // RESET active
  vTaskDelay(10);                    // wait 10ms
  TRX.RESET(0);                      // RESET released
  vTaskDelay(10);                    // wait 10ms

  // set TRX base frequency and channel separation
  TRX.setBaseFrequency(RF_FreqPlan.BaseFreq);
  TRX.setChannelSpacing(RF_FreqPlan.ChanSepar);
  TRX.setFrequencyCorrection(0);

  TRX.Configure(0, ogntp_proto_desc.syncword);  // setup RF chip parameters and set to channel #0
  TRX.WriteMode(RF_OPMODE_STANDBY);             // set RF chip mode to STANDBY

  switch(settings->txpower)
  {
  case RF_TX_POWER_FULL:

    /* Load regional max. EIRP at first */
    TxPower = RF_FreqPlan.MaxTxPower;

    if (TxPower > 20)
      TxPower = 20;
#if 1
    if (TxPower > 17)
      TxPower = 17;
#endif

#ifdef WITH_RFM69
    TRX.WriteTxPower(TxPower, RFM69_POWER_RATING == 1 ? true : false);
#else
    TRX.WriteTxPower(TxPower);
#endif /* WITH_RFM69 */
    break;
  case RF_TX_POWER_OFF:
  case RF_TX_POWER_LOW:
  default:
    TRX.WriteTxPowerMin();
    break;
  }

  /* Leave IC in standby mode */
}

static bool ognrf_receive()
{
  bool success = false;

#if !defined(WITH_SI4X32)

  uint8_t RxRSSI = 0;
  uint8_t Err [OGNTP_PAYLOAD_SIZE + OGNTP_CRC_SIZE];

  // Put into receive mode
  if (!ognrf_receive_active) {

//    TRX.ClearIrqFlags();

    TRX.WriteSYNC(7, 7, ogntp_proto_desc.syncword); // Shorter SYNC for RX
    TRX.WriteMode(RF_OPMODE_RECEIVER);
    vTaskDelay(1);

    ognrf_receive_active = true;
  }

  if(TRX.DIO0_isOn()) {
    RxRSSI = TRX.ReadRSSI();

    TRX.ReadPacket(RxBuffer, Err);
    if (LDPC_Check((uint8_t  *) RxBuffer) == 0) {
      success = true;
    }
  }

  if (success) {
    RF_last_rssi = RxRSSI;
    rx_packets_counter++;
  }

#endif /* WITH_SI4X32 */

  return success;
}

static void ognrf_transmit()
{
  ognrf_receive_active = false;

#if defined(WITH_SI4X32)

  TRX.WritePacket((uint8_t *) &TxBuffer[0]);
  TRX.Transmit();
  vTaskDelay(6);

#else

  TRX.WriteMode(RF_OPMODE_STANDBY);
  vTaskDelay(1);

  TRX.WriteSYNC(8, 7, ogntp_proto_desc.syncword);            // Full SYNC for TX

  TRX.ClearIrqFlags();
  TRX.WritePacket((uint8_t *) &TxBuffer[0]);

  TRX.WriteMode(RF_OPMODE_TRANSMITTER);
  vTaskDelay(5);

  uint8_t Break=0;
  for(uint16_t Wait=400; Wait; Wait--)        // wait for transmission to end
  {
    uint16_t Flags=TRX.ReadIrqFlags();
    if(Flags&RF_IRQ_PacketSent) Break++;
    if(Break>=2) break;
  }

  TRX.WriteMode(RF_OPMODE_STANDBY);

#endif /* WITH_SI4X32 */
}

static void ognrf_shutdown()
{
  TRX.WriteMode(RF_OPMODE_STANDBY);
  SPI.end();

  pinMode(lmic_pins.nss, INPUT);
}

#endif /* USE_OGN_RF_DRIVER */
