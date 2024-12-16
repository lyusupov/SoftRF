/*
 * easylink.cpp
 * Copyright (C) 2020-2025 Linar Yusupov
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

#if !defined(EXCLUDE_CC13XX)
/*
 * CC13XX-specific code
 *
 *
 */

#include "../EEPROM.h"

#include "EasyLink.h"

#include <uat.h>
#include <fec/char.h>
#include <fec.h>
#include <uat_decode.h>
#include <manchester.h>

#define MAX_SYNCWORD_SIZE       4

extern size_t RF_tx_size;

static bool cc13xx_probe(void);
static void cc13xx_setup(void);
static void cc13xx_channel(int8_t);
static bool cc13xx_receive(void);
static bool cc13xx_transmit(void);
static void cc13xx_shutdown(void);

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
    case RF_CHECKSUM_TYPE_CRC_MODES:
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
    case RF_PROTOCOL_ADSL_860:
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
    case RF_PROTOCOL_ADSL_860:
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
              case RF_CHECKSUM_TYPE_CRC_MODES:
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

        size = size>>1;

        switch (cc13xx_protocol->crc_type)
        {
        case RF_CHECKSUM_TYPE_GALLAGER:
          if (LDPC_Check((uint8_t  *) &RxBuffer[0]) == 0) {
            success = true;
          }
          break;
        case RF_CHECKSUM_TYPE_CRC_MODES:
#if defined(ENABLE_ADSL)
          if (ADSL_Packet::checkPI((uint8_t  *) &RxBuffer[0], size) == 0) {
            success = true;
          }
#endif /* ENABLE_ADSL */
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

    if (settings->rf_protocol == RF_PROTOCOL_LEGACY) {
      nRF905_band_t nrf_band;
      uint32_t nrf_freq_resolution;

      nrf_band = (frequency >= 844800000UL ? NRF905_BAND_868 : NRF905_BAND_433);
      nrf_freq_resolution = (nrf_band == NRF905_BAND_433 ? 100000UL : 200000UL);
      frequency -= (frequency % nrf_freq_resolution);
    }

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
#if defined(ENABLE_ADSL)
  case RF_PROTOCOL_ADSL_860:
    cc13xx_protocol = &adsl_proto_desc;
    protocol_encode = &adsl_encode;
    protocol_decode = &adsl_decode;

    myLink.begin(EasyLink_Phy_100kbps2gfsk_adsl);
    break;
#endif /* ENABLE_ADSL */
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

  RF_FreqPlan.setPlan(settings->band, settings->rf_protocol);

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

static bool cc13xx_transmit()
{
#if defined(EXCLUDE_OGLEP3)
  return false;
#else
  EasyLink_Status status;

  u1_t crc8;
  u2_t crc16;
  u1_t i;

  if (RF_tx_size <= 0) {
    return false;
  }

  if (cc13xx_protocol->type == RF_PROTOCOL_ADSB_UAT) {
    return false; /* no transmit on UAT */
  }

  EasyLink_abort();

  cc13xx_receive_active = false;
  cc13xx_transmit_complete = false;

  size_t PayloadLen = 0;

  switch (cc13xx_protocol->crc_type)
  {
  case RF_CHECKSUM_TYPE_GALLAGER:
  case RF_CHECKSUM_TYPE_CRC_MODES:
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
  case RF_PROTOCOL_ADSL_860:
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
    case RF_CHECKSUM_TYPE_CRC_MODES:
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
  case RF_CHECKSUM_TYPE_CRC_MODES:
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
  return true;
#endif /* EXCLUDE_OGLEP3 */
}

static void cc13xx_shutdown()
{
  EasyLink_abort();
}
#endif /* EXCLUDE_CC13XX */
