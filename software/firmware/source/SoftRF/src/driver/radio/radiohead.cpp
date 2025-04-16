/*
 * radiohead.cpp
 * Copyright (C) 2025 Linar Yusupov
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

#if defined(USE_RADIOHEAD)

#include "../EEPROM.h"
#include "../Battery.h"

#include <manchester.h>

#ifndef RadioSPI
#define RadioSPI        SPI
#endif

const rf_proto_desc_t *rh_protocol = &ogntp_proto_desc;

#define RADIOHEAD_MAX_DATA_LENGTH   128

typedef struct
{
  uint8_t len;
  uint8_t payload[RADIOHEAD_MAX_DATA_LENGTH];
} RadioHead_DataPacket;

static RadioHead_DataPacket RH_txPacket;
static RadioHead_DataPacket RH_rxPacket;

extern size_t RF_tx_size;

#if !defined(USE_BASICMAC)
#include <SPI.h>

static const SPISettings probe_settings(1000000UL, MSBFIRST, SPI_MODE0);

static void hal_spi_select (int on) {

#if defined(SPI_HAS_TRANSACTION)
    if (on)
        RadioSPI.beginTransaction(probe_settings);
    else
        RadioSPI.endTransaction();
#endif

    //Serial.println(val?">>":"<<");
    digitalWrite(lmic_pins.nss, !on ? HIGH : LOW);
}

// Datasheet defins typical times until busy goes low. Most are < 200us,
// except when waking up from sleep, which typically takes 3500us. Since
// we cannot know here if we are in sleep, we'll have to assume we are.
// Since 3500 is typical, not maximum, wait a bit more than that.
static unsigned long MAX_BUSY_TIME = 5000;

static void hal_pin_busy_wait (void) {
    if (lmic_pins.busy == LMIC_UNUSED_PIN) {
        // TODO: We could probably keep some state so we know the chip
        // is in sleep, since otherwise the delay can be much shorter.
        // Also, all delays after commands (rather than waking up from
        // sleep) are measured from the *end* of the previous SPI
        // transaction, so we could wait shorter if we remember when
        // that was.
        delayMicroseconds(MAX_BUSY_TIME);
    } else {
        unsigned long start = micros();

        while((micros() - start) < MAX_BUSY_TIME && digitalRead(lmic_pins.busy)) /* wait */;
    }
}
#endif /* USE_BASICMAC */

static bool memeqzero(const uint8_t *data, size_t length)
{
	const uint8_t *p = data;
	size_t len;

	/* Check first 16 bytes manually */
	for (len = 0; len < 16; len++) {
		if (!length)
			return true;
		if (*p)
			return false;
		p++;
		length--;
	}

	/* Now we know that's zero, memcmp with self. */
	return memcmp((void *) data, (void *) p, length) == 0;
}

#if !defined(EXCLUDE_SI446X)

#include <RH_RF24.h>

#define RADIOHEAD_SI443X_DEVICE_VERSION     0x06 //  4 0  chip version register

static bool si4463_probe(void);
static void si4463_setup(void);
static void si4463_channel(int8_t);
static bool si4463_receive(void);
static bool si4463_transmit(void);
static void si4463_shutdown(void);

const rfchip_ops_t si4463_ops = {
  RF_IC_SI4463,
  "Si4463",
  si4463_probe,
  si4463_setup,
  si4463_channel,
  si4463_receive,
  si4463_transmit,
  si4463_shutdown
};

static RH_RF24 *radio_silabs;

static int8_t si4463_channel_prev    = (int8_t) -1;

static bool si4463_receive_active    = false;

static const uint8_t byte_rev_table[256] = {
	0x00, 0x80, 0x40, 0xc0, 0x20, 0xa0, 0x60, 0xe0,
	0x10, 0x90, 0x50, 0xd0, 0x30, 0xb0, 0x70, 0xf0,
	0x08, 0x88, 0x48, 0xc8, 0x28, 0xa8, 0x68, 0xe8,
	0x18, 0x98, 0x58, 0xd8, 0x38, 0xb8, 0x78, 0xf8,
	0x04, 0x84, 0x44, 0xc4, 0x24, 0xa4, 0x64, 0xe4,
	0x14, 0x94, 0x54, 0xd4, 0x34, 0xb4, 0x74, 0xf4,
	0x0c, 0x8c, 0x4c, 0xcc, 0x2c, 0xac, 0x6c, 0xec,
	0x1c, 0x9c, 0x5c, 0xdc, 0x3c, 0xbc, 0x7c, 0xfc,
	0x02, 0x82, 0x42, 0xc2, 0x22, 0xa2, 0x62, 0xe2,
	0x12, 0x92, 0x52, 0xd2, 0x32, 0xb2, 0x72, 0xf2,
	0x0a, 0x8a, 0x4a, 0xca, 0x2a, 0xaa, 0x6a, 0xea,
	0x1a, 0x9a, 0x5a, 0xda, 0x3a, 0xba, 0x7a, 0xfa,
	0x06, 0x86, 0x46, 0xc6, 0x26, 0xa6, 0x66, 0xe6,
	0x16, 0x96, 0x56, 0xd6, 0x36, 0xb6, 0x76, 0xf6,
	0x0e, 0x8e, 0x4e, 0xce, 0x2e, 0xae, 0x6e, 0xee,
	0x1e, 0x9e, 0x5e, 0xde, 0x3e, 0xbe, 0x7e, 0xfe,
	0x01, 0x81, 0x41, 0xc1, 0x21, 0xa1, 0x61, 0xe1,
	0x11, 0x91, 0x51, 0xd1, 0x31, 0xb1, 0x71, 0xf1,
	0x09, 0x89, 0x49, 0xc9, 0x29, 0xa9, 0x69, 0xe9,
	0x19, 0x99, 0x59, 0xd9, 0x39, 0xb9, 0x79, 0xf9,
	0x05, 0x85, 0x45, 0xc5, 0x25, 0xa5, 0x65, 0xe5,
	0x15, 0x95, 0x55, 0xd5, 0x35, 0xb5, 0x75, 0xf5,
	0x0d, 0x8d, 0x4d, 0xcd, 0x2d, 0xad, 0x6d, 0xed,
	0x1d, 0x9d, 0x5d, 0xdd, 0x3d, 0xbd, 0x7d, 0xfd,
	0x03, 0x83, 0x43, 0xc3, 0x23, 0xa3, 0x63, 0xe3,
	0x13, 0x93, 0x53, 0xd3, 0x33, 0xb3, 0x73, 0xf3,
	0x0b, 0x8b, 0x4b, 0xcb, 0x2b, 0xab, 0x6b, 0xeb,
	0x1b, 0x9b, 0x5b, 0xdb, 0x3b, 0xbb, 0x7b, 0xfb,
	0x07, 0x87, 0x47, 0xc7, 0x27, 0xa7, 0x67, 0xe7,
	0x17, 0x97, 0x57, 0xd7, 0x37, 0xb7, 0x77, 0xf7,
	0x0f, 0x8f, 0x4f, 0xcf, 0x2f, 0xaf, 0x6f, 0xef,
	0x1f, 0x9f, 0x5f, 0xdf, 0x3f, 0xbf, 0x7f, 0xff,
};

#define RF_POWER_UP_100KBPS 0x02, 0x01, 0x00, 0x01, 0xC9, 0xC3, 0x80
#define RF_GPIO_PIN_CFG_100KBPS 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
#define RF_GLOBAL_XO_TUNE_2_100KBPS 0x11, 0x00, 0x02, 0x00, 0x52, 0x00
#define RF_GLOBAL_CONFIG_1_100KBPS 0x11, 0x00, 0x01, 0x03, 0x60
#define RF_INT_CTL_ENABLE_2_100KBPS 0x11, 0x01, 0x02, 0x00, 0x01, 0x20
#define RF_FRR_CTL_A_MODE_4_100KBPS 0x11, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00
#define RF_PREAMBLE_TX_LENGTH_9_100KBPS 0x11, 0x10, 0x09, 0x00, 0x04, 0x14, 0x00, 0x0F, 0x31, 0x00, 0x00, 0x00, 0x00
#define RF_SYNC_CONFIG_5_100KBPS 0x11, 0x11, 0x05, 0x00, 0x03, 0x55, 0x66, 0xAA, 0xA5
#define RF_PKT_CRC_CONFIG_7_100KBPS 0x11, 0x12, 0x07, 0x00, 0x80, 0x01, 0x08, 0xFF, 0xFF, 0x00, 0x02
#define RF_PKT_LEN_12_100KBPS 0x11, 0x12, 0x0C, 0x08, 0x00, 0x00, 0x00, 0x30, 0x3C, 0x00, 0x38, 0x04, 0x00, 0x00, 0x00, 0x00
#define RF_PKT_FIELD_2_CRC_CONFIG_12_100KBPS 0x11, 0x12, 0x0C, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
#define RF_PKT_FIELD_5_CRC_CONFIG_12_100KBPS 0x11, 0x12, 0x0C, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
#define RF_PKT_RX_FIELD_3_CRC_CONFIG_9_100KBPS 0x11, 0x12, 0x09, 0x2C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
#define RF_MODEM_MOD_TYPE_12_100KBPS 0x11, 0x20, 0x0C, 0x00, 0x03, 0x00, 0x07, 0x1E, 0x84, 0x80, 0x09, 0xC9, 0xC3, 0x80, 0x00, 0x06
#define RF_MODEM_FREQ_DEV_0_1_100KBPS 0x11, 0x20, 0x01, 0x0C, 0xD4
#define RF_MODEM_TX_RAMP_DELAY_8_100KBPS 0x11, 0x20, 0x08, 0x18, 0x01, 0x00, 0x08, 0x03, 0xC0, 0x00, 0x10, 0x20
#define RF_MODEM_BCR_OSR_1_9_100KBPS 0x11, 0x20, 0x09, 0x22, 0x00, 0x4B, 0x06, 0xD3, 0xA0, 0x06, 0xD4, 0x02, 0x00
#define RF_MODEM_AFC_GEAR_7_100KBPS 0x11, 0x20, 0x07, 0x2C, 0x00, 0x23, 0x83, 0x6A, 0x00, 0xD3, 0xA0
#define RF_MODEM_AGC_CONTROL_1_100KBPS 0x11, 0x20, 0x01, 0x35, 0xE2
#define RF_MODEM_AGC_WINDOW_SIZE_9_100KBPS 0x11, 0x20, 0x09, 0x38, 0x11, 0x10, 0x10, 0x00, 0x1A, 0x40, 0x00, 0x00, 0x28
#define RF_MODEM_OOK_CNT1_9_100KBPS 0x11, 0x20, 0x09, 0x42, 0xA4, 0x03, 0xD6, 0x03, 0x01, 0x00, 0x01, 0x80, 0xFF
#define RF_MODEM_RSSI_CONTROL_1_100KBPS 0x11, 0x20, 0x01, 0x4C, 0x00
#define RF_MODEM_RSSI_COMP_1_100KBPS 0x11, 0x20, 0x01, 0x4E, 0x40
#define RF_MODEM_CLKGEN_BAND_1_100KBPS 0x11, 0x20, 0x01, 0x51, 0x08
#define RF_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12_100KBPS 0x11, 0x21, 0x0C, 0x00, 0xFF, 0xC4, 0x30, 0x7F, 0xF5, 0xB5, 0xB8, 0xDE, 0x05, 0x17, 0x16, 0x0C
#define RF_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12_100KBPS 0x11, 0x21, 0x0C, 0x0C, 0x03, 0x00, 0x15, 0xFF, 0x00, 0x00, 0xFF, 0xC4, 0x30, 0x7F, 0xF5, 0xB5
#define RF_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12_100KBPS 0x11, 0x21, 0x0C, 0x18, 0xB8, 0xDE, 0x05, 0x17, 0x16, 0x0C, 0x03, 0x00, 0x15, 0xFF, 0x00, 0x00
#define RF_PA_MODE_4_100KBPS 0x11, 0x22, 0x04, 0x00, 0x08, 0x7F, 0x00, 0x3D
#define RF_SYNTH_PFDCP_CPFF_7_100KBPS 0x11, 0x23, 0x07, 0x00, 0x34, 0x04, 0x0B, 0x04, 0x07, 0x70, 0x03
#define RF_MATCH_VALUE_1_12_100KBPS 0x11, 0x30, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
#define RF_FREQ_CONTROL_INTE_8_100KBPS 0x11, 0x40, 0x08, 0x00, 0x38, 0x0F, 0x0A, 0x3D, 0x1B, 0x4F, 0x20, 0xFF

#define RADIO_CFG_DATA_ARRAY_100KBPS { \
        0x07, RF_POWER_UP_100KBPS, \
        0x08, RF_GPIO_PIN_CFG_100KBPS, \
        0x06, RF_GLOBAL_XO_TUNE_2_100KBPS, \
        0x05, RF_GLOBAL_CONFIG_1_100KBPS, \
        0x06, RF_INT_CTL_ENABLE_2_100KBPS, \
        0x08, RF_FRR_CTL_A_MODE_4_100KBPS, \
        0x0D, RF_PREAMBLE_TX_LENGTH_9_100KBPS, \
        0x09, RF_SYNC_CONFIG_5_100KBPS, \
        0x0B, RF_PKT_CRC_CONFIG_7_100KBPS, \
        0x10, RF_PKT_LEN_12_100KBPS, \
        0x10, RF_PKT_FIELD_2_CRC_CONFIG_12_100KBPS, \
        0x10, RF_PKT_FIELD_5_CRC_CONFIG_12_100KBPS, \
        0x0D, RF_PKT_RX_FIELD_3_CRC_CONFIG_9_100KBPS, \
        0x10, RF_MODEM_MOD_TYPE_12_100KBPS, \
        0x05, RF_MODEM_FREQ_DEV_0_1_100KBPS, \
        0x0C, RF_MODEM_TX_RAMP_DELAY_8_100KBPS, \
        0x0D, RF_MODEM_BCR_OSR_1_9_100KBPS, \
        0x0B, RF_MODEM_AFC_GEAR_7_100KBPS, \
        0x05, RF_MODEM_AGC_CONTROL_1_100KBPS, \
        0x0D, RF_MODEM_AGC_WINDOW_SIZE_9_100KBPS, \
        0x0D, RF_MODEM_OOK_CNT1_9_100KBPS, \
        0x05, RF_MODEM_RSSI_CONTROL_1_100KBPS, \
        0x05, RF_MODEM_RSSI_COMP_1_100KBPS, \
        0x05, RF_MODEM_CLKGEN_BAND_1_100KBPS, \
        0x10, RF_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12_100KBPS, \
        0x10, RF_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12_100KBPS, \
        0x10, RF_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12_100KBPS, \
        0x08, RF_PA_MODE_4_100KBPS, \
        0x0B, RF_SYNTH_PFDCP_CPFF_7_100KBPS, \
        0x10, RF_MATCH_VALUE_1_12_100KBPS, \
        0x0C, RF_FREQ_CONTROL_INTE_8_100KBPS, \
        0x00 \
 }

#define RF_POWER_UP_38KBPS 0x02, 0x01, 0x00, 0x01, 0xC9, 0xC3, 0x80
#define RF_GPIO_PIN_CFG_38KBPS 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
#define RF_GLOBAL_XO_TUNE_2_38KBPS 0x11, 0x00, 0x02, 0x00, 0x52, 0x00
#define RF_GLOBAL_CONFIG_1_38KBPS 0x11, 0x00, 0x01, 0x03, 0x60
#define RF_INT_CTL_ENABLE_2_38KBPS 0x11, 0x01, 0x02, 0x00, 0x01, 0x20
#define RF_FRR_CTL_A_MODE_4_38KBPS 0x11, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00
#define RF_PREAMBLE_TX_LENGTH_9_38KBPS 0x11, 0x10, 0x09, 0x00, 0x0A, 0x14, 0x00, 0x0C, 0x31, 0x00, 0x00, 0x00, 0x00
#define RF_SYNC_CONFIG_5_38KBPS 0x11, 0x11, 0x05, 0x00, 0x01, 0x2D, 0xD4, 0x00, 0x00
#define RF_PKT_CRC_CONFIG_7_38KBPS 0x11, 0x12, 0x07, 0x00, 0x80, 0x01, 0x08, 0xFF, 0xFF, 0x00, 0x02
#define RF_PKT_LEN_12_38KBPS 0x11, 0x12, 0x0C, 0x08, 0x00, 0x00, 0x00, 0x30, 0x3C, 0x00, 0x1F, 0x04, 0x00, 0x00, 0x00, 0x00
#define RF_PKT_FIELD_2_CRC_CONFIG_12_38KBPS 0x11, 0x12, 0x0C, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
#define RF_PKT_FIELD_5_CRC_CONFIG_12_38KBPS 0x11, 0x12, 0x0C, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
#define RF_PKT_RX_FIELD_3_CRC_CONFIG_9_38KBPS 0x11, 0x12, 0x09, 0x2C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
#define RF_MODEM_MOD_TYPE_12_38KBPS 0x11, 0x20, 0x0C, 0x00, 0x03, 0x00, 0x07, 0x0B, 0xB8, 0x00, 0x09, 0xC9, 0xC3, 0x80, 0x00, 0x01
#define RF_MODEM_FREQ_DEV_0_1_38KBPS 0x11, 0x20, 0x01, 0x0C, 0x50
#define RF_MODEM_TX_RAMP_DELAY_8_38KBPS 0x11, 0x20, 0x08, 0x18, 0x01, 0x00, 0x08, 0x03, 0xC0, 0x00, 0x20, 0x20
#define RF_MODEM_BCR_OSR_1_9_38KBPS 0x11, 0x20, 0x09, 0x22, 0x00, 0x62, 0x05, 0x3E, 0x2D, 0x07, 0xFF, 0x02, 0x00
#define RF_MODEM_AFC_GEAR_7_38KBPS 0x11, 0x20, 0x07, 0x2C, 0x00, 0x12, 0x80, 0xA8, 0x02, 0x55, 0xA0
#define RF_MODEM_AGC_CONTROL_1_38KBPS 0x11, 0x20, 0x01, 0x35, 0xE2
#define RF_MODEM_AGC_WINDOW_SIZE_9_38KBPS 0x11, 0x20, 0x09, 0x38, 0x11, 0x15, 0x15, 0x00, 0x1A, 0x20, 0x00, 0x00, 0x28
#define RF_MODEM_OOK_CNT1_9_38KBPS 0x11, 0x20, 0x09, 0x42, 0xA4, 0x03, 0xD6, 0x03, 0x00, 0x62, 0x01, 0x80, 0xFF
#define RF_MODEM_RSSI_CONTROL_1_38KBPS 0x11, 0x20, 0x01, 0x4C, 0x00
#define RF_MODEM_RSSI_COMP_1_38KBPS 0x11, 0x20, 0x01, 0x4E, 0x40
#define RF_MODEM_CLKGEN_BAND_1_38KBPS 0x11, 0x20, 0x01, 0x51, 0x08
#define RF_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12_38KBPS 0x11, 0x21, 0x0C, 0x00, 0xFF, 0xC4, 0x30, 0x7F, 0xF5, 0xB5, 0xB8, 0xDE, 0x05, 0x17, 0x16, 0x0C
#define RF_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12_38KBPS 0x11, 0x21, 0x0C, 0x0C, 0x03, 0x00, 0x15, 0xFF, 0x00, 0x00, 0xFF, 0xC4, 0x30, 0x7F, 0xF5, 0xB5
#define RF_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12_38KBPS 0x11, 0x21, 0x0C, 0x18, 0xB8, 0xDE, 0x05, 0x17, 0x16, 0x0C, 0x03, 0x00, 0x15, 0xFF, 0x00, 0x00
#define RF_PA_MODE_4_38KBPS 0x11, 0x22, 0x04, 0x00, 0x08, 0x7F, 0x00, 0x3D
#define RF_SYNTH_PFDCP_CPFF_7_38KBPS 0x11, 0x23, 0x07, 0x00, 0x2C, 0x0E, 0x0B, 0x04, 0x0C, 0x73, 0x03
#define RF_MATCH_VALUE_1_12_38KBPS 0x11, 0x30, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
#define RF_FREQ_CONTROL_INTE_8_38KBPS 0x11, 0x40, 0x08, 0x00, 0x38, 0x0F, 0xBF, 0x25, 0x0D, 0xA7, 0x20, 0xFF

#define RADIO_CFG_DATA_ARRAY_38KBPS { \
        0x07, RF_POWER_UP_38KBPS, \
        0x08, RF_GPIO_PIN_CFG_38KBPS, \
        0x06, RF_GLOBAL_XO_TUNE_2_38KBPS, \
        0x05, RF_GLOBAL_CONFIG_1_38KBPS, \
        0x06, RF_INT_CTL_ENABLE_2_38KBPS, \
        0x08, RF_FRR_CTL_A_MODE_4_38KBPS, \
        0x0D, RF_PREAMBLE_TX_LENGTH_9_38KBPS, \
        0x09, RF_SYNC_CONFIG_5_38KBPS, \
        0x0B, RF_PKT_CRC_CONFIG_7_38KBPS, \
        0x10, RF_PKT_LEN_12_38KBPS, \
        0x10, RF_PKT_FIELD_2_CRC_CONFIG_12_38KBPS, \
        0x10, RF_PKT_FIELD_5_CRC_CONFIG_12_38KBPS, \
        0x0D, RF_PKT_RX_FIELD_3_CRC_CONFIG_9_38KBPS, \
        0x10, RF_MODEM_MOD_TYPE_12_38KBPS, \
        0x05, RF_MODEM_FREQ_DEV_0_1_38KBPS, \
        0x0C, RF_MODEM_TX_RAMP_DELAY_8_38KBPS, \
        0x0D, RF_MODEM_BCR_OSR_1_9_38KBPS, \
        0x0B, RF_MODEM_AFC_GEAR_7_38KBPS, \
        0x05, RF_MODEM_AGC_CONTROL_1_38KBPS, \
        0x0D, RF_MODEM_AGC_WINDOW_SIZE_9_38KBPS, \
        0x0D, RF_MODEM_OOK_CNT1_9_38KBPS, \
        0x05, RF_MODEM_RSSI_CONTROL_1_38KBPS, \
        0x05, RF_MODEM_RSSI_COMP_1_38KBPS, \
        0x05, RF_MODEM_CLKGEN_BAND_1_38KBPS, \
        0x10, RF_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12_38KBPS, \
        0x10, RF_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12_38KBPS, \
        0x10, RF_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12_38KBPS, \
        0x08, RF_PA_MODE_4_38KBPS, \
        0x0B, RF_SYNTH_PFDCP_CPFF_7_38KBPS, \
        0x10, RF_MATCH_VALUE_1_12_38KBPS, \
        0x0C, RF_FREQ_CONTROL_INTE_8_38KBPS, \
        0x00 \
 }

PROGMEM const uint8_t si4463_modem_cfg_100kbps[] = RADIO_CFG_DATA_ARRAY_100KBPS;
PROGMEM const uint8_t si4463_modem_cfg_38kbps [] = RADIO_CFG_DATA_ARRAY_38KBPS;

extern Slots_descr_t RF_Time_Slots;

static bool si4463_CommandRead(uint8_t cmd, uint8_t* read_buf, uint8_t read_len)
{
  bool done = false;

#if defined(USE_BASICMAC)
  hal_spi_select(1);
#else
  hal_pin_nss(0);
#endif

  hal_spi(cmd);

#if defined(USE_BASICMAC)
  hal_spi_select(0);
#else
  hal_pin_nss(1);
#endif

  uint16_t count; // Number of times we have tried to get CTS
  for (count = 0; !done && count < RH_RF24_CTS_RETRIES; count++)
  {
#if defined(USE_BASICMAC)
    hal_spi_select(1);
#else
    hal_pin_nss(0);
#endif

    hal_spi(RH_RF24_CMD_READ_BUF);
    if (hal_spi(0x00) == RH_RF24_REPLY_CTS) {
        // Now read any expected reply data
        if (read_buf && read_len) {
          while (read_len--)
              *read_buf++ = hal_spi(0x00);
        }
        done = true;
    }

#if defined(USE_BASICMAC)
    hal_spi_select(0);
#else
    hal_pin_nss(1);
#endif
  }

  return done;
}

static bool si4463_probe()
{
  uint8_t buf[8];
  uint16_t deviceType;

  SoC->SPI_begin();

  lmic_hal_init (nullptr);

  hal_pin_rst(1); // drive SDN pin high
  delay(1);

  hal_pin_rst(0); // drive SDN pin low
  delay(100);

  bool status = si4463_CommandRead(RH_RF24_CMD_PART_INFO, buf, sizeof(buf));

  pinMode(lmic_pins.nss, INPUT);
  RadioSPI.end();

  hal_pin_rst(2); // configure SDN pin floating!

  if (!status) {
    return false; // SPI error ? Not connected ?
  }

  deviceType = (buf[1] << 8) | buf[2];

#if 0
  Serial.print("si44xx device = "); Serial.println(deviceType, HEX);
#endif

  // Check PART to be either 0x4460, 0x4461, 0x4463, 0x4464
  if (deviceType != 0x4460 && deviceType != 0x4461 &&
      deviceType != 0x4463 && deviceType != 0x4464) {
    return false; // Unknown radio type, or not connected
  } else {
    return true;
  }
}

static void si4463_channel(int8_t channel)
{
  if (channel != -1 && channel != si4463_channel_prev) {
    uint32_t frequency = RF_FreqPlan.getChanFrequency((uint8_t) channel);
    int8_t fc = settings->freq_corr;

    if (settings->rf_protocol == RF_PROTOCOL_LEGACY) {
      nRF905_band_t nrf_band;
      uint32_t nrf_freq_resolution;

      nrf_band = (frequency >= 844800000UL ? NRF905_BAND_868 : NRF905_BAND_433);
      nrf_freq_resolution = (nrf_band == NRF905_BAND_433 ? 100000UL : 200000UL);
      frequency -= (frequency % nrf_freq_resolution);
    }

    if (fc > 30) {
      fc = 30;
    } else if (fc < -30) {
      fc = -30;
    };

    bool state = radio_silabs->setFrequency((frequency + (fc * 1000)) / 1000000.0);

#if 0
    if (state == false) {
      Serial.println(F("[Si4463] Selected frequency is invalid for this module!"));
      while (true) { delay(10); }
    }
#endif

    si4463_channel_prev = channel;
    /* restart Rx upon a channel switch */
    si4463_receive_active = false;
  }
}

static void si4463_setup()
{
  SoC->SPI_begin();

  uint8_t irq = lmic_pins.busy == LMIC_UNUSED_PIN ?
                RH_INVALID_PIN : lmic_pins.busy;

  uint8_t sdn = lmic_pins.rst  == LMIC_UNUSED_PIN ?
                RH_INVALID_PIN : lmic_pins.rst;

  radio_silabs = new RH_RF24(lmic_pins.nss, irq, sdn /*, RadioSPI */);

  switch (settings->rf_protocol)
  {
  case RF_PROTOCOL_OGNTP:
    rh_protocol     = &ogntp_proto_desc;
    protocol_encode = &ogntp_encode;
    protocol_decode = &ogntp_decode;
    break;
  case RF_PROTOCOL_P3I:
    rh_protocol     = &p3i_proto_desc;
    protocol_encode = &p3i_encode;
    protocol_decode = &p3i_decode;
    break;
#if defined(ENABLE_ADSL)
  case RF_PROTOCOL_ADSL_860:
    rh_protocol     = &adsl_proto_desc;
    protocol_encode = &adsl_encode;
    protocol_decode = &adsl_decode;
    break;
#endif /* ENABLE_ADSL */
  case RF_PROTOCOL_LEGACY:
  default:
    rh_protocol     = &legacy_proto_desc;
    protocol_encode = &legacy_encode;
    protocol_decode = &legacy_decode;
    /*
     * Enforce legacy protocol setting for Si4463
     * if other value (UAT) left in EEPROM from other (UATM) radio
     */
    settings->rf_protocol = RF_PROTOCOL_LEGACY;
    break;
  }

  RF_FreqPlan.setPlan(settings->band, settings->rf_protocol);

#if 0
  Serial.print(F("[Si4463] Initializing ... "));
#endif

  bool success = radio_silabs->init();

#if 0
  if (success == true) {
    Serial.println(F("success!"));
  } else {
    Serial.println(F("failed."));
    while (true) { delay(10); }
  }
#endif

  const uint8_t* commands;
  uint8_t next_cmd_len;

  switch (rh_protocol->bitrate)
  {
  case RF_BITRATE_38400:
    commands = si4463_modem_cfg_38kbps;
    break;
  case RF_BITRATE_100KBPS:
  default:
    commands = si4463_modem_cfg_100kbps;
    break;
  }

  while (memcpy_P(&next_cmd_len, commands, 1), next_cmd_len > 0)
  {
    uint8_t buf[20]; // As least big as the biggest permitted command/property list of 15
    memcpy_P(buf, commands+1, next_cmd_len);
    radio_silabs->command(buf[0], buf+1, next_cmd_len - 1);
    commands += (next_cmd_len + 1);
  }

  uint8_t int_ctl[] = {RH_RF24_MODEM_INT_STATUS_EN | RH_RF24_PH_INT_STATUS_EN, 0xff, 0xff, 0x00 };
  radio_silabs->set_properties(RH_RF24_PROPERTY_INT_CTL_ENABLE, int_ctl, sizeof(int_ctl));

  uint8_t pkt_config1[] = { 0x00 };
  radio_silabs->set_properties(RH_RF24_PROPERTY_PKT_CONFIG1, pkt_config1, sizeof(pkt_config1));

  uint8_t pkt_fieldn[] = { 0x00, 0x00, 0x00, 0x00 };
  radio_silabs->set_properties(RH_RF24_PROPERTY_PKT_FIELD_1_LENGTH_12_8, pkt_fieldn, sizeof(pkt_fieldn));
  radio_silabs->set_properties(RH_RF24_PROPERTY_PKT_FIELD_2_LENGTH_12_8, pkt_fieldn, sizeof(pkt_fieldn));
  radio_silabs->set_properties(RH_RF24_PROPERTY_PKT_FIELD_3_LENGTH_12_8, pkt_fieldn, sizeof(pkt_fieldn));
  radio_silabs->set_properties(RH_RF24_PROPERTY_PKT_FIELD_4_LENGTH_12_8, pkt_fieldn, sizeof(pkt_fieldn));
  radio_silabs->set_properties(RH_RF24_PROPERTY_PKT_FIELD_5_LENGTH_12_8, pkt_fieldn, sizeof(pkt_fieldn));

  uint8_t pre_type = (rh_protocol->preamble_type == RF_PREAMBLE_TYPE_AA) ?
                     RH_RF24_PREAMBLE_FIRST_1 | RH_RF24_PREAMBLE_STANDARD_1010 :
                     RH_RF24_PREAMBLE_FIRST_0 | RH_RF24_PREAMBLE_STANDARD_0101;

  uint8_t pre_cfg[] = { (uint8_t) rh_protocol->preamble_size, 0x14, 0x00, 0x00,
		                pre_type | RH_RF24_PREAMBLE_LENGTH_BYTES }; // in bytes
  radio_silabs->set_properties(RH_RF24_PROPERTY_PREAMBLE_TX_LENGTH,
                               pre_cfg, sizeof(pre_cfg));

  size_t pkt_size = rh_protocol->payload_offset + rh_protocol->payload_size +
                    rh_protocol->crc_size;

  switch (rh_protocol->whitening)
  {
  case RF_WHITENING_MANCHESTER:
    pkt_size += pkt_size;
    break;
  case RF_WHITENING_PN9:
  case RF_WHITENING_NONE:
  case RF_WHITENING_NICERF:
  default:
    break;
  }

  /* Work around premature P3I syncword detection */
  if (rh_protocol->syncword_size == 2) {
    uint8_t preamble = rh_protocol->preamble_type == RF_PREAMBLE_TYPE_AA ?
                       0xAA : 0x55;
    uint8_t sword[4] = { byte_rev_table[preamble],
                         byte_rev_table[preamble],
                         byte_rev_table[rh_protocol->syncword[0]],
                         byte_rev_table[rh_protocol->syncword[1]]
                       };
    radio_silabs->setSyncWords(sword, 4);
  } else {
    uint8_t sword[4];
    uint8_t sw_size = rh_protocol->syncword_size > 4 ? 4 :
                      rh_protocol->syncword_size;
    for (int i=0; i<sw_size; i++) {
      sword[i] = byte_rev_table[rh_protocol->syncword[i]];
    }

    radio_silabs->setSyncWords(sword, sw_size);
    if (rh_protocol->syncword_size > 4) {
      pkt_size += rh_protocol->syncword_size - 4;
    }
  }

  radio_silabs->setPromiscuous(true);

  RH_rxPacket.len = pkt_size;

  uint8_t l[] = { (uint8_t)(pkt_size)};
  radio_silabs->set_properties(RH_RF24_PROPERTY_PKT_FIELD_1_LENGTH_7_0, l, sizeof(l));

  radio_silabs->setCRCPolynomial(RH_RF24::CRC_NONE);

  // Default freq comes from the radio config file
  // About 2.4dBm on RFM24:
  radio_silabs->setTxPower(0x10);

  /* Load regional max. EIRP at first */
  float   txpow = RF_FreqPlan.MaxTxPower;;
  uint8_t power = 0x0e; /* < 4.8 dBm */

  switch(settings->txpower)
  {
  case RF_TX_POWER_FULL:
    if (txpow > 20)
      txpow = 20;

#if 1
    /*
     * Enforce Tx power limit until confirmation
     * that Si4463 is doing well
     * when antenna is not connected
     */
    if (txpow > 17)
      txpow = 17;
#endif

    if (txpow >= 20) {
      power = 0x7f; /* 19.2 dBm */
    } else if (txpow >= 18) {
      power = 0x4f; /* 18.0 dBm */
    } else if (txpow >= 14) {
      power = 0x2f; /* 14.2 dBm */
    } else if (txpow >= 11) {
      power = 0x1f; /* 11.0 dBm */
    } else if (txpow >=  5) {
      power = 0x0f; /*  4.8 dBm */
    }
    break;
  case RF_TX_POWER_OFF:
  case RF_TX_POWER_LOW:
  default:
     /* < 4.8 dBm */
    break;
  }

  radio_silabs->setTxPower(power);
}

static bool si4463_receive()
{
  bool success = false;
  bool state;

  if (settings->power_save & POWER_SAVE_NORECEIVE) {
    return success;
  }

  if (!si4463_receive_active) {

    radio_silabs->available();

    if (radio_silabs->mode() == RHGenericDriver::RHModeRx) {
      si4463_receive_active = true;
    }
  }

  if (si4463_receive_active == true) {

    uint8_t len = RH_rxPacket.len;

    if (radio_silabs->recv(RH_rxPacket.payload, &len)) {

      if (len > sizeof(RH_rxPacket.payload)) {
        len = sizeof(RH_rxPacket.payload);
      }

      si4463_receive_active = false;

      if (!memeqzero(RH_rxPacket.payload, len)) {

        uint8_t i;
#if 0
        Serial.print("Payload ");
        for (i=0; i < len; i++) {
            Serial.print(RH_rxPacket.payload[i], HEX);
            //Serial.print(" ");
        }
        Serial.println();
#endif
        if (rh_protocol->syncword_size > 4) {
          for (i=4; i < rh_protocol->syncword_size; i++) {
            if (RH_rxPacket.payload[i-4] != rh_protocol->syncword[i]) {
#if 0
              Serial.print("syncword mismatch ");
              Serial.print("i="); Serial.print(i);
              Serial.print(" p="); Serial.print(RH_rxPacket.payload[i-4], HEX);
              Serial.print(" s="); Serial.print(rh_protocol->syncword[i], HEX);
              Serial.println();
#endif
#if 1
//              if (i != 4) {
                memset(RH_rxPacket.payload, 0, sizeof(RH_rxPacket.payload));
                len = 0;

                return success;
//              }
#endif
            }
          }

          memcpy(RH_rxPacket.payload,
                 RH_rxPacket.payload + rh_protocol->syncword_size - 4,
                 len - (rh_protocol->syncword_size - 4));
        }

        size_t size = 0;
        uint8_t offset;

        u1_t crc8, pkt_crc8;
        u2_t crc16, pkt_crc16;

        RadioHead_DataPacket *RH_rxPacket_ptr = &RH_rxPacket;

        switch (rh_protocol->crc_type)
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

        switch (rh_protocol->type)
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

        switch (rh_protocol->type)
        {
        case RF_PROTOCOL_P3I:
          offset = rh_protocol->payload_offset;
          for (i = 0; i < rh_protocol->payload_size; i++)
          {
            update_crc8(&crc8, (u1_t)(RH_rxPacket_ptr->payload[i + offset]));
            if (i < sizeof(RxBuffer)) {
              RxBuffer[i] = RH_rxPacket_ptr->payload[i + offset] ^
                            pgm_read_byte(&whitening_pattern[i]);
            }
          }

          pkt_crc8 = RH_rxPacket_ptr->payload[i + offset];

          if (crc8 == pkt_crc8) {
            success = true;
          }
          break;
        case RF_PROTOCOL_OGNTP:
        case RF_PROTOCOL_ADSL_860:
        case RF_PROTOCOL_LEGACY:
        default:
          offset = 0;
          size   = rh_protocol->payload_offset +
                   rh_protocol->payload_size +
                   rh_protocol->payload_size +
                   rh_protocol->crc_size +
                   rh_protocol->crc_size;
          if (RH_rxPacket_ptr->len >= (size + offset)) {
            uint8_t val1, val2;
            for (i = 0; i < size; i++) {
              val1 = pgm_read_byte(&ManchesterDecode[RH_rxPacket_ptr->payload[i + offset]]);
              i++;
              val2 = pgm_read_byte(&ManchesterDecode[RH_rxPacket_ptr->payload[i + offset]]);
              if ((i>>1) < sizeof(RxBuffer)) {
                RxBuffer[i>>1] = ((val1 & 0x0F) << 4) | (val2 & 0x0F);

                if (i < size - (rh_protocol->crc_size + rh_protocol->crc_size)) {
                  switch (rh_protocol->crc_type)
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

            switch (rh_protocol->crc_type)
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
              offset = rh_protocol->payload_offset + rh_protocol->payload_size;
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
        }

        if (success) {
          RF_last_rssi = radio_silabs->lastRssi();
          rx_packets_counter++;
        }
      }

      memset(RH_rxPacket.payload, 0, sizeof(RH_rxPacket.payload));
      len = 0;
    }

  }

  return success;
}

static bool si4463_transmit()
{
  u1_t crc8;
  u2_t crc16;
  u1_t i;

  bool success = false;

  if (RF_tx_size <= 0) {
    return success;
  }

  si4463_receive_active = false;

  size_t PayloadLen = 0;

#if 0
  /* Work around 0xAA preamble in use by OGNTP */
  if (rh_protocol->preamble_type == RF_PREAMBLE_TYPE_AA &&
      rh_protocol->preamble_size == 1) {
    if (rh_protocol->syncword_size > 3) {
      for (i=3; i < rh_protocol->syncword_size; i++) {
        RH_txPacket.payload[PayloadLen++] = rh_protocol->syncword[i];
      }
    }
  } else
#endif
  {
    if (rh_protocol->syncword_size > 4) {
      for (i=4; i < rh_protocol->syncword_size; i++) {
        RH_txPacket.payload[PayloadLen++] = rh_protocol->syncword[i];
      }
    }
  }

  switch (rh_protocol->crc_type)
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

  switch (rh_protocol->type)
  {
  case RF_PROTOCOL_LEGACY:
    /* take in account NRF905/FLARM "address" bytes */
    crc16 = update_crc_ccitt(crc16, 0x31);
    crc16 = update_crc_ccitt(crc16, 0xFA);
    crc16 = update_crc_ccitt(crc16, 0xB6);
    break;
  case RF_PROTOCOL_P3I:
    /* insert Net ID */
    RH_txPacket.payload[PayloadLen++] = (u1_t) ((rh_protocol->net_id >> 24) & 0x000000FF);
    RH_txPacket.payload[PayloadLen++] = (u1_t) ((rh_protocol->net_id >> 16) & 0x000000FF);
    RH_txPacket.payload[PayloadLen++] = (u1_t) ((rh_protocol->net_id >>  8) & 0x000000FF);
    RH_txPacket.payload[PayloadLen++] = (u1_t) ((rh_protocol->net_id >>  0) & 0x000000FF);
    /* insert byte with payload size */
    RH_txPacket.payload[PayloadLen++] = rh_protocol->payload_size;

    /* insert byte with CRC-8 seed value when necessary */
    if (rh_protocol->crc_type == RF_CHECKSUM_TYPE_CRC8_107) {
      RH_txPacket.payload[PayloadLen++] = crc8;
    }

    break;
  case RF_PROTOCOL_OGNTP:
  case RF_PROTOCOL_ADSL_860:
  default:
    break;
  }

  for (i=0; i < RF_tx_size; i++) {

    switch (rh_protocol->whitening)
    {
    case RF_WHITENING_NICERF:
      RH_txPacket.payload[PayloadLen] = TxBuffer[i] ^ pgm_read_byte(&whitening_pattern[i]);
      break;
    case RF_WHITENING_MANCHESTER:
      RH_txPacket.payload[PayloadLen] = pgm_read_byte(&ManchesterEncode[(TxBuffer[i] >> 4) & 0x0F]);
      PayloadLen++;
      RH_txPacket.payload[PayloadLen] = pgm_read_byte(&ManchesterEncode[(TxBuffer[i]     ) & 0x0F]);
      break;
    case RF_WHITENING_NONE:
    default:
      RH_txPacket.payload[PayloadLen] = TxBuffer[i];
      break;
    }

    switch (rh_protocol->crc_type)
    {
    case RF_CHECKSUM_TYPE_GALLAGER:
    case RF_CHECKSUM_TYPE_CRC_MODES:
    case RF_CHECKSUM_TYPE_NONE:
      break;
    case RF_CHECKSUM_TYPE_CRC8_107:
      update_crc8(&crc8, (u1_t)(RH_txPacket.payload[PayloadLen]));
      break;
    case RF_CHECKSUM_TYPE_CCITT_FFFF:
    case RF_CHECKSUM_TYPE_CCITT_0000:
    default:
      if (rh_protocol->whitening == RF_WHITENING_MANCHESTER) {
        crc16 = update_crc_ccitt(crc16, (u1_t)(TxBuffer[i]));
      } else {
        crc16 = update_crc_ccitt(crc16, (u1_t)(RH_txPacket.payload[PayloadLen]));
      }
      break;
    }

    PayloadLen++;
  }

  switch (rh_protocol->crc_type)
  {
  case RF_CHECKSUM_TYPE_GALLAGER:
  case RF_CHECKSUM_TYPE_CRC_MODES:
  case RF_CHECKSUM_TYPE_NONE:
    break;
  case RF_CHECKSUM_TYPE_CRC8_107:
    RH_txPacket.payload[PayloadLen++] = crc8;
    break;
  case RF_CHECKSUM_TYPE_CCITT_FFFF:
  case RF_CHECKSUM_TYPE_CCITT_0000:
  default:
    if (rh_protocol->whitening == RF_WHITENING_MANCHESTER) {
      RH_txPacket.payload[PayloadLen++] = pgm_read_byte(&ManchesterEncode[(((crc16 >>  8) & 0xFF) >> 4) & 0x0F]);
      RH_txPacket.payload[PayloadLen++] = pgm_read_byte(&ManchesterEncode[(((crc16 >>  8) & 0xFF)     ) & 0x0F]);
      RH_txPacket.payload[PayloadLen++] = pgm_read_byte(&ManchesterEncode[(((crc16      ) & 0xFF) >> 4) & 0x0F]);
      RH_txPacket.payload[PayloadLen++] = pgm_read_byte(&ManchesterEncode[(((crc16      ) & 0xFF)     ) & 0x0F]);
      PayloadLen++;
    } else {
      RH_txPacket.payload[PayloadLen++] = (crc16 >>  8) & 0xFF;
      RH_txPacket.payload[PayloadLen++] = (crc16      ) & 0xFF;
    }
    break;
  }

  RH_txPacket.len = PayloadLen;

  success = radio_silabs->send((uint8_t *) &RH_txPacket.payload, (size_t) RH_txPacket.len);

  if (success) {
    uint16_t timeout = RF_Time_Slots.air_time + RF_Time_Slots.air_time;

    if (radio_silabs->waitPacketSent(timeout) == false) {
      radio_silabs->setModeIdle();

//      success = false;
    }

    memset(RH_txPacket.payload, 0, sizeof(RH_txPacket.payload));
  }

#if 0
  if (success) {
    // the packet was successfully transmitted
    Serial.println(F("success!"));

  } else {
    // some other error occurred
    Serial.println(F("failed."));
  }
#endif

  return success;
}

static void si4463_shutdown()
{
  int state = radio_silabs->sleep();
}
#endif /* EXCLUDE_SI446X */

#endif /* USE_RADIOHEAD */
