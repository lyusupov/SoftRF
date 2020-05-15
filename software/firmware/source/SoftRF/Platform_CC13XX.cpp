/*
 * Platform_CC13XX.cpp
 * Copyright (C) 2019-2020 Linar Yusupov
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

#if defined(ENERGIA_ARCH_CC13XX) || defined(ENERGIA_ARCH_CC13X2)

#include <Wire.h>

#if defined(ENERGIA_ARCH_CC13XX)
#include <ti/devices/cc13x0/driverlib/sys_ctrl.h>
#endif /* ENERGIA_ARCH_CC13XX */
#if defined(ENERGIA_ARCH_CC13X2)
#include <ti/devices/cc13x2_cc26x2/driverlib/sys_ctrl.h>

#include <SPIFlash.h>
#include <ADXL362.h>
#endif /* ENERGIA_ARCH_CC13X2 */

#include "SoCHelper.h"
#include "RFHelper.h"
#include "LEDHelper.h"
#include "SoundHelper.h"
#include "BaroHelper.h"

#include "EasyLink.h"

#if !defined(EXCLUDE_SX12XX)
// RFM95W pin mapping
lmic_pinmap lmic_pins = {
    .nss = SOC_GPIO_PIN_SS,
    .txe = LMIC_UNUSED_PIN,
    .rxe = LMIC_UNUSED_PIN,
    .rst = SOC_GPIO_PIN_RST,
    .dio = {LMIC_UNUSED_PIN, LMIC_UNUSED_PIN, LMIC_UNUSED_PIN},
    .busy = LMIC_UNUSED_PIN,
    .tcxo = LMIC_UNUSED_PIN,
};
#endif

static uint8_t ieeeAddr[8];

#if !defined(EXCLUDE_LED_RING)
WS2812 strip(PIX_NUM);
uint8_t LEDs[PIX_NUM][3];
#endif /* EXCLUDE_LED_RING */

#if defined(USE_OLED)
#include <U8x8lib.h>

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8_i2c(U8X8_PIN_NONE);

static U8X8_SSD1306_128X64_NONAME_HW_I2C *u8x8 = NULL;

static bool OLED_display_probe_status = false;
static bool OLED_display_frontpage = false;
static uint32_t prev_tx_packets_counter = 0;
static uint32_t prev_rx_packets_counter = 0;
extern uint32_t tx_packets_counter, rx_packets_counter;

const char *OLED_Protocol_ID[] = {
  [RF_PROTOCOL_LEGACY]    = "L",
  [RF_PROTOCOL_OGNTP]     = "O",
  [RF_PROTOCOL_P3I]       = "P",
  [RF_PROTOCOL_ADSB_1090] = "A",
  [RF_PROTOCOL_ADSB_UAT]  = "U",
  [RF_PROTOCOL_FANET]     = "F"
};
#endif /* USE_OLED */

char UDPpacketBuffer[4]; // Dummy definition to satisfy build sequence

static struct rst_info reset_info = {
  .reason = REASON_DEFAULT_RST,
};

static uint32_t bootCount = 0;
static int cc13xx_board = SOFTRF_UAT_MODULE_19; /* default */

#if defined(ENERGIA_ARCH_CC13X2)

static void Uart2_ReadCallback(UART_Handle uart, void *buf, size_t count)
{
    Serial2.readCallback(uart, buf, count);
}

static void Uart2_WriteCallback(UART_Handle uart, void *buf, size_t count)
{
    Serial2.writeCallback(uart, buf, count);
}

HardwareSerial Serial2(1, Uart2_ReadCallback, Uart2_WriteCallback, true);

SPIFlash flash(SOC_GPIO_PIN_MX25_SS); // MACRONIX_MX25R8035F

ADXL362 adxl;


size_t strnlen (const char *string, size_t length)
{
   char *ret = (char *) memchr ((const void *) string, 0, length);
   return ret ? ret - string : length;
}

char* ltoa( long value, char *string, int radix )
{
  char tmp[33];
  char *tp = tmp;
  long i;
  unsigned long v;
  int sign;
  char *sp;

  if ( string == NULL )
  {
    return 0 ;
  }

  if (radix > 36 || radix <= 1)
  {
    return 0 ;
  }

  sign = (radix == 10 && value < 0);
  if (sign)
  {
    v = -value;
  }
  else
  {
    v = (unsigned long)value;
  }

  while (v || tp == tmp)
  {
    i = v % radix;
    v = v / radix;
    if (i < 10)
      *tp++ = i+'0';
    else
      *tp++ = i + 'a' - 10;
  }

  sp = string;

  if (sign)
    *sp++ = '-';
  while (tp > tmp)
    *sp++ = *--tp;
  *sp = 0;

  return string;
}

char* itoa( int value, char *string, int radix )
{
  return ltoa( value, string, radix ) ;
}

#endif /* ENERGIA_ARCH_CC13X2 */

static void CC13XX_setup()
{
  uint32_t  reset_source = SysCtrlResetSourceGet();

  EasyLink_getIeeeAddr(ieeeAddr);

#if defined(ENERGIA_ARCH_CC13X2)

  bool has_spiflash = false;
  uint16_t flash_id = 0;
  int16_t XValue = 0, YValue = 0, ZValue = 0, Temperature = 0;

  has_spiflash = flash.initialize();

  if (has_spiflash) {
    flash_id = flash.readDeviceId();
    flash.sleep();
  }
  flash.end();

  adxl.begin(SOC_GPIO_PIN_ADXL_SS);
  adxl.beginMeasure();

  adxl.readXYZTData(XValue, YValue, ZValue, Temperature);
  /* no .end() method for adxl */

  if (has_spiflash && flash_id == MACRONIX_MX25R8035F) {

    hw_info.model = SOFTRF_MODEL_UNI;

    if (XValue == 0 && YValue == 0 && ZValue == 0 && Temperature == 0) {
      cc13xx_board = TI_CC1352R1_LAUNCHXL;
    } else {
      cc13xx_board = TI_LPSTK_CC1352R;
    }
  } else {
    cc13xx_board  = SOFTRF_UAT_MODULE_20;
  }
#endif /* ENERGIA_ARCH_CC13X2 */

  hw_info.revision = cc13xx_board;
}

static void CC13XX_loop()
{

}

static void CC13XX_fini()
{
#if defined(ENERGIA_ARCH_CC13XX)
    SysCtrlSystemReset();
#elif defined(ENERGIA_ARCH_CC13X2)
    /* Disable interrupts */
    IntMasterDisable();

    SysCtrlStandby(false,
                   VIMS_ON_CPU_ON_MODE,
                   SYSCTRL_PREFERRED_RECHARGE_MODE);

    /* Should never return from SysCtrlStandby */
#endif /* ENERGIA_ARCH_CC13XX or ENERGIA_ARCH_CC13X2 */
}

static void CC13XX_reset()
{
  SysCtrlSystemReset();
}

static uint32_t CC13XX_getChipId()
{
  uint32_t id = (uint32_t) ieeeAddr[7]        | ((uint32_t) ieeeAddr[6] << 8) | \
               ((uint32_t) ieeeAddr[5] << 16) | ((uint32_t) ieeeAddr[4] << 24);

  /* remap address to avoid overlapping with congested FLARM range */
  if (((id & 0x00FFFFFF) >= 0xDD0000) && ((id & 0x00FFFFFF) <= 0xDFFFFF)) {
    id += 0x100000;
  }

  return id;
}

static void* CC13XX_getResetInfoPtr()
{
  return 0;
}

static String CC13XX_getResetInfo()
{
  switch (reset_info.reason)
  {
    default                     : return F("No reset information available");
  }
}

static String CC13XX_getResetReason()
{
  return F("DEFAULT");
}

static uint32_t CC13XX_getFreeHeap()
{
  return 0; /* TBD */
}

static long CC13XX_random(long howsmall, long howBig)
{
  return random(howsmall, howBig);
}

static void CC13XX_Sound_test(int var)
{

}

static void CC13XX_WiFi_setOutputPower(int dB)
{
  /* NONE */
}

static void CC13XX_WiFi_transmit_UDP(int port, byte *buf, size_t size)
{
  /* NONE */
}

static bool CC13XX_EEPROM_begin(size_t size)
{
#if !defined(EXCLUDE_EEPROM)
  EEPROM.begin(size);
#endif
  return true;
}

static void CC13XX_SPI_begin()
{
  SPI.begin();
}

static void CC13XX_swSer_begin(unsigned long baud)
{
  swSer.begin(baud);
}

static void CC13XX_swSer_enableRx(boolean arg)
{

}

static byte CC13XX_Display_setup()
{
  byte rval = DISPLAY_NONE;

#if defined(USE_OLED)
/*
 * BUG:
 * return value of Wire.endTransmission() is always '4' with Arduino Core for CCC13X2
 */
#if 0
  /* SSD1306 I2C OLED probing */
  Wire.begin();
  Wire.beginTransmission(SSD1306_OLED_I2C_ADDR);
  if (Wire.endTransmission() == 0)
#endif
  {
    u8x8 = &u8x8_i2c;
    rval = DISPLAY_OLED_TTGO;
  }

  if (u8x8) {
    u8x8->begin();
    u8x8->setFont(u8x8_font_chroma48medium8_r);
    u8x8->clear();

    u8x8->draw2x2String(2, 3, "SoftRF");
  }
#endif /* USE_OLED */

  return rval;
}

static void CC13XX_Display_loop()
{
#if defined(USE_OLED)
  char buf[16];
  uint32_t disp_value;

  if (u8x8) {
    if (!OLED_display_probe_status) {
      u8x8->clear();

      u8x8->draw2x2String(0, 0, "RADIO");
      u8x8->draw2x2String(14, 0, hw_info.rf   != RF_IC_NONE       ? "+" : "-");
      u8x8->draw2x2String(0, 2, "GNSS");
      u8x8->draw2x2String(14, 2, hw_info.gnss != GNSS_MODULE_NONE ? "+" : "-");
      u8x8->draw2x2String(0, 4, "OLED");
      u8x8->draw2x2String(14, 4, hw_info.display != DISPLAY_NONE  ? "+" : "-");
      u8x8->draw2x2String(0, 6, "BARO");
      u8x8->draw2x2String(14, 6, hw_info.baro != BARO_MODULE_NONE ? "+" : "-");

      delay(3000);

      OLED_display_probe_status = true;
    } else if (!OLED_display_frontpage) {

      u8x8->clear();

      u8x8->drawString(1, 1, "ID");

      itoa(ThisAircraft.addr & 0xFFFFFF, buf, 16);
      u8x8->draw2x2String(0, 2, buf);

      u8x8->drawString(8, 1, "PROTOCOL");

      u8x8->draw2x2String(14, 2, OLED_Protocol_ID[ThisAircraft.protocol]);

      u8x8->drawString(1, 5, "RX");

      itoa(rx_packets_counter % 1000, buf, 10);
      u8x8->draw2x2String(0, 6, buf);

      u8x8->drawString(9, 5, "TX");

      if (settings->mode != SOFTRF_MODE_BRIDGE ||
          (settings->mode == SOFTRF_MODE_BRIDGE &&
           settings->txpower == RF_TX_POWER_OFF)) {
        strcpy(buf, "OFF");
      } else {
        itoa(tx_packets_counter % 1000, buf, 10);
      }
      u8x8->draw2x2String(8, 6, buf);

      OLED_display_frontpage = true;
    } else {
      if (rx_packets_counter > prev_rx_packets_counter) {
        disp_value = rx_packets_counter % 1000;
        itoa(disp_value, buf, 10);

        if (disp_value < 10) {
          strcat_P(buf,PSTR("  "));
        } else {
          if (disp_value < 100) {
            strcat_P(buf,PSTR(" "));
          };
        }

        u8x8->draw2x2String(0, 6, buf);
        prev_rx_packets_counter = rx_packets_counter;
      }
      if (tx_packets_counter > prev_tx_packets_counter) {
        disp_value = tx_packets_counter % 1000;
        itoa(disp_value, buf, 10);

        if (disp_value < 10) {
          strcat_P(buf,PSTR("  "));
        } else {
          if (disp_value < 100) {
            strcat_P(buf,PSTR(" "));
          };
        }

        u8x8->draw2x2String(8, 6, buf);
        prev_tx_packets_counter = tx_packets_counter;
      }
    }
  }
#endif /* USE_OLED */
}

static void CC13XX_Display_fini(const char *msg)
{
#if defined(USE_OLED)
  if (u8x8) {
    u8x8->setFont(u8x8_font_chroma48medium8_r);
    u8x8->clear();
    u8x8->draw2x2String(1, 3, msg);
  }
#endif /* USE_OLED */
}

static void CC13XX_Battery_setup()
{

}

static float CC13XX_Battery_voltage()
{
  /* TBD */
  return 0 ;
}

void CC13XX_GNSS_PPS_Interrupt_handler() {
  PPS_TimeMarker = millis();
}

static unsigned long CC13XX_get_PPS_TimeMarker() {
  return PPS_TimeMarker;
}

static bool CC13XX_Baro_setup() {
  return true;
}

static void CC13XX_UATSerial_begin(unsigned long baud)
{
  UATSerial.begin(baud);
}

static void CC13XX_UATModule_restart()
{
  /* Nothing to do */
}

static void CC13XX_WDT_setup()
{
  /* TBD */
}

static void CC13XX_WDT_fini()
{
  /* TBD */
}

const SoC_ops_t CC13XX_ops = {
  SOC_CC13XX,
  "CC13XX",
  CC13XX_setup,
  CC13XX_loop,
  CC13XX_fini,
  CC13XX_reset,
  CC13XX_getChipId,
  CC13XX_getResetInfoPtr,
  CC13XX_getResetInfo,
  CC13XX_getResetReason,
  CC13XX_getFreeHeap,
  CC13XX_random,
  CC13XX_Sound_test,
  NULL,
  CC13XX_WiFi_setOutputPower,
  CC13XX_WiFi_transmit_UDP,
  NULL,
  NULL,
  NULL,
  CC13XX_EEPROM_begin,
  CC13XX_SPI_begin,
  CC13XX_swSer_begin,
  CC13XX_swSer_enableRx,
  NULL,
  CC13XX_Display_setup,
  CC13XX_Display_loop,
  CC13XX_Display_fini,
  CC13XX_Battery_setup,
  CC13XX_Battery_voltage,
  CC13XX_GNSS_PPS_Interrupt_handler,
  CC13XX_get_PPS_TimeMarker,
  CC13XX_Baro_setup,
  CC13XX_UATSerial_begin,
  CC13XX_UATModule_restart,
  CC13XX_WDT_setup,
  CC13XX_WDT_fini
};

#endif /* ENERGIA_ARCH_CC13XX || ENERGIA_ARCH_CC13X2 */
