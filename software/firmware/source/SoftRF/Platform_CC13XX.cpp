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
#include <ti/drivers/Power.h>
#include <ti/drivers/Watchdog.h>
#include <xdc/runtime/Memory.h>

#include "SoCHelper.h"
#include "RFHelper.h"
#include "LEDHelper.h"
#include "SoundHelper.h"
#include "BaroHelper.h"

#include "EasyLink.h"

#if defined(ASSERT)
#undef ASSERT
#endif

#if defined(ENERGIA_ARCH_CC13XX)
#include <ti/devices/cc13x0/driverlib/sys_ctrl.h>
#include <ti/devices/cc13x0/driverlib/aon_batmon.h>
#include <SCSerial.h>
#endif /* ENERGIA_ARCH_CC13XX */

#if defined(ENERGIA_ARCH_CC13X2)
#include <ti/devices/cc13x2_cc26x2/driverlib/sys_ctrl.h>
#include <ti/devices/cc13x2_cc26x2/driverlib/aon_batmon.h>

#include <SPIFlash.h>
#include <ADXL362.h>
#endif /* ENERGIA_ARCH_CC13X2 */

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
static uint32_t cc13xx_vdd = 0;

#if defined(ENERGIA_ARCH_CC13XX)

SCSerial scSerial;

#elif defined(ENERGIA_ARCH_CC13X2)

SPIFlash flash(SOC_GPIO_PIN_MX25_SS); // MACRONIX_MX25R8035F

ADXL362 adxl;

static Watchdog_Handle cc13xx_watchdogHandle;

static void Uart2_ReadCallback(UART_Handle uart, void *buf, size_t count)
{
    Serial2.readCallback(uart, buf, count);
}

static void Uart2_WriteCallback(UART_Handle uart, void *buf, size_t count)
{
    Serial2.writeCallback(uart, buf, count);
}

HardwareSerial Serial2(1, Uart2_ReadCallback, Uart2_WriteCallback, true);

#include <ti/common/cc26xx/oad/oad_image_header.h>
#include <ti/drivers/dpl/HwiP.h>
#include <ti/devices/cc13x2_cc26x2/driverlib/flash.h>

extern const imgHdr_t _imgHdr;

// Boot check timings in milliseconds
#define BOOTCHECK_SLEEP_DURATION_MS       5000
#define BOOTCHECK_SLEEP_INTERVAL_MS        200

static void RevertToFactoryImage(void)
{
    uint32_t key = HwiP_disable();

    uint8_t invalidCrc = CRC_INVALID;
    // Invalidate  CRC in current image header
    uint32_t retVal = FlashProgram(&invalidCrc,
                                 (uint32_t)&_imgHdr.fixedHdr.crcStat,
                                 sizeof(invalidCrc));
    if (retVal == FAPI_STATUS_SUCCESS)
    {
      Serial.println(F("CRC Status invalidated. Rebooting into BIM."));
      Serial.flush();

      SysCtrlSystemReset();
      // We never reach here.
    }
    else
    {
      Serial.print(F("CRC Invalidate write failed. Status 0x"));
      Serial.println(retVal, HEX);
      Serial.println(F("Continuing with on-chip application"));
    }

    HwiP_restore(key);
}

static void BootManagerCheck(void)
{
  bool revertIoInit;
  uint8_t CurrentLed;

  uint32_t addr = (uint32_t) &_imgHdr;

  if (addr) return;

  // Check if button is held on reset
  revertIoInit = (digitalRead(PUSH1) == LOW);

  uint8_t leds[] = {RED_LED, GREEN_LED, SOC_GPIO_PIN_LED_BLUE};
  uint8_t LED_count = (cc13xx_board == TI_LPSTK_CC1352R ? 3 : 2);

  if (revertIoInit)
  {
    Serial.println(F("Left button was held on reset."));
    Serial.println(F("Continue holding for 5 seconds to revert to factory image."));
    Serial.flush();

    if (RED_LED)                          pinMode(RED_LED,   OUTPUT);
    if (GREEN_LED)                        pinMode(GREEN_LED, OUTPUT);
    if (cc13xx_board == TI_LPSTK_CC1352R) pinMode(SOC_GPIO_PIN_LED_BLUE,  OUTPUT);

    // Check to see if button is continued to be held
    for (uint32_t i = 0; i < BOOTCHECK_SLEEP_DURATION_MS / BOOTCHECK_SLEEP_INTERVAL_MS; ++i)
    {
      CurrentLed = leds[i % LED_count];

      if (CurrentLed) digitalWrite(CurrentLed, HIGH);

      delay(BOOTCHECK_SLEEP_INTERVAL_MS);

      if (CurrentLed) digitalWrite(CurrentLed, LOW);

      if (digitalRead(PUSH1) == HIGH)
      {
        revertIoInit = false;
        Serial.println(F("Left button released"));
        Serial.println(F("Continuing with boot"));
        Serial.flush();
        break;
      }
    }
  }

  if(revertIoInit)
  {
      RevertToFactoryImage();
  }

  return;
}

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

#else
#error "This hardware platform is not supported!"
#endif /* ENERGIA_ARCH_CC13X0 & ENERGIA_ARCH_CC13X2 */

static void CC13XX_setup()
{
  uint32_t reset_source = SysCtrlResetSourceGet();

  switch (reset_source)
  {
    case RSTSRC_PIN_RESET:
      reset_info.reason = REASON_EXT_SYS_RST; break;
    case RSTSRC_VDDS_LOSS:
      reset_info.reason = REASON_WDT_RST; break;
    case RSTSRC_VDDR_LOSS:
      reset_info.reason = REASON_WDT_RST; break;
    case RSTSRC_CLK_LOSS:
      reset_info.reason = REASON_WDT_RST; break;
    case RSTSRC_SYSRESET:
      reset_info.reason = REASON_SOFT_RESTART; break;
    case RSTSRC_WARMRESET:
      reset_info.reason = REASON_SOFT_RESTART; break;
    case RSTSRC_WAKEUP_FROM_SHUTDOWN:
      reset_info.reason = REASON_DEEP_SLEEP_AWAKE; break;
#if defined(ENERGIA_ARCH_CC13X2)
    case RSTSRC_WAKEUP_FROM_TCK_NOISE:
      reset_info.reason = REASON_DEEP_SLEEP_AWAKE; break;
#endif /* ENERGIA_ARCH_CC13X2 */
    case RSTSRC_PWR_ON:
    default:
      reset_info.reason = REASON_DEFAULT_RST; break;
  }

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
  SPI.end(SOC_GPIO_PIN_ADXL_SS);

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

  if (SOC_GPIO_PIN_GNSS_PPS != SOC_UNUSED_PIN) {
    uint8_t pps_led = (cc13xx_board == TI_LPSTK_CC1352R ? SOC_GPIO_PIN_LED_BLUE : RED_LED);

    pinMode(pps_led, OUTPUT);
    /* Indicate GNSS PPS signal */
    digitalWrite(pps_led, LOW);
  }
#endif /* ENERGIA_ARCH_CC13X2 */

  hw_info.revision = cc13xx_board;
}

static void CC13XX_loop()
{
#if defined(ENERGIA_ARCH_CC13X2)

  if (cc13xx_watchdogHandle) {
    // Reload the watchdog
    Watchdog_clear(cc13xx_watchdogHandle);
  }

  uint8_t pps_led = (cc13xx_board == TI_LPSTK_CC1352R ? SOC_GPIO_PIN_LED_BLUE : RED_LED);
  if ((SOC_GPIO_PIN_GNSS_PPS != SOC_UNUSED_PIN) &&
      (millis() - PPS_TimeMarker) < 100) {
    digitalWrite(pps_led, HIGH);
  } else {
    digitalWrite(pps_led, LOW);
  }
#endif /* ENERGIA_ARCH_CC13X2 */
}

#if defined(USE_SERIAL_DEEP_SLEEP)

#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>

/* Wake-up UART0 pin table */
PIN_Config UART0_TableWakeUp[] = {
    IOID_12 | PIN_INPUT_EN | PIN_PULLUP | PINCC26XX_WAKEUP_NEGEDGE,
    PIN_TERMINATE                         /* Terminate list */
};
#endif

static void CC13XX_fini()
{
  // Disable battery monitoring
  AONBatMonDisable();

#if defined(ENERGIA_ARCH_CC13X2)

  if (SOC_GPIO_PIN_STATUS != SOC_UNUSED_PIN) {
    digitalWrite(SOC_GPIO_PIN_STATUS, LOW);
    pinMode(SOC_GPIO_PIN_STATUS, INPUT);
  }

  if ((SOC_GPIO_PIN_GNSS_PPS != SOC_UNUSED_PIN)) {
    uint8_t pps_led = (cc13xx_board == TI_LPSTK_CC1352R ? SOC_GPIO_PIN_LED_BLUE : RED_LED);
    digitalWrite(pps_led, LOW);
    pinMode(pps_led, INPUT);
  }

#endif /* ENERGIA_ARCH_CC13X2 */

#if defined(USE_SERIAL_DEEP_SLEEP)
  Serial.end();

  /* Configure DIO for wake up from shutdown */
  PINCC26XX_setWakeup(UART0_TableWakeUp);
#endif

  Power_shutdown(0, 0);
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
  return (void *) &reset_info;
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
  switch (reset_info.reason)
  {
    case REASON_DEFAULT_RST       : return F("DEFAULT");
    case REASON_WDT_RST           : return F("WDT");
    case REASON_EXCEPTION_RST     : return F("EXCEPTION");
    case REASON_SOFT_WDT_RST      : return F("SOFT_WDT");
    case REASON_SOFT_RESTART      : return F("SOFT_RESTART");
    case REASON_DEEP_SLEEP_AWAKE  : return F("DEEP_SLEEP_AWAKE");
    case REASON_EXT_SYS_RST       : return F("EXT_SYS");
    default                       : return F("NO_MEAN");
  }
}

static uint32_t CC13XX_getFreeHeap()
{
  Memory_Stats memStat;

  Memory_getStats(NULL, &memStat);

  return memStat.totalFreeSize;
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
  /* init SPI bus of LoRa radio */
  SPI.begin(SOC_GPIO_PIN_SS);
}

static void CC13XX_swSer_begin(unsigned long baud)
{
  swSer.begin(baud);
}

static void CC13XX_swSer_enableRx(boolean arg)
{
#if defined(ENERGIA_ARCH_CC13XX)
  swSer.enableRx(arg);
#endif /* ENERGIA_ARCH_CC13XX */
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

      if (settings->mode        == SOFTRF_MODE_RECEIVER ||
          settings->rf_protocol == RF_PROTOCOL_ADSB_UAT ||
          settings->txpower     == RF_TX_POWER_OFF) {
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
  // Enable battery monitoring
  AONBatMonEnable();
}

static float CC13XX_Battery_voltage()
{
  if (AONBatMonNewBatteryMeasureReady()) {
    // Get the VDDS voltage
    cc13xx_vdd = AONBatMonBatteryVoltageGet();
  }

  return ((cc13xx_vdd >> 8) & 0x7) + (float) (cc13xx_vdd & 0xFF) / 256.0;
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
#if defined(ENERGIA_ARCH_CC13X2)
  Watchdog_Params params;
  uint32_t        reloadValue;

  Watchdog_init();

  /* Open a Watchdog driver instance */
  Watchdog_Params_init(&params);
  params.callbackFxn = (Watchdog_Callback) NULL;
  params.debugStallMode = Watchdog_DEBUG_STALL_ON;
  params.resetMode = Watchdog_RESET_ON;

  cc13xx_watchdogHandle = Watchdog_open(Board_WATCHDOG, &params);
  if (cc13xx_watchdogHandle) {
    /*
     * The watchdog reload value is initialized during the
     * Watchdog_open() call. The reload value can also be
     * set dynamically during runtime.
     *
     * Converts TIMEOUT_MS to watchdog clock ticks.
     * This API is not applicable for all devices.
     * See the device specific watchdog driver documentation
     * for your device.
     */
    reloadValue = Watchdog_convertMsToTicks(cc13xx_watchdogHandle, WD_TIMEOUT_MS);
  
    /*
     * A value of zero (0) indicates the converted value exceeds 32 bits
     * OR that the API is not applicable for this specific device.
     */
    if (reloadValue != 0) {
        Watchdog_setReload(cc13xx_watchdogHandle, reloadValue);
    }
  }
#endif /* ENERGIA_ARCH_CC13X2 */
}

static void CC13XX_WDT_fini()
{
#if defined(ENERGIA_ARCH_CC13X2)
  if (cc13xx_watchdogHandle) {
    Watchdog_close(cc13xx_watchdogHandle);
  }
#endif /* ENERGIA_ARCH_CC13X2 */
}

static void CC13XX_Button_setup()
{
#if defined(ENERGIA_ARCH_CC13X2)
  pinMode(PUSH1, INPUT_PULLUP);
  pinMode(PUSH2, INPUT_PULLUP);

  BootManagerCheck();
#endif
}

static void CC13XX_Button_loop()
{
#if 0
  if (digitalRead(PUSH1) == LOW) {
    Serial.println(F("PUSH1 PRESSED"));
    Serial.flush();
  }
  if (digitalRead(PUSH2) == LOW) {
    Serial.println(F("PUSH2 PRESSED"));
    Serial.flush();
  }
#endif

  /* TODO */
}

static void CC13XX_Button_fini()
{
#if defined(ENERGIA_ARCH_CC13X2)
  pinMode(PUSH1, INPUT);
  pinMode(PUSH2, INPUT);
#endif
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
  CC13XX_WDT_fini,
  CC13XX_Button_setup,
  CC13XX_Button_loop,
  CC13XX_Button_fini
};

#endif /* ENERGIA_ARCH_CC13XX || ENERGIA_ARCH_CC13X2 */
