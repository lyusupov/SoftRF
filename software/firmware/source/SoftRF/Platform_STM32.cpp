/*
 * Platform_STM32.cpp
 * Copyright (C) 2019 Linar Yusupov
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

#if defined(ARDUINO_ARCH_STM32)

#include <SPI.h>
#include <Wire.h>
#include <IWatchdog.h>

#include "SoCHelper.h"
#include "RFHelper.h"
#include "LEDHelper.h"
#include "SoundHelper.h"
#include "EEPROMHelper.h"

#include <U8x8lib.h>

// RFM95W pin mapping
lmic_pinmap lmic_pins = {
    .nss = SOC_GPIO_PIN_SS,
    .rxtx = { LMIC_UNUSED_PIN, SOC_GPIO_PIN_ANT_RXTX },
#if !defined(USE_OGN_RF_DRIVER)
    .rst = LMIC_UNUSED_PIN,
    .dio = {LMIC_UNUSED_PIN, LMIC_UNUSED_PIN, LMIC_UNUSED_PIN},
#else
    .rst = SOC_GPIO_PIN_RST,
    .dio = {SOC_GPIO_PIN_DIO0, LMIC_UNUSED_PIN, LMIC_UNUSED_PIN},
#endif
};

#if defined(ARDUINO_NUCLEO_L073RZ)

HardwareSerial Serial1(SOC_GPIO_PIN_CONS_RX,  SOC_GPIO_PIN_CONS_TX);
HardwareSerial Serial4(SOC_GPIO_PIN_SWSER_RX, SOC_GPIO_PIN_SWSER_TX);

#elif defined(ARDUINO_BLUEPILL_F103C8)

HardwareSerial Serial2(SOC_GPIO_PIN_SWSER_RX, SOC_GPIO_PIN_SWSER_TX);
HardwareSerial Serial3(SOC_GPIO_PIN_RX3,      SOC_GPIO_PIN_TX3);

#else
#error "This hardware platform is not supported!"
#endif

// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIX_NUM, SOC_GPIO_PIN_LED,
                              NEO_GRB + NEO_KHZ800);

#if defined(USE_OLED)
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8_i2c(U8X8_PIN_NONE);
#endif /* USE_OLED */

static U8X8_SSD1306_128X64_NONAME_HW_I2C *u8x8 = NULL;

char UDPpacketBuffer[4]; // Dummy definition to satisfy build sequence

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

static struct rst_info reset_info = {
  .reason = REASON_DEFAULT_RST,
};

static void STM32_setup()
{
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST))
    {
        reset_info.reason = REASON_WDT_RST; // "LOW_POWER_RESET"
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST))
    {
        reset_info.reason = REASON_WDT_RST; // "WINDOW_WATCHDOG_RESET"
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST))
    {
        reset_info.reason = REASON_SOFT_WDT_RST; // "INDEPENDENT_WATCHDOG_RESET"
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST))
    {
        // This reset is induced by calling the ARM CMSIS `NVIC_SystemReset()` function!
        reset_info.reason = REASON_SOFT_RESTART; // "SOFTWARE_RESET"
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST))
    {
        reset_info.reason = REASON_DEFAULT_RST; // "POWER-ON_RESET (POR) / POWER-DOWN_RESET (PDR)"
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST))
    {
        reset_info.reason = REASON_EXT_SYS_RST; // "EXTERNAL_RESET_PIN_RESET"
    }

    // Clear all the reset flags or else they will remain set during future resets until system power is fully removed.
    __HAL_RCC_CLEAR_RESET_FLAGS();

  /* TBD */
}

static void STM32_loop()
{
  // Reload the watchdog
  IWatchdog.reload();
}

static void STM32_fini()
{
  /* TBD */
}

static void STM32_reset()
{
  HAL_NVIC_SystemReset();
}

static uint32_t STM32_getChipId()
{
  uint32_t UniqueID[3];

  HAL_GetUID(UniqueID);

  /* Same method as STM32 OGN tracker does */
  return UniqueID[0] ^ UniqueID[1] ^ UniqueID[2];
}

static void* STM32_getResetInfoPtr()
{
  return (void *) &reset_info;
}

static String STM32_getResetReason()
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

static long STM32_random(long howsmall, long howBig)
{
  return random(howsmall, howBig);
}

static void STM32_Sound_test(int var)
{
  if (settings->volume != BUZZER_OFF) {
    tone(SOC_GPIO_PIN_BUZZER, 440,  500); delay(500);
    tone(SOC_GPIO_PIN_BUZZER, 640,  500); delay(500);
    tone(SOC_GPIO_PIN_BUZZER, 840,  500); delay(500);
    tone(SOC_GPIO_PIN_BUZZER, 1040, 500); delay(600);
  }
}

static void STM32_WiFi_setOutputPower(int dB)
{
  /* NONE */
}

static void STM32_WiFi_transmit_UDP(int port, byte *buf, size_t size)
{
  /* NONE */
}

static bool STM32_EEPROM_begin(size_t size)
{
  if (size > E2END) {
    return false;
  }

  EEPROM.begin();

  return true;
}

static void STM32_SPI_begin()
{
  SPI.setMISO(SOC_GPIO_PIN_MISO);
  SPI.setMOSI(SOC_GPIO_PIN_MOSI);
  SPI.setSCLK(SOC_GPIO_PIN_SCK);
  SPI.setSSEL(SOC_GPIO_PIN_SS);

  SPI.begin();
}

static void STM32_swSer_begin(unsigned long baud)
{
#if !defined(ARDUINO_NUCLEO_L073RZ)
  swSer.begin(baud);
#else
  /* drive GNSS RST pin low */
  pinMode(SOC_GPIO_PIN_GNSS_RST, OUTPUT);
  digitalWrite(SOC_GPIO_PIN_GNSS_RST, LOW);

  /* activate 1.8V<->3.3V level shifters */
  pinMode(SOC_GPIO_PIN_GNSS_LS,  OUTPUT);
  digitalWrite(SOC_GPIO_PIN_GNSS_LS,  HIGH);

  /* keep RST low to ensure proper IC reset */
  delay(200);

  /* release */
  digitalWrite(SOC_GPIO_PIN_GNSS_RST, HIGH);

  /* give Sony GNSS few ms to warm up */
  delay(100);

  /* S76G GNSS is operating at 115200 baud by default */
  swSer.begin(115200);

  // swSer.write("@VER\r\n");

  if (reset_info.reason == REASON_DEFAULT_RST) {
    /* Idle */
    swSer.write("@GSTP\r\n");      delay(250);
    /* GGA + GSA + RMC */
    swSer.write("@BSSL 0x25\r\n"); delay(250);
    /* GPS + GLONASS */
    swSer.write("@GNS 0x3\r\n");   delay(250);
#if SOC_GPIO_PIN_GNSS_PPS != SOC_UNUSED_PIN
    /* Enable 1PPS output */
    swSer.write("@GPPS 0x1\r\n");   delay(250);
#endif
    /* hot start */
    swSer.write("@GSR\r\n");       delay(250);
  }
#endif /* ARDUINO_NUCLEO_L073RZ */
}

static void STM32_swSer_enableRx(boolean arg)
{
  /* NONE */
}

static byte STM32_Display_setup()
{
  byte rval = DISPLAY_NONE;

#if defined(USE_OLED)
  /* SSD1306 I2C OLED probing */
  Wire.begin();
  Wire.beginTransmission(SSD1306_OLED_I2C_ADDR);
  if (Wire.endTransmission() == 0) {
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

static void STM32_Display_loop()
{
#if defined(USE_OLED)
  char buf[16];
  uint32_t disp_value;

  if (u8x8) {
    if (!OLED_display_frontpage) {

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

      itoa(tx_packets_counter % 1000, buf, 10);
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

static void STM32_Display_fini(const char *msg)
{
#if defined(USE_OLED)
  if (u8x8) {
    u8x8->setFont(u8x8_font_chroma48medium8_r);
    u8x8->clear();
    u8x8->draw2x2String(1, 3, msg);
  }
#endif /* USE_OLED */
}

static void STM32_Battery_setup()
{

}

static float STM32_Battery_voltage()
{
  return analogRead (SOC_GPIO_PIN_BATTERY) / SOC_ADC9_VOLTAGE_DIVIDER ;
}

void STM32_GNSS_PPS_Interrupt_handler() {
  PPS_TimeMarker = millis();
}

static unsigned long STM32_get_PPS_TimeMarker() {
  return PPS_TimeMarker;
}

static bool STM32_Baro_setup() {
  return true;
}

static void STM32_UATSerial_begin(unsigned long baud)
{
  UATSerial.begin(baud);
}

static void STM32_restart()
{
  digitalWrite(SOC_GPIO_PIN_TXE, LOW);
  pinMode(SOC_GPIO_PIN_TXE, OUTPUT);

  delay(100);

  digitalWrite(SOC_GPIO_PIN_TXE, HIGH);

  delay(100);

  pinMode(SOC_GPIO_PIN_TXE, INPUT);
}

static void STM32_WDT_setup()
{
  // Init the watchdog timer with 5 seconds timeout
  IWatchdog.begin(5000000);
}

static void STM32_WDT_fini()
{
  /* once emabled - there is no way to disable WDT on STM32 */
}

#if defined(USBD_USE_CDC) && defined(ARDUINO_BLUEPILL_F103C8)

#include <USBSerial.h>
#include "BluetoothHelper.h"

static void STM32_USB_setup()
{
  SerialUSB.begin();
}

static void STM32_USB_loop()
{

}

static int STM32_USB_available()
{
  return SerialUSB.available();
}

static int STM32_USB_read()
{
  return SerialUSB.read();
}

static size_t STM32_USB_write(const uint8_t *buffer, size_t size)
{
  return SerialUSB.write(buffer, size);
}

Bluetooth_ops_t STM32_USBSerial_ops = {
  "STM32 USBSerial",
  STM32_USB_setup,
  STM32_USB_loop,
  STM32_USB_available,
  STM32_USB_read,
  STM32_USB_write
};

#endif /* ARDUINO_BLUEPILL_F103C8 */

const SoC_ops_t STM32_ops = {
  SOC_STM32,
  "STM32",
  STM32_setup,
  STM32_loop,
  STM32_fini,
  STM32_reset,
  STM32_getChipId,
  STM32_getResetInfoPtr,
  NULL,
  STM32_getResetReason,
  STM32_random,
  STM32_Sound_test,
  NULL,
  STM32_WiFi_setOutputPower,
  STM32_WiFi_transmit_UDP,
  NULL,
  NULL,
  NULL,
  STM32_EEPROM_begin,
  STM32_SPI_begin,
  STM32_swSer_begin,
  STM32_swSer_enableRx,
#if defined(USBD_USE_CDC) && defined(ARDUINO_BLUEPILL_F103C8)
  &STM32_USBSerial_ops,
#else
  NULL,
#endif
  STM32_Display_setup,
  STM32_Display_loop,
  STM32_Display_fini,
  STM32_Battery_setup,
  STM32_Battery_voltage,
  STM32_GNSS_PPS_Interrupt_handler,
  STM32_get_PPS_TimeMarker,
  STM32_Baro_setup,
  STM32_UATSerial_begin,
  STM32_restart,
  STM32_WDT_setup,
  STM32_WDT_fini
};

#endif /* ARDUINO_ARCH_STM32 */
