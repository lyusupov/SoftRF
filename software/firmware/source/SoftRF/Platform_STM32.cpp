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
    .rxtx = { LMIC_UNUSED_PIN, LMIC_UNUSED_PIN },
    .rst = LMIC_UNUSED_PIN,
    .dio = {LMIC_UNUSED_PIN, LMIC_UNUSED_PIN, LMIC_UNUSED_PIN},
};

HardwareSerial Serial2(SOC_GPIO_PIN_SWSER_RX, SOC_GPIO_PIN_SWSER_TX);
HardwareSerial Serial3(SOC_GPIO_PIN_RX3, SOC_GPIO_PIN_TX3);

// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIX_NUM, SOC_GPIO_PIN_LED,
                              NEO_GRB + NEO_KHZ800);

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8_i2c(U8X8_PIN_NONE);

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

static void STM32_setup()
{
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

static uint32_t STM32_getChipId()
{
  uint32_t UniqueID[3];

  HAL_GetUID(UniqueID);

  /* Same method as STM32 OGN tracker does */
  return UniqueID[0] ^ UniqueID[1] ^ UniqueID[2];
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
  SPI.begin();
}

static void STM32_swSer_begin(unsigned long baud)
{
  swSer.begin(baud);
}

static void STM32_swSer_enableRx(boolean arg)
{
  /* NONE */
}

static byte STM32_Display_setup()
{
  byte rval = DISPLAY_NONE;

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

  return rval;
}

static void STM32_Display_loop()
{
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
}

static void STM32_Display_fini(const char *msg)
{
  if (u8x8) {
    u8x8->setFont(u8x8_font_chroma48medium8_r);
    u8x8->clear();
    u8x8->draw2x2String(1, 3, msg);
  }
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
  STM32_getChipId,
  NULL,
  NULL,
  NULL,
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
