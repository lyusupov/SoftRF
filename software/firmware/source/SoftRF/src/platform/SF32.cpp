/*
 * Platform_SF32.cpp
 * Copyright (C) 2026 Linar Yusupov
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

#if defined(ARDUINO_ARCH_SF32LB52)

#include <SPI.h>
#include <Wire.h>

//#include <ArduinoJson.h>

#include "../system/SoC.h"
#include "../driver/RF.h"
#include "../driver/EEPROM.h"
#include "../driver/GNSS.h"
#include "../driver/Baro.h"
#include "../driver/LED.h"
#include "../driver/Bluetooth.h"
#include "../driver/EPD.h"
#include "../driver/Battery.h"
#include "../driver/Sound.h"
#include "../protocol/data/NMEA.h"
#include "../protocol/data/GDL90.h"
#include "../protocol/data/D1090.h"
//#include "../protocol/data/JSON.h"
#include "../system/Time.h"

// RFM95W pin mapping
lmic_pinmap lmic_pins = {
    .nss = LMIC_UNUSED_PIN,
    .txe = LMIC_UNUSED_PIN,
    .rxe = LMIC_UNUSED_PIN,
    .rst = LMIC_UNUSED_PIN,
    .dio = {LMIC_UNUSED_PIN, LMIC_UNUSED_PIN, LMIC_UNUSED_PIN},
    .busy = LMIC_UNUSED_PIN,
    .tcxo = LMIC_UNUSED_PIN,
};

static struct rst_info reset_info = {
  .reason = REASON_DEFAULT_RST,
};

static uint32_t bootCount __attribute__ ((section (".noinit")));

static SF32_board_id SF32_board = SF32_LB52_DEVKIT; /* default */

const char *SF32_Device_Manufacturer = SOFTRF_IDENT;
const char *SF32_Device_Model = "Academy Edition";

#if defined(EXCLUDE_EEPROM)
eeprom_t eeprom_block;
settings_t *settings = &eeprom_block.field.settings;
#endif /* EXCLUDE_EEPROM */

#if defined(EXCLUDE_WIFI)
char UDPpacketBuffer[256]; // Dummy definition to satisfy build sequence
#endif /* EXCLUDE_WIFI */

char *dtostrf_workaround(double number, signed char width, unsigned char prec, char *s) {
    bool negative = false;

    if (isnan(number)) {
        strcpy(s, "nan");
        return s;
    }
    if (isinf(number)) {
        strcpy(s, "inf");
        return s;
    }

    char* out = s;

    int fillme = width; // how many cells to fill for the integer part
    if (prec > 0) {
        fillme -= (prec+1);
    }

    // Handle negative numbers
    if (number < 0.0) {
        negative = true;
        fillme--;
        number = -number;
    }

    // Round correctly so that print(1.999, 2) prints as "2.00"
    // I optimized out most of the divisions
    double rounding = 2.0;
    for (uint8_t i = 0; i < prec; ++i)
        rounding *= 10.0;
    rounding = 1.0 / rounding;

    number += rounding;

    // Figure out how big our number really is
    double tenpow = 1.0;
    int digitcount = 1;
    while (number >= 10.0 * tenpow) {
        tenpow *= 10.0;
        digitcount++;
    }

    number /= tenpow;
    fillme -= digitcount;

    // Pad unused cells with spaces
    while (fillme-- > 0) {
        *out++ = ' ';
    }

    // Handle negative sign
    if (negative) *out++ = '-';

    // Print the digits, and if necessary, the decimal point
    digitcount += prec;
    int8_t digit = 0;
    while (digitcount-- > 0) {
        digit = (int8_t)number;
        if (digit > 9) digit = 9; // insurance
        *out++ = (char)('0' | digit);
        if ((digitcount == prec) && (prec > 0)) {
            *out++ = '.';
        }
        number -= digit;
        number *= 10.0;
    }

    // make sure the string is terminated
    *out = 0;
    return s;
}


static void SF32_setup()
{

}

static void SF32_post_init()
{

}

static void SF32_loop()
{

}

static void SF32_fini(int reason)
{

}

static void SF32_reset()
{

}

static uint32_t SF32_getChipId()
{
  return 0;
}

static void* SF32_getResetInfoPtr()
{
  return (void *) &reset_info;
}

static String SF32_getResetInfo()
{
  switch (reset_info.reason)
  {
    default                     : return F("No reset information available");
  }
}

static String SF32_getResetReason()
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

static uint32_t SF32_getFreeHeap()
{
  return 0; /* TBD */
}

static long SF32_random(long howsmall, long howBig)
{
  return random(howsmall, howBig);
}


static void SF32_Sound_test(int var)
{

}

static void SF32_Sound_tone(int hz, uint8_t volume)
{

}

static void SF32_WiFi_set_param(int ndx, int value)
{
  /* NONE */
}

static void SF32_WiFi_transmit_UDP(int port, byte *buf, size_t size)
{
  /* NONE */
}

static bool SF32_EEPROM_begin(size_t size)
{
#if !defined(EXCLUDE_EEPROM)
  if (size > EEPROM.length()) {
    return false;
  }

  EEPROM.begin();
#endif /* EXCLUDE_EEPROM */

  return true;
}

static void SF32_EEPROM_extension(int cmd)
{

}

static void SF32_SPI_begin()
{
  SPI.begin();
}

static void SF32_swSer_begin(unsigned long baud)
{
  Serial_GNSS_In.begin(baud);
}

static void SF32_swSer_enableRx(boolean arg)
{
  /* NONE */
}

static byte SF32_Display_setup()
{
  return 0;
}

static void SF32_Display_loop()
{

}

static void SF32_Display_fini(int reason)
{

}

static void SF32_Battery_setup()
{

}

static float SF32_Battery_param(uint8_t param)
{
  float rval = 0;

  return rval;
}

void SF32_GNSS_PPS_Interrupt_handler() {
  PPS_TimeMarker = millis();
}

static unsigned long SF32_get_PPS_TimeMarker() {
  return PPS_TimeMarker;
}

static bool SF32_Baro_setup() {
  return false;
}

static void SF32_UATSerial_begin(unsigned long baud)
{

}

static void SF32_UATModule_restart()
{

}

static void SF32_WDT_setup()
{

}

static void SF32_WDT_fini()
{

}

static void SF32_Button_setup()
{

}

static void SF32_Button_loop()
{

}

static void SF32_Button_fini()
{

}

static void SF32_USB_setup()
{

}

static void SF32_USB_loop()
{

}

static void SF32_USB_fini()
{

}

static int SF32_USB_available()
{
  int rval = 0;

  return rval;
}

static int SF32_USB_read()
{
  int rval = -1;

  return rval;
}

static size_t SF32_USB_write(const uint8_t *buffer, size_t size)
{
  size_t rval = size;

  return rval;
}

IODev_ops_t SF32_USBSerial_ops = {
  "SF32 USBSerial",
  SF32_USB_setup,
  SF32_USB_loop,
  SF32_USB_fini,
  SF32_USB_available,
  SF32_USB_read,
  SF32_USB_write
};

static void SF32_TTS(char *message)
{

}

static bool SF32_ADB_setup()
{
  return false;
}

static bool SF32_ADB_fini()
{
  return false;
}

/*
 * One aircraft CDB (20000+ records) query takes:
 * 1)     FOUND : 5-7 milliseconds
 * 2) NOT FOUND :   3 milliseconds
 */
static bool SF32_ADB_query(uint8_t type, uint32_t id, char *buf, size_t size)
{
  return false;
}

DB_ops_t SF32_ADB_ops = {
  SF32_ADB_setup,
  SF32_ADB_fini,
  SF32_ADB_query
};

const SoC_ops_t SF32_ops = {
  SOC_SF32,
  "SF32",
  SF32_setup,
  SF32_post_init,
  SF32_loop,
  SF32_fini,
  SF32_reset,
  SF32_getChipId,
  SF32_getResetInfoPtr,
  SF32_getResetInfo,
  SF32_getResetReason,
  SF32_getFreeHeap,
  SF32_random,
  SF32_Sound_test,
  SF32_Sound_tone,
  NULL,
  SF32_WiFi_set_param,
  SF32_WiFi_transmit_UDP,
  NULL,
  NULL,
  NULL,
  SF32_EEPROM_begin,
  SF32_EEPROM_extension,
  SF32_SPI_begin,
  SF32_swSer_begin,
  SF32_swSer_enableRx,
  NULL,
  &SF32_USBSerial_ops,
  NULL,
  SF32_Display_setup,
  SF32_Display_loop,
  SF32_Display_fini,
  SF32_Battery_setup,
  SF32_Battery_param,
  SF32_GNSS_PPS_Interrupt_handler,
  SF32_get_PPS_TimeMarker,
  SF32_Baro_setup,
  SF32_UATSerial_begin,
  SF32_UATModule_restart,
  SF32_WDT_setup,
  SF32_WDT_fini,
  SF32_Button_setup,
  SF32_Button_loop,
  SF32_Button_fini,
  SF32_TTS,
  &SF32_ADB_ops
};

#endif /* ARDUINO_ARCH_SF32LB52 */
