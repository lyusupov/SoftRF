/*
 * Platform_LPC43.cpp
 * Copyright (C) 2021-2025 Linar Yusupov
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

#if defined(HACKRF_ONE)

#include "src/system/Time.h"
#include "src/driver/LED.h"
#include "src/driver/GNSS.h"
#include "src/driver/RF.h"
#include "src/driver/Sound.h"
#include "src/driver/EEPROM.h"
#include "src/driver/Battery.h"
#include "src/protocol/data/GDL90.h"
#include "src/protocol/data/NMEA.h"
#include "src/protocol/data/D1090.h"
#include "src/system/SoC.h"
#include "src/driver/Baro.h"
#include <src/TrafficHelper.h>
#include "src/protocol/data/JSON.h"
#include "src/driver/Bluetooth.h"
#include "src/driver/OLED.h"

#include "Adafruit_USBD_Device.h"
#include "Uart.h"

Uart Serial1(LPC43_UART0);
Uart Serial4(LPC43_UART3);

ufo_t ThisAircraft;

uint32_t tx_packets_counter = 0;
uint32_t rx_packets_counter = 0;

char UDPpacketBuffer[UDP_PACKET_BUFSIZE]; // buffer to hold GDL90 frames
void RF_Shutdown() { }   // Dummy definition to satisfy build sequence

static struct rst_info reset_info = {
  .reason = REASON_DEFAULT_RST,
};

hardware_info_t hw_info = {
  .model    = DEFAULT_SOFTRF_MODEL,
  .revision = 0,
  .soc      = SOC_NONE,
  .rf       = RF_IC_MAX2837,
  .gnss     = GNSS_MODULE_NONE,
  .baro     = BARO_MODULE_NONE,
  .display  = DISPLAY_NONE,
  .storage  = STORAGE_NONE,
  .rtc      = RTC_NONE,
  .imu      = IMU_NONE,
  .mag      = MAG_NONE,
  .pmu      = PMU_NONE,
};

const uint16_t LPC43_Vendor_Id = 0x1d50; /* OpenMoko, Inc. */
const uint16_t LPC43_Device_Id = 0x6089; /* HackRF One */
const char *LPC43_Device_Manufacturer = SOFTRF_IDENT;
const char *LPC43_Device_Model        = "ES Edition";
const uint16_t LPC43_Device_Version   = SOFTRF_USB_FW_VERSION;

const char *LPC43_boot_str1 = SOFTRF_IDENT "-" PLAT_LPC43_NAME
                              " FW.REV: " SOFTRF_FIRMWARE_VERSION " DEV.ID: ";
const char *LPC43_boot_str2 = "Copyright (C) 2015-2025 Linar Yusupov. ";
const char *LPC43_boot_str3 = "All rights reserved";

#if defined(USE_PORTAPACK)
extern "C" const void* portapack(void);
#endif /* USE_PORTAPACK */

static bool wdt_is_active = false;
static bool has_portapack = false;

#if defined(USE_OLED)
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
#endif /* USE_OLED */

void LPC43_setup(void)
{
#if defined(USE_PORTAPACK)
  has_portapack = portapack();
#endif /* USE_PORTAPACK */

  if (!has_portapack) {
    SerialOutput.begin(SERIAL_OUT_BR, SERIAL_OUT_BITS);

    SerialOutput.println();
    SerialOutput.print(LPC43_boot_str1);
    SerialOutput.println(SoC->getChipId(), HEX);
    SerialOutput.print(LPC43_boot_str2); SerialOutput.println(LPC43_boot_str3);
    SerialOutput.println();
  }
}

static void LPC43_post_init()
{
  Serial.println();
  Serial.print(LPC43_Device_Manufacturer); Serial.print(' ');
  Serial.print(LPC43_Device_Model); Serial.println(" Power-on Self Test");
//  Serial.println("SoftRF ES Edition Power-on Self Test");
  Serial.println();
  SERIAL_FLUSH();

  Serial.print(F("GNSS    : "));
  Serial.println(hw_info.gnss    != GNSS_MODULE_NONE  ? F("PASS") : F("N/A"));
  Serial.print(F("BARO    : "));
  Serial.println(hw_info.baro    != BARO_MODULE_NONE  ? F("PASS") : F("N/A"));
  Serial.print(F("DISPLAY : "));
  Serial.println(hw_info.display != DISPLAY_NONE      ? F("PASS") : F("N/A"));

  Serial.println();
  Serial.println(F("Power-on Self Test is complete."));
  Serial.println();
  SERIAL_FLUSH();

  Serial.println(F("Data output device(s):"));

  Serial.print(F("NMEA   - "));
  Serial.println(settings->nmea_out == NMEA_UART ? F("UART")    :
                 settings->nmea_out == NMEA_USB  ? F("USB CDC") :
                                                   F("NULL"));
  Serial.print(F("GDL90  - "));
  Serial.println(settings->gdl90 == GDL90_UART  ? F("UART")    :
                 settings->gdl90 == GDL90_USB   ? F("USB CDC") :
                                                  F("NULL"));
  Serial.print(F("D1090  - "));
  Serial.println(settings->d1090 == D1090_UART  ? F("UART")    :
                 settings->d1090 == D1090_USB   ? F("USB CDC") :
                                                  F("NULL"));
  Serial.println();
  SERIAL_FLUSH();

#if defined(USE_OLED)
  OLED_info1();
#endif /* USE_OLED */
}

static uint32_t prev_rx_packets_counter = 0;
static unsigned long rx_led_time_marker = 0;
static bool rx_led_state = false;

#define LED_BLINK_TIME 100

typedef enum {
        LED1 = 0,
        LED2 = 1,
        LED3 = 2,
} led_t;

extern "C" void led_on (const led_t led);
extern "C" void led_off(const led_t led);
extern "C" void led_toggle(const led_t led);

static void LPC43_loop()
{
  if (wdt_is_active) {
    platform_watchdog_feed();
  }

#if SOC_GPIO_RADIO_LED_RX != SOC_UNUSED_PIN
  if (!rx_led_state) {
    if (rx_packets_counter != prev_rx_packets_counter) {
      led_on(LED2);
      rx_led_state = true;
      prev_rx_packets_counter = rx_packets_counter;
      rx_led_time_marker = millis();
    }
  } else {
    if (millis() - rx_led_time_marker > LED_BLINK_TIME) {
      led_off(LED2);
      rx_led_state = false;
      prev_rx_packets_counter = rx_packets_counter;
    }
  }
#endif /* SOC_GPIO_RADIO_LED_RX */
}

static void LPC43_fini(int reason)
{

}

static void LPC43_reset()
{
  system_reset(RESET_REASON_SOFT_RESET, true);
}

extern uint32_t LPC43_Device_ID;

static uint32_t LPC43_getChipId()
{
  return DevID_Mapper(LPC43_Device_ID);
}

static void* LPC43_getResetInfoPtr()
{
  return (void *) &reset_info;
}

static String LPC43_getResetReason()
{
  return (String) system_get_reset_reason_string();
}

extern "C" void * _sbrk   (int);

static uint32_t LPC43_getFreeHeap()
{
  char top;
  return &top - reinterpret_cast<char*>(_sbrk(0));
}

static long LPC43_random(long howsmall, long howBig)
{
  return howsmall + random() % (howBig - howsmall);
}

static void LPC43_WiFi_transmit_UDP(int port, byte *buf, size_t size)
{

}

static bool LPC43_EEPROM_begin(size_t size)
{
  EEPROM.begin(size);

  return true;
}

static void LPC43_EEPROM_extension(int cmd)
{
  if (cmd == EEPROM_EXT_LOAD) {
    settings->mode        = SOFTRF_MODE_NORMAL;
    settings->rf_protocol = RF_PROTOCOL_ADSB_1090;
    settings->txpower     = RF_TX_POWER_OFF;

    if ( settings->nmea_out == NMEA_BLUETOOTH ||
         settings->nmea_out == NMEA_UDP       ||
         settings->nmea_out == NMEA_TCP       ||
        (settings->nmea_out == NMEA_UART && has_portapack)) {
      settings->nmea_out = NMEA_USB;
    }
    if ( settings->gdl90 == GDL90_BLUETOOTH  ||
         settings->gdl90 == GDL90_UDP        ||
        (settings->gdl90 == GDL90_UART && has_portapack)) {
      settings->gdl90 = GDL90_USB;
    }
    if ( settings->d1090 == D1090_BLUETOOTH  ||
         settings->d1090 == D1090_UDP        ||
        (settings->d1090 == D1090_UART && has_portapack)) {
      settings->d1090 = D1090_USB;
    }
  }
}

static void LPC43_SPI_begin()
{

}

static void LPC43_swSer_begin(unsigned long baud)
{
  Serial_GNSS_In.begin(baud);
}

extern "C" {
#include <i2c_lpc.h>
}
extern i2c_bus_t i2c0;

bool LPC43_OLED_probe_func()
{
  return i2c_probe(&i2c0, SSD1306_OLED_I2C_ADDR);
}

static byte LPC43_Display_setup()
{
  byte rval = DISPLAY_NONE;

  rval = has_portapack ? DISPLAY_TFT_PORTAPACK : rval;

#if defined(USE_OLED)
  if (rval != DISPLAY_TFT_PORTAPACK) {
    rval = OLED_setup();
  }
#endif /* USE_OLED */

  return rval;
}

static void LPC43_Display_loop()
{
#if defined(USE_OLED)
  OLED_loop();
#endif /* USE_OLED */
}

static void LPC43_Display_fini(int reason)
{

}

static void LPC43_Battery_setup()
{
  /* TBD */
}

static float LPC43_Battery_param(uint8_t param)
{
  float rval;

  switch (param)
  {
  case BATTERY_PARAM_THRESHOLD:
    rval = BATTERY_THRESHOLD_USB;
    break;

  case BATTERY_PARAM_CUTOFF:
    rval = BATTERY_CUTOFF_USB;
    break;

  case BATTERY_PARAM_CHARGE:
    /* TBD */

    rval = 100;
    break;

  case BATTERY_PARAM_VOLTAGE:
  default:
    rval = BATTERY_THRESHOLD_USB + 0.05;
    break;
  }

  return rval;
}

void LPC43_GNSS_PPS_Interrupt_handler() {
  PPS_TimeMarker = millis();
}

static unsigned long LPC43_get_PPS_TimeMarker() {
  return PPS_TimeMarker;
}

static bool LPC43_Baro_setup() {
  return true;
}

static void LPC43_UATSerial_begin(unsigned long baud)
{

}

static void LPC43_UATModule_restart()
{

}

static void LPC43_WDT_setup()
{
  reset_driver_initialize();
#if SOFTRF_USB_FW_VERSION > 0x0101 /* not active until release */
  platform_watchdog_enable(16000000UL); /* approx. 6 seconds */
  wdt_is_active = true;
#endif
}

static void LPC43_WDT_fini()
{
  /* TBD */
}

#define DFU_CLICK_DELAY     200
#define DFU_LONGPRESS_DELAY 2000

static unsigned long dfu_time_marker = 0;
static bool prev_dfu_state = false;
static bool is_dfu_click = false;

static void On_Button_Click()
{
#if defined(USE_OLED)
  OLED_Next_Page();
#endif
}

static void On_Button_LongPress()
{

}

static void LPC43_Button_setup()
{

}

static void LPC43_Button_loop()
{
  if (dfu_button_state()) {
    if (!prev_dfu_state) {
      dfu_time_marker = millis();
      prev_dfu_state = true;
    } else {
      if (dfu_time_marker && !is_dfu_click &&
          millis() - dfu_time_marker > DFU_CLICK_DELAY) {
        is_dfu_click = true;
      }
      if (dfu_time_marker &&
          millis() - dfu_time_marker > DFU_LONGPRESS_DELAY) {

        On_Button_LongPress();

//      Serial.println(F("This will never be printed."));
      }
    }
  } else {
    if (prev_dfu_state) {
      if (is_dfu_click) {

        On_Button_Click();

        is_dfu_click = false;
      }
      dfu_time_marker = 0;
      prev_dfu_state = false;
    }
  }
}

static void LPC43_Button_fini()
{
  /* TODO */
}

void LPC43_USB_CDC_Sync()
{
  unsigned long ms = millis();
  while (millis() - ms < 200) {
    if (SoC->USB_ops) SoC->USB_ops->loop();
  }
}

static void LPC43_USB_setup()
{
  TinyUSB_Device_Init(0);

  USBDevice.setID(LPC43_Vendor_Id, LPC43_Device_Id);
  USBDevice.setManufacturerDescriptor(LPC43_Device_Manufacturer);
  USBDevice.setProductDescriptor(LPC43_Device_Model);
  USBDevice.setDeviceVersion(LPC43_Device_Version);
}

static bool usb_led_state = false;

static void LPC43_USB_loop()
{
  USBDevice.task();
  Serial.flush();

  if (USBSerial && !usb_led_state) {
    led_on(LED1);
    usb_led_state = true;
  } else if (!USBSerial && usb_led_state) {
    led_off(LED1);
    usb_led_state = false;
  }
}

static void LPC43_USB_fini()
{
#if 0
  if (USBSerial && USBSerial != Serial) {
    USBSerial.end();
  }
#endif
}

static int LPC43_USB_available()
{
  int rval = 0;

  if (USBSerial) {
    rval = USBSerial.available();
  }

  return rval;
}

static int LPC43_USB_read()
{
  int rval = -1;

  if (USBSerial) {
    rval = USBSerial.read();
  }

  return rval;
}

static size_t LPC43_USB_write(const uint8_t *buffer, size_t size)
{
  size_t rval = size;

  if (USBSerial && (size < USBSerial.availableForWrite())) {
    rval = USBSerial.write(buffer, size);
  }

  return rval;
}

IODev_ops_t LPC43_USBSerial_ops = {
  "LPC43 USBSerial",
  LPC43_USB_setup,
  LPC43_USB_loop,
  LPC43_USB_fini,
  LPC43_USB_available,
  LPC43_USB_read,
  LPC43_USB_write
};

const SoC_ops_t LPC43_ops = {
  SOC_LPC43,
  PLAT_LPC43_NAME,
  LPC43_setup,
  LPC43_post_init,
  LPC43_loop,
  LPC43_fini,
  LPC43_reset,
  LPC43_getChipId,
  LPC43_getResetInfoPtr,
  NULL,
  LPC43_getResetReason,
  LPC43_getFreeHeap,
  LPC43_random,
  NULL,
  NULL,
  NULL,
  NULL,
  LPC43_WiFi_transmit_UDP,
  NULL,
  NULL,
  NULL,
  LPC43_EEPROM_begin,
  LPC43_EEPROM_extension,
  LPC43_SPI_begin,
  LPC43_swSer_begin,
  NULL,
  NULL,
  &LPC43_USBSerial_ops,
  NULL,
  LPC43_Display_setup,
  LPC43_Display_loop,
  LPC43_Display_fini,
  LPC43_Battery_setup,
  LPC43_Battery_param,
  NULL,
  LPC43_get_PPS_TimeMarker,
  LPC43_Baro_setup,
  LPC43_UATSerial_begin,
  LPC43_UATModule_restart,
  LPC43_WDT_setup,
  LPC43_WDT_fini,
  LPC43_Button_setup,
  LPC43_Button_loop,
  LPC43_Button_fini,
  NULL
};

void setup_CPP(void)
{
  hw_info.soc = SoC_setup(); // Has to be very first procedure in the execution order

  if (SoC->USB_ops) {
     SoC->USB_ops->setup();
  }

  Serial.begin(SERIAL_OUT_BR);

  unsigned long ms = millis();
  while (millis() - ms < 3000) {
    if (Serial) { delay(1000); break; } else LPC43_USB_CDC_Sync();
  }

  Serial.println();
  Serial.print(LPC43_boot_str1);
  Serial.println(SoC->getChipId(), HEX);
  Serial.print(LPC43_boot_str2); Serial.println(LPC43_boot_str3);
  SERIAL_FLUSH();

  Serial.println();
  Serial.print(F("Reset reason: ")); Serial.print(SoC->getResetReason());
  Serial.print(F(" ("));
  Serial.print(system_reset_reason(), HEX);
  Serial.println(')');
  Serial.print(F("Free heap size: ")); Serial.println(SoC->getFreeHeap());
  SERIAL_FLUSH();

  EEPROM_setup();

  SoC->Button_setup();

  ThisAircraft.addr = SoC->getChipId() & 0x00FFFFFF;
  ThisAircraft.aircraft_type = settings->aircraft_type;
  ThisAircraft.protocol = settings->rf_protocol;
  ThisAircraft.stealth  = settings->stealth;
  ThisAircraft.no_track = settings->no_track;

  hw_info.display = SoC->Display_setup();

  if (hw_info.display != DISPLAY_TFT_PORTAPACK) {
    hw_info.gnss = GNSS_setup();
    hw_info.baro = Baro_setup();
  }

  Traffic_setup();
  NMEA_setup();
  SERIAL_FLUSH();

  SoC->post_init();

  SoC->WDT_setup();
}

void main_loop_CPP(void)
{
  Baro_loop();

  ThisAircraft.timestamp = now();

  GNSS_loop();

  if (isValidFix()) {
    ThisAircraft.latitude         = gnss.location.lat();
    ThisAircraft.longitude        = gnss.location.lng();
    ThisAircraft.altitude         = gnss.altitude.meters();
    ThisAircraft.course           = gnss.course.deg();
    ThisAircraft.speed            = gnss.speed.knots();
    ThisAircraft.hdop             = (uint16_t) gnss.hdop.value();
    ThisAircraft.geoid_separation = gnss.separation.meters();

    Traffic_loop();
  }

  Sound_loop();

  NMEA_loop();

  SoC->Display_loop();

  SoC->loop();

  if (SoC->Bluetooth_ops) {
    SoC->Bluetooth_ops->loop();
  }

  if (SoC->USB_ops) {
    SoC->USB_ops->loop();
  }

  if (SoC->UART_ops) {
     SoC->UART_ops->loop();
  }

  Battery_loop();

  SoC->Button_loop();

  Time_loop();

  yield();
}

extern mode_s_t state;
#include <malloc.h>
struct mallinfo mi;

void once_per_second_task_CPP(void)
{
  struct mode_s_aircraft *a;

#if 0
  int i = 0;
  a = state.aircrafts;

  while (a) {
    i++;
    a = a->next;
  }

  mi = mallinfo();
  if (!has_portapack) {
    SerialOutput.print(millis() / 1000); SerialOutput.write(' ');
    SerialOutput.print(i); SerialOutput.write(' ');
    SerialOutput.println(mi.fordblks);
  }
#endif

  for (a = state.aircrafts; a; a = a->next) {
    if (a->even_cprtime && a->odd_cprtime &&
        abs((long) (a->even_cprtime - a->odd_cprtime)) <= MODE_S_INTERACTIVE_TTL * 1000 ) {
      if (es1090_decode(a, &ThisAircraft, &fo)) {
        memset(fo.raw, 0, sizeof(fo.raw));
        Traffic_Update(&fo);
        Traffic_Add(&fo);
      }
    }
  }

  NMEA_Export();
  GDL90_Export();

  /* D1090 data comes directly out of MODE-S low-level frames decoder */

  ClearExpired();
}

void shutdown(int reason)
{

}

#endif /* HACKRF_ONE */
