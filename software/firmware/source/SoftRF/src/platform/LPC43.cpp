/*
 * Platform_LPC43.cpp
 * Copyright (C) 2021-2022 Linar Yusupov
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
  .imu      = IMU_NONE
};

const uint16_t LPC43_Vendor_Id = 0x1d50; /* OpenMoko, Inc. */
const uint16_t LPC43_Device_Id = 0x6089; /* HackRF One */
const char *LPC43_Device_Manufacturer = SOFTRF_IDENT;
const char *LPC43_Device_Model = "ES Edition";
const uint16_t LPC43_Device_Version = SOFTRF_USB_FW_VERSION;

#if defined(USE_PORTAPACK)
extern "C" const void* portapack(void);
#endif /* USE_PORTAPACK */

#if 0
usb_data_t usb_data_type = USB_DATA_GDL90;
#endif

static bool wdt_is_active = false;

#if defined(USE_OLED)
const char *Protocol_ID[] = {
  [RF_PROTOCOL_LEGACY]    = "LEG",
  [RF_PROTOCOL_OGNTP]     = "OGN",
  [RF_PROTOCOL_P3I]       = "P3I",
  [RF_PROTOCOL_ADSB_1090] = "ADS",
  [RF_PROTOCOL_ADSB_UAT]  = "UAT",
  [RF_PROTOCOL_FANET]     = "FAN"
};
#endif /* USE_OLED */

void LPC43_setup(void)
{
  SerialOutput.begin(SERIAL_OUT_BR, SERIAL_OUT_BITS);

#if 0
  SerialOutput.println();
  SerialOutput.print(F(SOFTRF_IDENT "-"));
  SerialOutput.print(SoC->name);
  SerialOutput.print(F(" FW.REV: " SOFTRF_FIRMWARE_VERSION " DEV.ID: "));
  SerialOutput.println(String(SoC->getChipId(), HEX));
  SerialOutput.println(F("Copyright (C) 2015-2022 Linar Yusupov. All rights reserved."));
  SerialOutput.println();
#endif
}

static void LPC43_post_init()
{
  Serial.println();
  Serial.println(F("SoftRF ES Edition Power-on Self Test"));
  Serial.println();
  Serial.flush();

  Serial.print(F("GNSS    : "));
  Serial.println(hw_info.gnss    != GNSS_MODULE_NONE  ? F("PASS") : F("N/A"));
  Serial.print(F("BARO    : "));
  Serial.println(hw_info.baro    != BARO_MODULE_NONE  ? F("PASS") : F("N/A"));
  Serial.print(F("DISPLAY : "));
  Serial.println(hw_info.display != DISPLAY_NONE      ? F("PASS") : F("N/A"));

  Serial.println();
  Serial.println(F("Power-on Self Test is complete."));
  Serial.println();
  Serial.flush();

  Serial.println(F("Data output device(s):"));

  Serial.print(F("NMEA   - "));
  switch (settings->nmea_out)
  {
    case NMEA_UART       :  Serial.println(F("UART"));    break;
    case NMEA_USB        :  Serial.println(F("USB CDC")); break;
    case NMEA_OFF        :
    default              :  Serial.println(F("NULL"));    break;
  }

  Serial.print(F("GDL90  - "));
  switch (settings->gdl90)
  {
    case GDL90_UART      :  Serial.println(F("UART"));    break;
    case GDL90_USB       :  Serial.println(F("USB CDC")); break;
    case GDL90_OFF       :
    default              :  Serial.println(F("NULL"));    break;
  }

  Serial.print(F("D1090  - "));
  switch (settings->d1090)
  {
    case D1090_UART      :  Serial.println(F("UART"));    break;
    case D1090_USB       :  Serial.println(F("USB CDC")); break;
    case D1090_OFF       :
    default              :  Serial.println(F("NULL"));    break;
  }

  Serial.println();
  Serial.flush();

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
    if (settings->mode != SOFTRF_MODE_NORMAL
#if !defined(EXCLUDE_TEST_MODE)
        &&
        settings->mode != SOFTRF_MODE_TXRX_TEST
#endif /* EXCLUDE_TEST_MODE */
        ) {
      settings->mode = SOFTRF_MODE_NORMAL;
    }

    if (settings->nmea_out == NMEA_BLUETOOTH ||
        settings->nmea_out == NMEA_UDP       ||
        settings->nmea_out == NMEA_TCP ) {
      settings->nmea_out = NMEA_USB;
    }
    if (settings->gdl90 == GDL90_BLUETOOTH  ||
        settings->gdl90 == GDL90_UDP) {
      settings->gdl90 = GDL90_USB;
    }
    if (settings->d1090 == D1090_BLUETOOTH  ||
        settings->d1090 == D1090_UDP) {
      settings->d1090 = D1090_USB;
    }
    settings->rf_protocol = RF_PROTOCOL_ADSB_1090;
    settings->txpower     = RF_TX_POWER_OFF;
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

#if defined(USE_PORTAPACK)
  rval = portapack() ? DISPLAY_TFT_PORTAPACK : rval;
#endif /* USE_PORTAPACK */

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

#if 0
static usb_data_t prev_usb_data_type = USB_DATA_D1090;
#endif

static void On_Button_Click()
{
#if defined(USE_OLED)
  OLED_Next_Page();
#endif

#if 0
  led_toggle(LED3);

  if (usb_data_type != USB_DATA_D1090) {
    prev_usb_data_type = usb_data_type;
    usb_data_type = USB_DATA_D1090;

    if (prev_usb_data_type == USB_DATA_NMEA) {
      settings->nmea_out = NMEA_OFF;
    } else if (prev_usb_data_type == USB_DATA_GDL90) {
      settings->gdl90    = GDL90_OFF;
    }

    settings->d1090 = D1090_USB;

  } else {

    usb_data_type = prev_usb_data_type;
    prev_usb_data_type = USB_DATA_D1090;

    settings->d1090 = D1090_OFF;

    if (usb_data_type == USB_DATA_NMEA) {
      settings->nmea_out = NMEA_USB;
    } else if (usb_data_type == USB_DATA_GDL90) {
      settings->gdl90    = GDL90_USB;
    }
  }
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

static void LPC43_USB_setup()
{
  TinyUSB_Device_Init(0);

  USBDevice.setID(LPC43_Vendor_Id, LPC43_Device_Id);
  USBDevice.setManufacturerDescriptor(LPC43_Device_Manufacturer);
  USBDevice.setProductDescriptor(LPC43_Device_Model);
  USBDevice.setDeviceVersion(LPC43_Device_Version);

  if (USBSerial && USBSerial != Serial) {
    USBSerial.begin(SERIAL_OUT_BR);
  }
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
  if (USBSerial && USBSerial != Serial) {
    USBSerial.end();
  }
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
  "LPC43",
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
    if (Serial) { delay(1000); break; } else if (SoC->USB_ops) SoC->USB_ops->loop();
  }

  Serial.println();
  Serial.print(F(SOFTRF_IDENT "-"));
  Serial.print(SoC->name);
  Serial.print(F(" FW.REV: " SOFTRF_FIRMWARE_VERSION " DEV.ID: "));
  Serial.println(String(SoC->getChipId(), HEX));
  Serial.println(F("Copyright (C) 2015-2022 Linar Yusupov. All rights reserved."));
  Serial.flush();

  Serial.println();
  Serial.print(F("Reset reason: ")); Serial.print(SoC->getResetReason());
  Serial.print(F(" ("));
  Serial.print(system_reset_reason(), HEX);
  Serial.println(')');
  Serial.print(F("Free heap size: ")); Serial.println(SoC->getFreeHeap());
  Serial.flush();

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

#if 0
    if (hw_info.gnss != GNSS_MODULE_NONE) {
      settings->nmea_out = NMEA_USB;
      settings->gdl90    = GDL90_OFF;

      usb_data_type = USB_DATA_NMEA;
    }
#endif

    hw_info.baro = Baro_setup();
  }

  Traffic_setup();
  NMEA_setup();
  Serial.flush();

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

#if defined(TAKE_CARE_OF_MILLIS_ROLLOVER)
  /* restart the device when uptime is more than 47 days */
  if (millis() > (47 * 24 * 3600 * 1000UL)) {
    SoC->reset();
  }
#endif /* TAKE_CARE_OF_MILLIS_ROLLOVER */

  yield();
}

extern mode_s_t state;
#include <malloc.h>
struct mallinfo mi;

void once_per_second_task_CPP(void)
{
  struct mode_s_aircraft *a;
  int i = 0;

#if 0
  a = state.aircrafts;

  while (a) {
    i++;
    a = a->next;
  }

  mi = mallinfo();
  SerialOutput.print(millis() / 1000); SerialOutput.write(' ');
  SerialOutput.print(i); SerialOutput.write(' ');
  SerialOutput.println(mi.fordblks);
#endif

  for (a = state.aircrafts; a; a = a->next) {
    if (a->even_cprtime && a->odd_cprtime &&
        abs((long) (a->even_cprtime - a->odd_cprtime)) <= MODE_S_INTERACTIVE_TTL * 1000 ) {
      if (es1090_decode(a, &ThisAircraft, &fo)) {
        memset(fo.raw, 0, sizeof(fo.raw));

        Traffic_Update(&fo);

        for (i=0; i < MAX_TRACKING_OBJECTS; i++) {
          if (Container[i].addr == fo.addr) {
            uint8_t alert_bak = Container[i].alert;
            Container[i] = fo;
            Container[i].alert = alert_bak;
            break;
          }
        }
        if (i < MAX_TRACKING_OBJECTS) continue;

        int max_dist_ndx = 0;
        int min_level_ndx = 0;

        for (i=0; i < MAX_TRACKING_OBJECTS; i++) {
          if (now() - Container[i].timestamp > ENTRY_EXPIRATION_TIME) {
            Container[i] = fo;
            break;
          }
#if !defined(EXCLUDE_TRAFFIC_FILTER_EXTENSION)
          if (Container[i].distance > Container[max_dist_ndx].distance) {
            max_dist_ndx = i;
          }
          if (Container[i].alarm_level < Container[min_level_ndx].alarm_level) {
            min_level_ndx = i;
          }
#endif /* EXCLUDE_TRAFFIC_FILTER_EXTENSION */
        }
        if (i < MAX_TRACKING_OBJECTS) continue;

#if !defined(EXCLUDE_TRAFFIC_FILTER_EXTENSION)
        if (fo.alarm_level > Container[min_level_ndx].alarm_level) {
          Container[min_level_ndx] = fo;
          continue;
        }

        if (fo.distance    <  Container[max_dist_ndx].distance &&
            fo.alarm_level >= Container[max_dist_ndx].alarm_level) {
          Container[max_dist_ndx] = fo;
          continue;
        }
#endif /* EXCLUDE_TRAFFIC_FILTER_EXTENSION */
      }
    }
  }

  NMEA_Export();
  GDL90_Export();

  /* D0190 data comes directly out of MODE-S low-level frames decoder */

  ClearExpired();
}

void shutdown(int reason)
{

}

#endif /* HACKRF_ONE */
