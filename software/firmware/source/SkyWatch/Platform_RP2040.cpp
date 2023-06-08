/*
 * Platform_RP2040.cpp
 * Copyright (C) 2023 Linar Yusupov
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
#if defined(ARDUINO_ARCH_RP2040)

#include "SoCHelper.h"
#include "EEPROMHelper.h"
#if !defined(EXCLUDE_WIFI)
#include "WiFiHelper.h"
#endif /* EXCLUDE_WIFI */
#if !defined(EXCLUDE_BLUETOOTH)
#include "BluetoothHelper.h"
#endif /* EXCLUDE_BLUETOOTH */

#include "SkyWatch.h"

#include <hardware/watchdog.h>
#include <Adafruit_SleepyDog.h>

#if !defined(ARDUINO_ARCH_MBED)
#include "pico/unique_id.h"

#define PICO_ON_DEVICE 1
extern "C" {
#include "pico/binary_info.h"
}
#else
extern "C"
{
#include "hardware/flash.h"
}
#endif /* ARDUINO_ARCH_MBED */

#include <pico_sleep.h>

#if defined(USE_TINYUSB)
#if defined(USE_USB_HOST)
#include "pio_usb.h"
#endif /* USE_USB_HOST */
#include "Adafruit_TinyUSB.h"
#endif /* USE_TINYUSB */

#define SOFTRF_DESC "Multifunctional, compatible DIY general aviation proximity awareness system"
#define SOFTRF_URL  "https://github.com/lyusupov/SoftRF"

#if !defined(ARDUINO_ARCH_MBED)
bi_decl(bi_program_name(WEBTOP_IDENT));
bi_decl(bi_program_description(SOFTRF_DESC));
bi_decl(bi_program_version_string(SKYWATCH_FIRMWARE_VERSION));
bi_decl(bi_program_build_date_string(__DATE__));
bi_decl(bi_program_url(SOFTRF_URL));
extern char __flash_binary_end;
bi_decl(bi_binary_end((intptr_t)&__flash_binary_end));
#endif /* ARDUINO_ARCH_MBED */

#if defined(EXCLUDE_WIFI)
char UDPpacketBuffer[4]; // Dummy definition to satisfy build sequence
#else
WebServer server ( 80 );
#endif /* EXCLUDE_WIFI */

static uint32_t bootCount __attribute__ ((section (".noinit")));
static bool wdt_is_active              = false;

static RP2040_board_id RP2040_board    = RP2040_RPIPICO_W; /* default */
const char *RP2040_Device_Manufacturer = SOFTRF_IDENT;
const char *RP2040_Device_Model        = WEBTOP_IDENT " Pico";
const uint16_t RP2040_Device_Version   = SKYWATCH_USB_FW_VERSION;

static volatile bool core1_booting     = true;
static volatile bool core0_booting     = true;

#define UniqueIDsize                   2

static union {
#if !defined(ARDUINO_ARCH_MBED)
  pico_unique_board_id_t RP2040_unique_flash_id;
#endif /* ARDUINO_ARCH_MBED */
  uint32_t RP2040_chip_id[UniqueIDsize];
};

static void RP2040_setup()
{
#if !defined(ARDUINO_ARCH_MBED)
  pico_get_unique_board_id(&RP2040_unique_flash_id);
#else
  flash_get_unique_id((uint8_t *)&RP2040_chip_id);
#endif /* ARDUINO_ARCH_MBED */

#if defined(USE_TINYUSB)
  USBDevice.setManufacturerDescriptor(RP2040_Device_Manufacturer);
  USBDevice.setProductDescriptor(RP2040_Device_Model);
  USBDevice.setDeviceVersion(RP2040_Device_Version);
#endif /* USE_TINYUSB */

#if !defined(ARDUINO_ARCH_MBED)
  SerialInput.setRX(SOC_GPIO_PIN_CONS_RX);
  SerialInput.setTX(SOC_GPIO_PIN_CONS_TX);
  SerialInput.setFIFOSize(255);
  SerialInput.begin(SERIAL_OUT_BR, SERIAL_OUT_BITS);

  Serial_GNSS_In.setRX(SOC_GPIO_PIN_GNSS_RX);
  Serial_GNSS_In.setTX(SOC_GPIO_PIN_GNSS_TX);
  Serial_GNSS_In.setFIFOSize(255);
#endif /* ARDUINO_ARCH_MBED */

#if defined(ARDUINO_RASPBERRY_PI_PICO)
  RP2040_board = RP2040_RPIPICO;
#elif defined(ARDUINO_RASPBERRY_PI_PICO_W)
  RP2040_board = rp2040.isPicoW() ? RP2040_RPIPICO_W : RP2040_RPIPICO;
#endif /* ARDUINO_RASPBERRY_PI_PICO */

  RP2040_board = (SoC->getChipId() == 0xcf516424) ?
                  RP2040_WEACT : RP2040_board;

  hw_info.model    = SOFTRF_MODEL_WEBTOP_USB;
  hw_info.revision = HW_REV_PICO_W;

  USBSerial.begin(SERIAL_OUT_BR);

  for (int i=0; i < 20; i++) {if (USBSerial) break; else delay(100);}

#if defined(USE_USB_HOST)
  while (core1_booting) {}
#endif /* USE_USB_HOST */
}

static void RP2040_post_init()
{
  core0_booting = false;
}

static void RP2040_loop()
{
  if (wdt_is_active) {
#if !defined(ARDUINO_ARCH_MBED)
    Watchdog.reset();
#endif /* ARDUINO_ARCH_MBED */
  }
}

static void RP2040_fini()
{
#if 0 /* TBD */
  // back from dormant state
  rosc_enable();
  clocks_init();

  rp2040.restart();
#endif
}

static void RP2040_reset()
{
  rp2040.restart();
}

static void RP2040_sleep_ms(int ms)
{
  sleep_ms(ms);
}

static uint32_t RP2040_getChipId()
{
  return __builtin_bswap32(RP2040_chip_id[UniqueIDsize - 1]);
}

static uint32_t RP2040_getFreeHeap()
{
  return rp2040.getFreeHeap();
}

static bool RP2040_EEPROM_begin(size_t size)
{
  EEPROM.begin(size);
  return true;
}

static void RP2040_WiFi_set_param(int ndx, int value)
{
#if !defined(EXCLUDE_WIFI)
  switch (ndx)
  {
  case WIFI_PARAM_TX_POWER:
    switch (value)
    {
    case WIFI_TX_POWER_MAX:
      WiFi.noLowPowerMode();
      break;
    case WIFI_TX_POWER_MIN:
      WiFi.aggressiveLowPowerMode();
      break;
    case WIFI_TX_POWER_MED:
    default:
      WiFi.defaultLowPowerMode();
      break;
    }
    break;
  case WIFI_PARAM_DHCP_LEASE_TIME:
    if (WiFi.getMode() == WIFI_AP) {
      /* RP2040 Wi-Fi default lease time is 24 hours */
    }
    break;
  default:
    break;
  }
#endif /* EXCLUDE_WIFI */
}

static bool RP2040_WiFi_hostname(String aHostname)
{
  bool rval = false;
#if !defined(EXCLUDE_WIFI)
  if (RP2040_board == RP2040_RPIPICO_W) {
    WiFi.hostname(aHostname.c_str());
    rval = true;
  }
#endif /* EXCLUDE_WIFI */
  return rval;
}

static void RP2040_WiFiUDP_stopAll()
{
#if !defined(EXCLUDE_WIFI)
  WiFiUDP::stopAll();
#endif /* EXCLUDE_WIFI */
}

#if !defined(EXCLUDE_WIFI)
#include <dhcpserver/dhcpserver.h>
#endif /* EXCLUDE_WIFI */

union rp2040_ip {
  uint32_t addr;
  uint8_t bytes[4];
};

static void RP2040_WiFi_transmit_UDP(int port, byte *buf, size_t size)
{
#if !defined(EXCLUDE_WIFI)
  union rp2040_ip ipv4;
  IPAddress ClientIP;
  ipv4.addr       = (uint32_t) WiFi.localIP();
  WiFiMode_t mode = WiFi.getMode();

  switch (mode)
  {
  case WIFI_STA:
    ClientIP = IPAddress(ipv4.addr | ~((uint32_t) WiFi.subnetMask()));

    Uni_Udp.beginPacket(ClientIP, port);
    Uni_Udp.write(buf, size);
    Uni_Udp.endPacket();

    break;
  case WIFI_AP:
    if (WiFi.softAPgetStationNum() > 0) {
      for (int i=0; i<4; i++) {
        ClientIP = IPAddress(ipv4.bytes[0], ipv4.bytes[1], ipv4.bytes[2],
                             DHCPS_BASE_IP + i);
        Uni_Udp.beginPacket(ClientIP, port);
        Uni_Udp.write(buf, size);
        Uni_Udp.endPacket();
      }
    }
    break;
  case WIFI_OFF:
  default:
    break;
  }
#endif /* EXCLUDE_WIFI */
}

static size_t RP2040_WiFi_Receive_UDP(uint8_t *buf, size_t max_size)
{
  return 0; // WiFi_Receive_UDP(buf, max_size);
}

static int RP2040_WiFi_clients_count()
{
#if !defined(EXCLUDE_WIFI)
  struct station_info *stat_info;
  int clients = 0;
  WiFiMode_t mode = WiFi.getMode();

  switch (mode)
  {
  case WIFI_AP:
    return WiFi.softAPgetStationNum();
  case WIFI_STA:
  default:
    return -1; /* error */
  }
#else
  return -1;
#endif /* EXCLUDE_WIFI */
}

static void RP2040_swSer_begin(unsigned long baud)
{
  SerialInput.begin(baud);
}

static void RP2040_swSer_enableRx(boolean arg)
{
  /* NONE */
}

static uint32_t RP2040_maxSketchSpace()
{
  return 1048576; /* TBD */
}

static void RP2040_Battery_setup()
{
#if SOC_GPIO_PIN_BATTERY != SOC_UNUSED_PIN
  analogReadResolution(12);
  analogRead(SOC_GPIO_PIN_BATTERY);
#endif
}

#include "hardware/gpio.h"
#include "hardware/adc.h"

static float RP2040_Battery_voltage()
{
  uint16_t mV = 0;

#if SOC_GPIO_PIN_BATTERY != SOC_UNUSED_PIN
  enum gpio_function pin25_func;
  enum gpio_function pin29_func;
  uint pin25_dir;
  uint pin29_dir;

  if (RP2040_board == RP2040_RPIPICO_W) {
    pin29_dir  = gpio_get_dir(SOC_GPIO_PIN_BATTERY);
    pin29_func = gpio_get_function(SOC_GPIO_PIN_BATTERY);
    adc_gpio_init(SOC_GPIO_PIN_BATTERY);

    pin25_dir  = gpio_get_dir(SOC_GPIO_PIN_CYW43_EN);
    pin25_func = gpio_get_function(SOC_GPIO_PIN_CYW43_EN);
    pinMode(SOC_GPIO_PIN_CYW43_EN, OUTPUT);
    digitalWrite(SOC_GPIO_PIN_CYW43_EN, HIGH);
  }

  mV = (analogRead(SOC_GPIO_PIN_BATTERY) * 3300UL) >> 12;

  if (RP2040_board == RP2040_RPIPICO_W) {
    gpio_set_function(SOC_GPIO_PIN_CYW43_EN, pin25_func);
    gpio_set_dir(SOC_GPIO_PIN_CYW43_EN, pin25_dir);
    gpio_set_function(SOC_GPIO_PIN_BATTERY,  pin29_func);
    gpio_set_dir(SOC_GPIO_PIN_BATTERY,  pin29_dir);
  }
#endif
  return (mV * SOC_ADC_VOLTAGE_DIV / 1000.0);
}

static bool RP2040_DB_init()
{
  return false;
}

static bool RP2040_DB_query(uint8_t type, uint32_t id, char *buf, size_t size)
{
  return false;
}

static void RP2040_DB_fini()
{

}

static void RP2040_TTS(char *message)
{

}

static void RP2040_Button_setup()
{

}

static void RP2040_Button_loop()
{

}

static void RP2040_Button_fini()
{

}

static bool RP2040_Baro_setup()
{
  return false;
}

static void RP2040_WDT_setup()
{
#if !defined(ARDUINO_ARCH_MBED)
  Watchdog.enable(4000);
#endif /* ARDUINO_ARCH_MBED */
  wdt_is_active = true;
}

static void RP2040_WDT_fini()
{
  if (wdt_is_active) {
#if !defined(ARDUINO_ARCH_MBED)
    hw_clear_bits(&watchdog_hw->ctrl, WATCHDOG_CTRL_ENABLE_BITS);
#endif /* ARDUINO_ARCH_MBED */
    wdt_is_active = false;
  }
}

static void RP2040_Service_Mode(boolean arg)
{

}

#include <api/RingBuffer.h>

#define USB_TX_FIFO_SIZE (256)
#define USB_RX_FIFO_SIZE (MAX_TRACKING_OBJECTS * 65 + 75 + 75 + 42 + 20)

#if !defined(USBD_CDC_IN_OUT_MAX_SIZE)
#define USBD_CDC_IN_OUT_MAX_SIZE (64)
#endif /* USBD_CDC_IN_OUT_MAX_SIZE */

RingBufferN<USB_TX_FIFO_SIZE> USB_TX_FIFO = RingBufferN<USB_TX_FIFO_SIZE>();
RingBufferN<USB_RX_FIFO_SIZE> USB_RX_FIFO = RingBufferN<USB_RX_FIFO_SIZE>();

#if defined(USE_USB_HOST)
/*********************************************************************
 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 Copyright (c) 2019 Ha Thach for Adafruit Industries
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#define LANGUAGE_ID 0x0409  // English

// USB Host object
Adafruit_USBH_Host USBHost;

// CDC Host object
Adafruit_USBH_CDC  SerialHost;

// holding device descriptor
tusb_desc_device_t desc_device;

void setup1() {

  // Check for CPU frequency, must be multiple of 120Mhz for bit-banging USB
  uint32_t cpu_hz = clock_get_hz(clk_sys);
  if ( cpu_hz != 120000000UL && cpu_hz != 240000000UL ) {
    while ( !Serial ) delay(10);   // wait for native usb
    Serial.printf("Error: CPU Clock = %u, PIO USB require CPU clock must be multiple of 120 Mhz\r\n", cpu_hz);
    Serial.printf("Change your CPU Clock to either 120 or 240 Mhz in Menu->CPU Speed \r\n", cpu_hz);
    while(1) delay(1);
  }

  pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
  pio_cfg.pin_dp     = SOC_GPIO_PIN_USBH_DP;
  pio_cfg.sm_tx      = 3;
  pio_cfg.sm_rx      = 2;
  pio_cfg.sm_eop     = 3;
  pio_cfg.pio_rx_num = 0;
  pio_cfg.pio_tx_num = 1;
  pio_cfg.tx_ch      = 9;
  USBHost.configure_pio_usb(1, &pio_cfg);

  // run host stack on controller (rhport) 1
  // Note: For rp2040 pico-pio-usb, calling USBHost.begin() on core1 will have most of the
  // host bit-banging processing works done in core1 to free up core0 for other works
  USBHost.begin(1);

  SerialHost.begin(SERIAL_OUT_BR);

  core1_booting = false;
  while (core0_booting) { }
}

// core1's loop
void loop1()
{
  USBHost.task();

  // periodically flush SerialHost if connected
  if ( SerialHost && SerialHost.connected() ) {
    SerialHost.flush();
  }
}

//--------------------------------------------------------------------+
// String Descriptor Helper
//--------------------------------------------------------------------+

static void _convert_utf16le_to_utf8(const uint16_t *utf16, size_t utf16_len, uint8_t *utf8, size_t utf8_len) {
  // TODO: Check for runover.
  (void)utf8_len;
  // Get the UTF-16 length out of the data itself.

  for (size_t i = 0; i < utf16_len; i++) {
    uint16_t chr = utf16[i];
    if (chr < 0x80) {
      *utf8++ = chr & 0xff;
    } else if (chr < 0x800) {
      *utf8++ = (uint8_t)(0xC0 | (chr >> 6 & 0x1F));
      *utf8++ = (uint8_t)(0x80 | (chr >> 0 & 0x3F));
    } else {
      // TODO: Verify surrogate.
      *utf8++ = (uint8_t)(0xE0 | (chr >> 12 & 0x0F));
      *utf8++ = (uint8_t)(0x80 | (chr >> 6 & 0x3F));
      *utf8++ = (uint8_t)(0x80 | (chr >> 0 & 0x3F));
    }
    // TODO: Handle UTF-16 code points that take two entries.
  }
}

// Count how many bytes a utf-16-le encoded string will take in utf-8.
static int _count_utf8_bytes(const uint16_t *buf, size_t len) {
  size_t total_bytes = 0;
  for (size_t i = 0; i < len; i++) {
    uint16_t chr = buf[i];
    if (chr < 0x80) {
      total_bytes += 1;
    } else if (chr < 0x800) {
      total_bytes += 2;
    } else {
      total_bytes += 3;
    }
    // TODO: Handle UTF-16 code points that take two entries.
  }
  return total_bytes;
}

static void print_utf16(uint16_t *temp_buf, size_t buf_len) {
  size_t utf16_len = ((temp_buf[0] & 0xff) - 2) / sizeof(uint16_t);
  size_t utf8_len = _count_utf8_bytes(temp_buf + 1, utf16_len);

  _convert_utf16le_to_utf8(temp_buf + 1, utf16_len, (uint8_t *) temp_buf, sizeof(uint16_t) * buf_len);
  ((uint8_t*) temp_buf)[utf8_len] = '\0';

  Serial.printf((char*)temp_buf);
}

void print_device_descriptor(tuh_xfer_t* xfer)
{
  if ( XFER_RESULT_SUCCESS != xfer->result )
  {
    Serial.printf("Failed to get device descriptor\r\n");
    return;
  }

  uint8_t const daddr = xfer->daddr;

  Serial.printf("Device %u: ID %04x:%04x\r\n", daddr, desc_device.idVendor, desc_device.idProduct);
  Serial.printf("Device Descriptor:\r\n");
  Serial.printf("  bLength             %u\r\n"     , desc_device.bLength);
  Serial.printf("  bDescriptorType     %u\r\n"     , desc_device.bDescriptorType);
  Serial.printf("  bcdUSB              %04x\r\n"   , desc_device.bcdUSB);
  Serial.printf("  bDeviceClass        %u\r\n"     , desc_device.bDeviceClass);
  Serial.printf("  bDeviceSubClass     %u\r\n"     , desc_device.bDeviceSubClass);
  Serial.printf("  bDeviceProtocol     %u\r\n"     , desc_device.bDeviceProtocol);
  Serial.printf("  bMaxPacketSize0     %u\r\n"     , desc_device.bMaxPacketSize0);
  Serial.printf("  idVendor            0x%04x\r\n" , desc_device.idVendor);
  Serial.printf("  idProduct           0x%04x\r\n" , desc_device.idProduct);
  Serial.printf("  bcdDevice           %04x\r\n"   , desc_device.bcdDevice);

  // Get String descriptor using Sync API
  uint16_t temp_buf[128];

  Serial.printf("  iManufacturer       %u     "     , desc_device.iManufacturer);
  if (XFER_RESULT_SUCCESS == tuh_descriptor_get_manufacturer_string_sync(daddr, LANGUAGE_ID, temp_buf, sizeof(temp_buf)) )
  {
    print_utf16(temp_buf, TU_ARRAY_SIZE(temp_buf));
  }
  Serial.printf("\r\n");

  Serial.printf("  iProduct            %u     "     , desc_device.iProduct);
  if (XFER_RESULT_SUCCESS == tuh_descriptor_get_product_string_sync(daddr, LANGUAGE_ID, temp_buf, sizeof(temp_buf)))
  {
    print_utf16(temp_buf, TU_ARRAY_SIZE(temp_buf));
  }
  Serial.printf("\r\n");

  Serial.printf("  iSerialNumber       %u     "     , desc_device.iSerialNumber);
  if (XFER_RESULT_SUCCESS == tuh_descriptor_get_serial_string_sync(daddr, LANGUAGE_ID, temp_buf, sizeof(temp_buf)))
  {
    print_utf16(temp_buf, TU_ARRAY_SIZE(temp_buf));
  }
  Serial.printf("\r\n");

  Serial.printf("  bNumConfigurations  %u\r\n"     , desc_device.bNumConfigurations);
}

//--------------------------------------------------------------------+
// TinyUSB Host callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted (configured)
void tuh_mount_cb (uint8_t daddr)
{
  Serial.printf("Device attached, address = %d\r\n", daddr);

  // Get Device Descriptor
  tuh_descriptor_get_device(daddr, &desc_device, 18, print_device_descriptor, 0);
}

/// Invoked when device is unmounted (bus reset/unplugged)
void tuh_umount_cb(uint8_t daddr)
{
  Serial.printf("Device removed, address = %d\r\n", daddr);
}

extern "C" {

// Invoked when a device with CDC interface is mounted
// idx is index of cdc interface in the internal pool.
void tuh_cdc_mount_cb(uint8_t idx) {

  // bind SerialHost object to this interface index
  uint32_t SerialBaud;

  switch (settings->m.baudrate)
  {
  case B4800:
    SerialBaud = 4800;
    break;
  case B9600:
    SerialBaud = 9600;
    break;
  case B19200:
    SerialBaud = 19200;
    break;
  case B57600:
    SerialBaud = 57600;
    break;
  case B115200:
    SerialBaud = 115200;
    break;
  case B2000000:
    SerialBaud = 2000000;
    break;
  case B38400:
  default:
    SerialBaud = 38400;
    break;
  }

  SerialHost.mount(idx);

  SerialHost.setBaudrate(SerialBaud);
  SerialHost.setDtrRts(true, false);

  Serial.println("SerialHost is connected to a new CDC device");
}

// Invoked when a device with CDC interface is unmounted
void tuh_cdc_umount_cb(uint8_t idx) {
  SerialHost.umount(idx);
  Serial.println("SerialHost is disconnected");
}

} // extern "C"

#endif /* USE_USB_HOST */

static void RP2040_USB_setup()
{
#if defined(USE_USB_HOST)
  /* TBD */
#elif !defined(ARDUINO_ARCH_MBED)
  if (USBSerial && USBSerial != Serial) {
    USBSerial.begin(SERIAL_OUT_BR);
  }
#endif /* ARDUINO_ARCH_MBED */
}

static void RP2040_USB_loop()
{
#if defined(USE_USB_HOST)
  uint8_t buf[USBD_CDC_IN_OUT_MAX_SIZE];
  size_t size;

  while (SerialHost             &&
         SerialHost.connected() &&
         (size = SerialHost.availableForWrite()) > 0) {
    size_t avail = USB_TX_FIFO.available();

    if (avail == 0) {
      break;
    }

    if (size > avail) {
      size = avail;
    }

    if (size > sizeof(buf)) {
      size = sizeof(buf);
    }

    for (size_t i=0; i < size; i++) {
      buf[i] = USB_TX_FIFO.read_char();
    }

    if (SerialHost && SerialHost.connected()) {
      SerialHost.write(buf, size);
    }
  }

  if ( SerialHost             &&
       SerialHost.connected() &&
       (size = USB_RX_FIFO.availableForStore()) > 0 ) {
    if (size > sizeof(buf)) {
      size = sizeof(buf);
    }
    size_t avail = SerialHost.available();
    if (size > avail) {
      size = avail;
    }

    size_t count = SerialHost.read(buf, size);
//    Serial.write(buf, count);
    for (size_t i=0; i < count; i++) {
      if (!USB_RX_FIFO.isFull()) {
        USB_RX_FIFO.store_char(buf[i]);
      }
    }
  }
#elif !defined(USE_TINYUSB)
  uint8_t buf[USBD_CDC_IN_OUT_MAX_SIZE];
  size_t size;

  while (USBSerial && (size = USBSerial.availableForWrite()) > 0) {
    size_t avail = USB_TX_FIFO.available();

    if (avail == 0) {
      break;
    }

    if (size > avail) {
      size = avail;
    }

    if (size > sizeof(buf)) {
      size = sizeof(buf);
    }

    for (size_t i=0; i < size; i++) {
      buf[i] = USB_TX_FIFO.read_char();
    }

    if (USBSerial) {
      USBSerial.write(buf, size);
    }
  }

  while (USBSerial && USBSerial.available() > 0) {
    if (!USB_RX_FIFO.isFull()) {
      USB_RX_FIFO.store_char(USBSerial.read());
    } else {
      break;
    }
  }
#endif /* USE_TINYUSB */
}

static void RP2040_USB_fini()
{
#if defined(USE_USB_HOST)
  /* TBD */
#elif !defined(ARDUINO_ARCH_MBED)
  if (USBSerial && USBSerial != Serial) {
    USBSerial.end();
  }
#endif /* ARDUINO_ARCH_MBED */
}

static int RP2040_USB_available()
{
  int rval = 0;

#if defined(USE_USB_HOST) || !defined(USE_TINYUSB)
  rval = USB_RX_FIFO.available();
#else
  if (USBSerial) {
    rval = USBSerial.available();
  }
#endif /* USE_TINYUSB */

  return rval;
}

static int RP2040_USB_read()
{
  int rval = -1;

#if defined(USE_USB_HOST) || !defined(USE_TINYUSB)
  rval = USB_RX_FIFO.read_char();
#else
  if (USBSerial) {
    rval = USBSerial.read();
  }
#endif /* USE_TINYUSB */

  return rval;
}

static size_t RP2040_USB_write(const uint8_t *buffer, size_t size)
{
#if defined(USE_USB_HOST) || !defined(USE_TINYUSB)
  size_t written;

  for (written=0; written < size; written++) {
    if (!USB_TX_FIFO.isFull()) {
      USB_TX_FIFO.store_char(buffer[written]);
    } else {
      break;
    }
  }
  return written;
#else
  size_t rval = size;

  if (USBSerial && (size < USBSerial.availableForWrite())) {
    rval = USBSerial.write(buffer, size);
  }

  return rval;
#endif /* USE_TINYUSB */
}

IODev_ops_t RP2040_USBSerial_ops = {
  "RP2040 USB ACM",
  RP2040_USB_setup,
  RP2040_USB_loop,
  RP2040_USB_fini,
  RP2040_USB_available,
  RP2040_USB_read,
  RP2040_USB_write
};

const SoC_ops_t RP2040_ops = {
  SOC_RP2040,
  "RP2040",
  RP2040_setup,
  RP2040_post_init,
  RP2040_loop,
  RP2040_fini,
  RP2040_reset,
  RP2040_sleep_ms,
  RP2040_getChipId,
  RP2040_getFreeHeap,
  RP2040_EEPROM_begin,
  RP2040_WiFi_set_param,
  RP2040_WiFi_hostname,
  RP2040_WiFiUDP_stopAll,
  RP2040_WiFi_transmit_UDP,
  RP2040_WiFi_Receive_UDP,
  RP2040_WiFi_clients_count,
  RP2040_swSer_begin,
  RP2040_swSer_enableRx,
  RP2040_maxSketchSpace,
  RP2040_Battery_setup,
  RP2040_Battery_voltage,
  RP2040_DB_init,
  RP2040_DB_query,
  RP2040_DB_fini,
  RP2040_TTS,
  RP2040_Button_setup,
  RP2040_Button_loop,
  RP2040_Button_fini,
  RP2040_Baro_setup,
  RP2040_WDT_setup,
  RP2040_WDT_fini,
  RP2040_Service_Mode,
#if !defined(EXCLUDE_BLUETOOTH)
  &CYW43_Bluetooth_ops,
#else
  NULL,
#endif /* EXCLUDE_BLUETOOTH */
  &RP2040_USBSerial_ops,
};

#endif /* ARDUINO_ARCH_RP2040 */
