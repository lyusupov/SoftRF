// ArduinoCompat/HardwareSerial.cpp
//
// Author: mikem@airspayce.com

#include <RadioHead.h>
#if (RH_PLATFORM == RH_PLATFORM_MONGOOSE_OS)
#include <mgos.h>
#include <HardwareSerial.h>

extern "C" {
    static inline void mgos_sys_config_set_rh_serial_baud(int v);
    static inline void mgos_sys_config_set_rh_serial_databits(int v);
    static inline void mgos_sys_config_set_rh_serial_parity(int v);
    static inline void mgos_sys_config_set_rh_serial_stopbits(int v);

    void mgos_uart_config_set_defaults(int uart_no, struct mgos_uart_config *cfg);
    bool mgos_uart_configure(int uart_no, const struct mgos_uart_config *cfg);
    void mgos_uart_set_rx_enabled(int uart_no, bool enabled);
    size_t mgos_uart_read_avail(int uart_no);
    size_t mgos_uart_read(int uart_no, void *buf, size_t len);
    size_t mgos_uart_write(int uart_no, const void *buf, size_t len);
};


// instantiate Serial objects
HardwareSerial Serial0(0);
HardwareSerial Serial1(1);
HardwareSerial Serial2(2);

/*
 * Serial ports
 *
 * ESP8266
 * The esp8266 device has two uarts (0 and 1), Uart 1 TX is typically used for debugging. No Uart 1 RX is available on the esp8266.
 *
 * Uart 0
 * RX  = GPIO3
 * TX  = GPIO1
 *
 * Uart 1
 * TX  = GPIO2
 *
 * ESP32
 * The esp32 device has three uarts (0,1 and 2). Uart 0 is typically used for debugging/loading code.
 *
 * Uart 0
 * RX  = GPIO3
 * TX  = GPIO1
 * CTS = GPIO19
 * RTS = GPIO22
 *
 * Uart 1
 * RX  = GPIO25
 * TX  = GPIO26
 * CTS = GPIO27
 * RTS = GPIO13
 *
 * Uart 2
 * RX  = GPIO16
 * TX  = GPIO17
 * CTS = GPIO14
 * RTS = GPIO15
 */

///////////////////////////////////////////////////////////////
// HardwareSerial
///////////////////////////////////////////////////////////////

HardwareSerial::HardwareSerial(int uartIndex)
{
    this->uartIndex=uartIndex;
}

/**
 * @brief Init the serial port.
 * @param baud If the baud rate is 0 or -ve then the persistent sorage value
 * is used. Starting at the value in mos.yml.
 */
void HardwareSerial::begin(int baud)
{
    struct mgos_uart_config ucfg;

    if( mgos_sys_config_get_rh_serial_baud() != baud ) {
        mgos_sys_config_set_rh_serial_baud(baud);
    }

    mgos_uart_config_set_defaults(this->uartIndex, &ucfg);
    ucfg.baud_rate = mgos_sys_config_get_rh_serial_baud();
    ucfg.num_data_bits = mgos_sys_config_get_rh_serial_databits();
    ucfg.parity = (mgos_uart_parity)mgos_sys_config_get_rh_serial_parity();
    ucfg.stop_bits = (mgos_uart_stop_bits)mgos_sys_config_get_rh_serial_stopbits();
    mgos_uart_configure(this->uartIndex, &ucfg);
    if( mgos_uart_configure(this->uartIndex, &ucfg) ) {
        mgos_uart_set_rx_enabled(this->uartIndex, true);
        mgos_uart_set_dispatcher(this->uartIndex, NULL, NULL);
    }

#ifdef NO_ESP32_RXD_PULLUP
    if( this->uartIndex == 0 ) {
        mgos_gpio_setup_input(3, MGOS_GPIO_PULL_NONE);
    }
    else if( this->uartIndex == 1 ) {
        mgos_gpio_setup_input(25, MGOS_GPIO_PULL_NONE);
    }
    else if( this->uartIndex == 2 ) {
        mgos_gpio_setup_input(16, MGOS_GPIO_PULL_NONE);
    }
#endif
}

void HardwareSerial::end()
{
    mgos_uart_set_rx_enabled(this->uartIndex, false);
}

int HardwareSerial::available(void)
{
    size_t  reqRxByteCount=1;
    //We have to read the byte because Mongoose OS requires a return
    //to the RTOS in order to update the value read by mgos_uart_read_avail()
    rxByteCountAvail = mgos_uart_read(this->uartIndex, &rxByte, reqRxByteCount);
    return rxByteCountAvail;
}

int HardwareSerial::read(void)
{
    return rxByte;
}

size_t HardwareSerial::write(uint8_t ch)
{
    size_t wr_byte_count = 0;

    wr_byte_count = mgos_uart_write(this->uartIndex, &ch, 1);

    return wr_byte_count;
}

size_t HardwareSerial::print(char ch)
{
  printf("%c", ch);
  return 0;
}

size_t HardwareSerial::println(char ch)
{
  printf("%c\n", ch);
  return 0;
}

size_t HardwareSerial::print(unsigned char ch, int base)
{
  if( base == DEC ) {
      printf("%d", ch);
  }
  else if( base == HEX ) {
      printf("%02x", ch);
  }
  else if( base == OCT ) {
      printf("%o", ch);
  }
  //TODO Add binary print
  return 0;
}

size_t HardwareSerial::println(unsigned char ch, int base)
{
  print((unsigned int)ch, base);
  printf("\n");
  return 0;
}

size_t HardwareSerial::println(const char* s)
{
    if( s ) {
        printf("%s\n",s);
    }
    return 0;
}

size_t HardwareSerial::print(const char* s)
{
    if( s) {
        printf(s);
    }
    return 0;
}

#endif
