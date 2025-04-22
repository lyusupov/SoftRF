// ArduinoCompat/HardwareSerial.h
// STM32 implementation of Arduino compatible serial class

#include <RadioHead.h>
#if (RH_PLATFORM == RH_PLATFORM_STM32STD)
#ifndef _HardwareSerial_h
#define _HardwareSerial_h

#include <stdint.h>
#include <stdio.h>
#include <stm32f4xx.h>

#ifndef ARDUINO_RINGBUFFER_SIZE
#define ARDUINO_RINGBUFFER_SIZE 64
#endif

class RingBuffer
{
public:
    RingBuffer();
    bool    isEmpty();
    bool    isFull();
    bool    write(uint8_t ch);
    uint8_t read();

private:
    uint8_t _buffer[ARDUINO_RINGBUFFER_SIZE]; // In fact we can hold up to ARDUINO_RINGBUFFER_SIZE-1 bytes
    uint16_t _head;      // Index of next write
    uint16_t _tail;      // Index of next read
    uint32_t _overruns;  // Write attempted when buffer full
    uint32_t _underruns; // Read attempted when buffer empty
};

// Mostly compatible wuith Arduino HardwareSerial
// Theres just enough here to support RadioHead RH_Serial
class HardwareSerial
{
public:
    HardwareSerial(USART_TypeDef* usart);
    void begin(unsigned long baud);
    void end();
    virtual int available(void);
    virtual int read(void);
    virtual size_t write(uint8_t);
    inline size_t write(unsigned long n) { return write((uint8_t)n); }
    inline size_t write(long n) { return write((uint8_t)n); }
    inline size_t write(unsigned int n) { return write((uint8_t)n); }
    inline size_t write(int n) { return write((uint8_t)n); }

    // These need to be public so the IRQ handler can read and write to them:
    RingBuffer     _rxRingBuffer;
    RingBuffer     _txRingBuffer;

private:
    USART_TypeDef* _usart;

};

// Predefined serial ports are configured so:
// Serial       STM32 UART   RX pin   Tx Pin   Comments
// Serial1      USART1       PA10     PA9      TX Conflicts with GREEN LED on Discovery
// Serial2      USART2       PA3      PA2
// Serial3      USART3       PD9      PD10     
// Serial4      UART4        PA1      PA0      TX conflicts with USER button on Discovery
// Serial5      UART5        PD2      PC12     TX conflicts with CS43L22 SDIN on Discovery
// Serial6      USART6       PC7      PC6      RX conflicts with CS43L22 MCLK on Discovery
//
// All ports are idle HIGH, LSB first, 8 bits, No parity, 1 stop bit
extern HardwareSerial Serial1;
extern HardwareSerial Serial2;
extern HardwareSerial Serial3;
extern HardwareSerial Serial4;
extern HardwareSerial Serial5;
extern HardwareSerial Serial6;

#endif

#endif
