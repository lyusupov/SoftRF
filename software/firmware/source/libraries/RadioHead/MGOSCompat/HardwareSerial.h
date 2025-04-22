// ArduinoCompat/HardwareSerial.h
// Mongoose OS implementation of Arduino compatible serial class

#include <RadioHead.h>
#if (RH_PLATFORM == RH_PLATFORM_MONGOOSE_OS)
#ifndef _HardwareSerial_h
#define _HardwareSerial_h

#include <mgos.h>
#include <stdint.h>
#include <stdio.h>

// Mostly compatible wuith Arduino HardwareSerial
// There is just enough here to support RadioHead RH_Serial
class HardwareSerial
{
public:
    HardwareSerial(int uart_index);
    void begin(int baud);
    void end();
    virtual int available(void);
    virtual int read(void);
    virtual size_t write(uint8_t);
    inline size_t write(unsigned long n) { return write((uint8_t)n); }
    inline size_t write(long n) { return write((uint8_t)n); }
    inline size_t write(unsigned int n) { return write((uint8_t)n); }
    inline size_t write(int n) { return write((uint8_t)n); }

    //These methods will send debug info on the debug serial port (if enabled)
    size_t println(unsigned char ch, int base);
    size_t print(unsigned char ch, int base);
    size_t println(const char ch);
    size_t print(const char ch);
    size_t println(const char* s);
    size_t print(const char* s);

private:
    int uartIndex;
    size_t rxByteCountAvail;
    uint8_t rxByte;
};

extern HardwareSerial Serial0;
extern HardwareSerial Serial1;
extern HardwareSerial Serial2;

#endif

#endif
