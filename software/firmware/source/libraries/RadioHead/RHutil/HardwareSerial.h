// HardwareSerial.h
// Author: Mike McCauley (mikem@airspayce.com)
// Copyright (C) 2015 Mike McCauley
// $Id: HardwareSerial.h,v 1.4 2020/08/05 04:32:19 mikem Exp mikem $
#ifndef HardwareSerial_h
#define HardwareSerial_h

#include <stdio.h>

/////////////////////////////////////////////////////////////////////
/// \class HardwareSerial HardwareSerial.h <RHutil/HardwareSerial.h>
/// \brief Encapsulates a Posix compliant serial port as a HarwareSerial
///
/// This class provides access to a serial port on Unix and OSX.
/// It is equivalent to HardwareSerial in Arduino, and can be used by RH_Serial
/// We implement just enough to provide the services RadioHead needs.
/// Additional methods not present on Arduino are also provided for waiting for characters.
///
/// The device port is configured for 8 bits, no parity, 1 stop bit and full raw transparency, so it can be used
/// to send and receive any 8 bit character. A limited range of baud rates is supported.
///
/// \par Device Names
///
/// Device naming conventions vary from OS to OS. ON linux, an FTDI serial port may have a name like
/// /dev/ttyUSB0. On OSX, it might be something like /dev/tty.usbserial-A501YSWL
/// \par errors
///
/// A number of these methods print error messages to stderr in the event of an IO error.
class HardwareSerial
{
public:
    /// Constructor
    // \param [in] deviceName Name of the derial port device to connect to
    HardwareSerial(const char* deviceName);

    /// Open and configure the port.
    /// The named port is opened, and the given baud rate is set.
    /// The port is configure for raw input and output and 8,N,1 protocol
    /// with no flow control.
    /// This must be called before any other operations are attempted.
    /// IO failures and unsupported baud rates will result in an error message on stderr.
    /// \param[in] baud The desired baud rate. The only rates supported are: 50, 75, 110, 134, 150
    /// 200, 300, 600, 1200, 1800, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400. On some platform
    /// such as Linux you may also use: 460800, 921600. 
    void begin(int baud);

    /// Close the port.
    /// If begin() has previously been called successfully, the device port will be closed.
    /// It may be reopened again with another call to begin().
    void end();

    /// Flush remaining data.
    /// Blocks until any data yet to be transmtted is sent.
    void flush();

    /// Peek at the nex available character without consuming it.
    /// CAUTION: Not implemented.
    int peek(void);

    /// Returns the number of bytes immediately available to be read from the
    /// device.
    /// \return 0 if none available else the number of characters available for immediate reading
    int available();

    /// Read and return the next available character.
    /// If no character is available prints a message to stderr and returns 0;
    /// \return The next available character
    int read();

    /// Transmit a single character oin the serial port.
    /// Returns immediately.
    /// IO errors are repored by printing aa message to stderr.
    /// \param[in] ch The character to send. Anything in the range 0x00 to 0xff is permitted
    /// \return 1 if successful else 0
    size_t write(uint8_t ch);

    // These are not usually in HardwareSerial but we 
    // need them in a Unix environment

    /// Wait until a character is available from the port.
    void waitAvailable();

    /// Wait until a a character is available from the port.
    /// or the timeout expires
    /// \param[in] timeout The maximum time to wait in milliseconds. 0 means wait forever.
    /// \return true if a message is available as reported by available()
    bool waitAvailableTimeout(uint16_t timeout);

protected:
    bool openDevice();
    bool closeDevice();
    bool setBaud(int baud);

private:
    const char* _deviceName;
    int         _device; // file desriptor
    int         _baud;
};

#endif
