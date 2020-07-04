// HardwareSerial.cpp
//
// Copyright (C) 2015 Mike McCauley
// $Id: HardwareSerial.cpp,v 1.3 2015/08/13 02:45:47 mikem Exp mikem $

#ifdef RASPBERRY_PI

#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <stdlib.h>

#include "TTYSerial.h"

TTYSerial::TTYSerial(const char* deviceName)
    : _deviceName(deviceName),
      _device(-1)
{
    // Override device name from environment
    char* e = getenv("TTYSERIAL_DEVICE_NAME");
    if (e)
	_deviceName = e;
}

void TTYSerial::begin(int baud)
{
    if (openDevice())
	setBaud(baud);
}

void TTYSerial::end()
{
    closeDevice();
}

void TTYSerial::flush()
{
    tcdrain(_device);
}

int TTYSerial::peek(void)
{
    printf("TTYSerial::peek not implemented\n");
    return 0;
}

int TTYSerial::available()
{
    int bytes;

    if (_device == -1)
      return 0;

    if (ioctl(_device, FIONREAD, &bytes) != 0)
    {
	fprintf(stderr, "TTYSerial::available ioctl failed: %s\n", strerror(errno));
	return 0;
    }
    return bytes;
}

int TTYSerial::read()
{
    uint8_t data;

    if (_device == -1)
      return 0;

    ssize_t result = ::read(_device, &data, 1);
    if (result != 1)
    {
	fprintf(stderr, "TTYSerial::read read failed: %s\n", strerror(errno));
	return 0;
    }
//    printf("got: %02x\n", data);
    return data;
}

size_t TTYSerial::write(uint8_t ch)
{
    if (_device == -1)
      return 0;

    size_t result = ::write(_device, &ch, 1);
    if (result != 1)
    {
	fprintf(stderr, "TTYSerial::write failed: %s\n", strerror(errno));
	return 0;
    }
//    printf("sent: %02x\n", ch);
    return 1; // OK
}

size_t TTYSerial::write(unsigned char* s, size_t len) {

    if (_device == -1)
      return 0;

    size_t result = ::write(_device, s, len);
    if (result != len)
    {
	fprintf(stderr, "TTYSerial::write failed: %s\n", strerror(errno));
	return 0;
    }
//    printf("sent: %02x\n", ch);
    return result; // OK
}

size_t TTYSerial::write(const char* s) {
    size_t len = strlen(s);

    if (_device == -1)
      return 0;

    size_t result = ::write(_device, s, len);
    if (result != len)
    {
	fprintf(stderr, "TTYSerial::write failed: %s\n", strerror(errno));
	return 0;
    }
//    printf("sent: %02x\n", ch);
    return result; // OK
}

bool TTYSerial::openDevice()
{
    if (_device == -1)
	closeDevice();
    _device = open(_deviceName, O_RDWR | O_NOCTTY | O_NDELAY);
    if (_device == -1)
    {
	// Could not open the port.
	fprintf(stderr, "WARNING! TTYSerial::openDevice could not open %s: %s\n", _deviceName, strerror(errno));
	return false;
    }

    // Device opened
    fcntl(_device, F_SETFL, 0);
    return true;
}

bool TTYSerial::closeDevice()
{
    if (_device != -1)
	close(_device);
    _device = -1;
    return true;
}

bool TTYSerial::setBaud(int baud)
{
    speed_t speed;

    if (_device == -1)
      return false;

    // This is kind of ugly, but its prob better than a case
    if (baud == 50)
	speed = B50;
    else if (baud == 75)
	speed = B75;
    else if (baud == 110)
	speed = B110;
    else if (baud == 134)
	speed = B134;
    else if (baud == 150)
	speed = B150;
    else if (baud == 200)
	speed = B200;
    else if (baud == 300)
	speed = B300;
    else if (baud == 600)
	speed = B600;
    else if (baud == 1200)
	speed = B1200;
    else if (baud == 1800)
	speed = B1800;
    else if (baud == 2400)
	speed = B2400;
    else if (baud == 4800)
	speed = B4800;
    else if (baud == 9600)
	speed = B9600;
    else if (baud == 19200)
	speed = B19200;
    else if (baud == 38400)
	speed = B38400;
    else if (baud == 57600)
	speed = B57600;
#ifdef B76800
    else if (baud == 76800)  // Not available on Linux
	speed = B76800;
#endif
    else if (baud == 115200)
	speed = B115200;
    else if (baud == 230400)
	speed = B230400;
#ifdef B460800
    else if (baud == 460800) // Not available on OSX
	speed = B460800;
#endif
#ifdef B921600
    else if (baud == 921600) // Not available on OSX
	speed = B921600;
#endif
#ifdef B1000000
    else if (baud == 1000000)
	speed = B1000000;
#endif
#ifdef B1152000
    else if (baud == 1152000)
	speed = B1152000;
#endif
#ifdef B1500000
    else if (baud == 1500000)
	speed = B1500000;
#endif
#ifdef B2000000
    else if (baud == 2000000)
	speed = B2000000;
#endif
    else 
    {
	fprintf(stderr, "TTYSerial::setBaud: unsupported baud rate %d\n", baud);
	return false;
    }

    struct termios options;
    // Get current options
    if (tcgetattr(_device, &options) != 0)
    {
	fprintf(stderr, "TTYSerial::setBaud: could not tcgetattr %s\n", strerror(errno));
	return false;
    }

    // Set new speed options
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);
    // Enable the receiver and set local mode...
    options.c_cflag |= (CLOCAL | CREAD);

    // Force mode to 8,N,1
    // to be compatible with Arduino TTYSerial
    // Should this be configurable? Prob not, must have 8 bits, dont need parity.
    options.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
    options.c_cflag |= CS8;
   
    // Disable flow control and input character conversions
    options.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR);

    // Raw input:
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // Raw output
    options.c_oflag &= ~(OPOST | OCRNL | ONLCR);

    // Set the options in the port
    if (tcsetattr(_device, TCSANOW, &options) != 0)
    {
	fprintf(stderr, "TTYSerial::setBaud: could not tcsetattr %s\n", strerror(errno));
	return false;
    }

    _baud = baud;
    return true;
}

bool TTYSerial::rts(bool value)
{
    int RTS_flag = TIOCM_RTS;
    int err;

    if (_device == -1)
      return false;

    if (value)
      err = ioctl(_device,TIOCMBIS,&RTS_flag);
    else
      err = ioctl(_device,TIOCMBIC,&RTS_flag);

    return (err == -1 ? false : true);
}

bool TTYSerial::dtr(bool value)
{
    int DTR_flag = TIOCM_DTR;
    int err;

    if (_device == -1)
      return false;

    if (value)
      err = ioctl(_device,TIOCMBIS,&DTR_flag);
    else
      err = ioctl(_device,TIOCMBIC,&DTR_flag);

    return (err == -1 ? false : true);
}

// Block until something is available
void TTYSerial::waitAvailable()
{
    waitAvailableTimeout(0); // 0 = Wait forever
}

// Block until something is available or timeout expires
bool TTYSerial::waitAvailableTimeout(uint16_t timeout)
{
    int            max_fd;
    fd_set         input;
    int            result;

    FD_ZERO(&input);
    FD_SET(_device, &input);
    max_fd = _device + 1;

    if (timeout)
    {
	struct timeval timer;
	// Timeout is in milliseconds
	timer.tv_sec  = timeout / 1000;
	timer.tv_usec = (timeout % 1000) * 1000;
	result = select(max_fd, &input, NULL, NULL, &timer);
    }
    else
    {
	result = select(max_fd, &input, NULL, NULL, NULL);
    }
    if (result < 0)
	fprintf(stderr, "TTYSerial::waitAvailableTimeout: select failed %s\n", strerror(errno));
    return result > 0;
}

#endif // RASPBERRY_PI
