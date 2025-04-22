// HardwareSerial.cpp
//
// Copyright (C) 2015 Mike McCauley
// $Id: HardwareSerial.cpp,v 1.4 2020/08/05 04:32:19 mikem Exp mikem $

#include <RadioHead.h>
#if (RH_PLATFORM == RH_PLATFORM_UNIX)

#include <HardwareSerial.h>

#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/select.h>

HardwareSerial::HardwareSerial(const char* deviceName)
    : _deviceName(deviceName),
      _device(-1)
{
    // Override device name from environment
    char* e = getenv("RH_HARDWARESERIAL_DEVICE_NAME");
    if (e)
	_deviceName = e;
}

void HardwareSerial::begin(int baud)
{
    if (openDevice())
	setBaud(baud);
}

void HardwareSerial::end()
{
    closeDevice();
}

void HardwareSerial::flush()
{
    tcdrain(_device);
}

int HardwareSerial::peek(void)
{
    printf("HardwareSerial::peek not implemented\n");
    return 0;
}

int HardwareSerial::available()
{
    int bytes;

    if (ioctl(_device, FIONREAD, &bytes) != 0)
    {
	fprintf(stderr, "HardwareSerial::available ioctl failed: %s\n", strerror(errno));
	return 0;
    }
    return bytes;
}

int HardwareSerial::read()
{
    uint8_t data;
    ssize_t result = ::read(_device, &data, 1);
    if (result != 1)
    {
	fprintf(stderr, "HardwareSerial::read read failed: %s\n", strerror(errno));
	return 0;
    }
//    printf("got: %02x\n", data);
    return data;
}

size_t HardwareSerial::write(uint8_t ch)
{
    size_t result = ::write(_device, &ch, 1);
    if (result != 1)
    {
	fprintf(stderr, "HardwareSerial::write failed: %s\n", strerror(errno));
	return 0;
    }
//    printf("sent: %02x\n", ch);
    return 1; // OK
}

bool HardwareSerial::openDevice()
{
    if (_device == -1)
	closeDevice();
    _device = open(_deviceName, O_RDWR | O_NOCTTY | O_NDELAY);
    if (_device == -1)
    {
	// Could not open the port.
	fprintf(stderr, "HardwareSerial::openDevice could not open %s: %s\n", _deviceName, strerror(errno));
	return false;
    }

    // Device opened
    fcntl(_device, F_SETFL, 0);
    return true;
}

bool HardwareSerial::closeDevice()
{
    if (_device != -1)
	close(_device);
    _device = -1;
    return true;
}

bool HardwareSerial::setBaud(int baud)
{
    speed_t speed;

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
    else 
    {
	fprintf(stderr, "HardwareSerial::setBaud: unsupported baud rate %d\n", baud);
	return false;
    }

    struct termios options;
    // Get current options
    if (tcgetattr(_device, &options) != 0)
    {
	fprintf(stderr, "HardwareSerial::setBaud: could not tcgetattr %s\n", strerror(errno));
	return false;
    }

    // Set new speed options
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);
    // Enable the receiver and set local mode...
    options.c_cflag |= (CLOCAL | CREAD);

    // Force mode to 8,N,1
    // to be compatible with Arduino HardwareSerial
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
	fprintf(stderr, "HardwareSerial::setBaud: could not tcsetattr %s\n", strerror(errno));
	return false;
    }

    _baud = baud;
    return true;
}

// Block until something is available
void HardwareSerial::waitAvailable()
{
    waitAvailableTimeout(0); // 0 = Wait forever
}

// Block until something is available or timeout expires
bool HardwareSerial::waitAvailableTimeout(uint16_t timeout)
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
	fprintf(stderr, "HardwareSerial::waitAvailableTimeout: select failed %s\n", strerror(errno));
    return result > 0;
}

#endif
