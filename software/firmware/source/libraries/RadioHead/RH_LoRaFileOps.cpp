// RH_LoRaFileOpscpp
//
// Copyright (C) 2021 Mike McCauley

#include <RH_LoRaFileOps.h>

// This can only build on Linux and compatible systems
// Caution also requires Lora-file-ops driver to be installed
// See https://github.com/starnight/LoRa/tree/file-ops
#if (RH_PLATFORM == RH_PLATFORM_UNIX) 

#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/select.h>

RH_LoRaFileOps::RH_LoRaFileOps(const char* port)
    :
    _port(port),
    _fd(-1)
{
}

bool RH_LoRaFileOps::init()
{
    if (!RHGenericDriver::init())
	return false;

    _fd = open(_port, O_RDWR);
    if (_fd == -1)
    {
	Serial.print("RH_LoRaFileOps::init LORA device port open failed: ");
	Serial.println(strerror(errno));
	return false;
    }
  
    // These settings are compatible with RH_RF95::Bw125Cr45Sf2048 and the other defaults
    // for RH_RF95
    setFrequency(434000000);
    setTxPower(13);
    setSpreadingFactor(2048);
    setBW(125000);
    setCRC(true);
    setLNA(0);

    return true;
}

bool RH_LoRaFileOps::available()
{
    // Test if the port can be read
    fd_set read_fds;
    struct timeval tv = {.tv_sec = 0, .tv_usec = 0};
    FD_ZERO(&read_fds);
    FD_SET(_fd, &read_fds);
    if (select(_fd+1, &read_fds, NULL, NULL, &tv) == -1)
    {
	Serial.println("Select failed");
	return false;
    }
  
    return FD_ISSET(_fd, &read_fds) ? true : false;
    
}

bool RH_LoRaFileOps::recv(uint8_t* buf, uint8_t* len)
{
    if (_fd == -1) return false;

    if (!available()) return false;

    // Read the available packet from the driver
    uint8_t readbuf[RH_LORAFILEOPS_MAX_PAYLOAD_LEN];
    ssize_t sz;
    sz = read(_fd, readbuf, RH_LORAFILEOPS_MAX_PAYLOAD_LEN);

    // Remember the last signal to noise ratio, LORA mode
    // Per page 111, SX1276/77/78/79 datasheet
    _lastSNR = getSNR();

    // Remember the RSSI of this packet, LORA mode
    // This figure has already been massaged by the driver 
    _lastRssi = getRSSI();

    // Test if its really for us
    if (sz < 4)
	return false; // Too short to be a real message
    // Extract the 4 headers
    _rxHeaderTo    = readbuf[0];
    _rxHeaderFrom  = readbuf[1];
    _rxHeaderId    = readbuf[2];
    _rxHeaderFlags = readbuf[3];
    if (_promiscuous ||
	_rxHeaderTo == _thisAddress ||
	_rxHeaderTo == RH_BROADCAST_ADDRESS)
    {
	// Yes its for us
	// Skip the 4 headers that are at the beginning of the rxBuf
	if (*len > (sz - RH_LORAFILEOPS_HEADER_LEN))
	    *len = sz - RH_LORAFILEOPS_HEADER_LEN;
	memcpy(buf, readbuf + RH_LORAFILEOPS_HEADER_LEN, *len);
	Serial.println("recv OK");
	return true;
    }

    // Not for us
    return false;
  
    
}

bool RH_LoRaFileOps::send(const uint8_t* data, uint8_t len)
{
    if (_fd == -1) return false;

    if (len > RH_LORAFILEOPS_MAX_MESSAGE_LEN)
	return false;

    // Should never need to wait for a packet to be sent since the driver always
    // waits before returning from write


    // Assemble the entire message, including RadioHead header
    // Header bytes, total size of RH_LORAFILEOPS_HEADER_LEN
    uint8_t buf[RH_LORAFILEOPS_MAX_PAYLOAD_LEN];
    buf[0] = _txHeaderTo;
    buf[1] = _txHeaderFrom;
    buf[2] = _txHeaderId;
    buf[3] = _txHeaderFlags;
    memcpy(buf+4, data, len);

    // Send the message to the driver
  
    ssize_t sz;
    // Returns when the packet has been transmtted:
    sz = write(_fd, buf, len+RH_LORAFILEOPS_HEADER_LEN);
    return sz == len+RH_LORAFILEOPS_HEADER_LEN;
}

uint8_t RH_LoRaFileOps::maxMessageLength()
{
    return RH_LORAFILEOPS_MAX_MESSAGE_LEN;
}

bool RH_LoRaFileOps::setFrequency(uint32_t centre)
{
    if (_fd == -1) return false;

    ioctl(_fd, LORA_SET_FREQUENCY, &centre);
    return true; // FIXME??
}

uint32_t RH_LoRaFileOps::getFrequency()
{
    if (_fd == -1) return 0;

    uint32_t freq;
    ioctl(_fd, LORA_GET_FREQUENCY, &freq);
    return freq;
}

void RH_LoRaFileOps::setTxPower(int32_t power)
{
    if (_fd == -1) return;

    ioctl(_fd, LORA_SET_POWER, &power);
}

int32_t RH_LoRaFileOps::getTxPower()
{
    if (_fd == -1) return 0;

    int32_t power;
    ioctl(_fd, LORA_GET_POWER, &power);
    return power;
}

void RH_LoRaFileOps::setState(uint32_t state)
{
    if (_fd == -1) return;

    ioctl(_fd, LORA_SET_STATE, &state);
}

uint32_t RH_LoRaFileOps::getState()
{
    if (_fd == -1) return LORA_STATE_SLEEP;

    uint32_t state;
    ioctl(_fd, LORA_GET_STATE, &state);
    return state;
}

void RH_LoRaFileOps::setSpreadingFactor(int32_t sf)
{
    ioctl(_fd, LORA_SET_SPRFACTOR, &sf);
}

int32_t RH_LoRaFileOps::getSpreadingFactor()
{
    if (_fd == -1) return 0;

    uint32_t sprf;
    ioctl(_fd, LORA_GET_SPRFACTOR, &sprf);
    return sprf;
}

// This gets the raw RSSI figure. Its not adjusted to dBm
int32_t RH_LoRaFileOps::getRSSI()
{
    if (_fd == -1) return 0;

    int32_t rssi;
    ioctl(_fd, LORA_GET_RSSI, &rssi);
    return rssi;
}

// Last packet SNR
int32_t RH_LoRaFileOps::getSNR()
{
    if (_fd == -1) return 0;

    int32_t snr;
    ioctl(_fd, LORA_GET_SNR, &snr);
    return snr;
}

void RH_LoRaFileOps::setLNA(int32_t lna)
{
    if (_fd == -1) return;

    ioctl(_fd, LORA_SET_LNA, &lna);
}

int32_t RH_LoRaFileOps::getLNA()
{
    if (_fd == -1) return 0;

    int32_t lna;
    ioctl(_fd, LORA_GET_LNA, &lna);
    return lna;
}
    
void RH_LoRaFileOps::setLNAAGC(int32_t lnaagc)
{
    if (_fd == -1) return;

    ioctl(_fd, LORA_SET_LNAAGC, &lnaagc);
}
			    
void RH_LoRaFileOps::setBW(int32_t bw)
{
    if (_fd == -1) return;

    ioctl(_fd, LORA_SET_BANDWIDTH, &bw);
}

int32_t RH_LoRaFileOps::getBW()
{
    if (_fd == -1) return 0;
  
    uint32_t bw;
    ioctl(_fd, LORA_GET_BANDWIDTH, &bw);
    return bw;
}

void RH_LoRaFileOps::setCRC(uint32_t crc)
{
    if (_fd == -1) return;

    ioctl(_fd, LORA_SET_CRC, &crc);
}

int RH_LoRaFileOps::lastSNR()
{
    return _lastSNR;
}

#endif
