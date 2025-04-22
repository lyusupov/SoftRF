// RH_TCP.cpp
//
// Copyright (C) 2014 Mike McCauley
// $Id: RH_TCP.cpp,v 1.6 2017/01/12 23:58:00 mikem Exp $

#include <RadioHead.h>

// This can only build on Linux and compatible systems
#if (RH_PLATFORM == RH_PLATFORM_UNIX) 

#include <RH_TCP.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <netdb.h>
#include <string>

RH_TCP::RH_TCP(const char* server)
    : _server(server),
      _rxBufLen(0),
      _rxBufValid(false),
      _socket(-1)
{
}
    
bool RH_TCP::init()
{   
    if (!connectToServer())
	return false;
    return sendThisAddress(_thisAddress);
}
    
bool RH_TCP::connectToServer()
{
    struct addrinfo hints;
    struct addrinfo *result, *rp;
    int sfd, s;
    struct sockaddr_storage peer_addr;
    socklen_t peer_addr_len;

    memset(&hints, 0, sizeof(struct addrinfo));
    hints.ai_family = AF_UNSPEC;    // Allow IPv4 or IPv6
    hints.ai_socktype = SOCK_STREAM; // Stream socket
    hints.ai_flags = AI_PASSIVE;    // For wildcard IP address 
    hints.ai_protocol = 0;          // Any protocol 
    hints.ai_canonname = NULL;
    hints.ai_addr = NULL;
    hints.ai_next = NULL;
    
    std::string server(_server);
    std::string port("4000");
    size_t indexOfSeparator = server.find_first_of(':');
    if (indexOfSeparator != std::string::npos)
    {
	port = server.substr(indexOfSeparator+1);
	server.erase(indexOfSeparator);
    }

    s = getaddrinfo(server.c_str(), port.c_str(), &hints, &result);
    if (s != 0) 
    {
	fprintf(stderr, "RH_TCP::connect getaddrinfo failed: %s\n", gai_strerror(s));
	return false;
    }

    // getaddrinfo() returns a list of address structures.
    // Try each address until we successfully connect(2).
    // If socket(2) (or connect(2)) fails, we (close the socket
    // and) try the next address. */

    for (rp = result; rp != NULL; rp = rp->ai_next) 
    {
	_socket = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
	if (_socket == -1)
	    continue;
	
	if (connect(_socket, rp->ai_addr, rp->ai_addrlen) == 0)
	    break;                  /* Success */

	close(_socket);
    }

    if (rp == NULL) 
    {               /* No address succeeded */
	fprintf(stderr, "RH_TCP::connect could not connect to %s\n", _server);
	return false;
    }

    freeaddrinfo(result);           /* No longer needed */

    // Now make the socket non-blocking
    int on = 1;
    int rc = ioctl(_socket, FIONBIO, (char *)&on);
    if (rc < 0)
    {
	fprintf(stderr,"RH_TCP::init failed to set socket non-blocking: %s\n", strerror(errno));
	close(_socket);
	_socket = -1;
	return false;
    }
    return true;
}

void RH_TCP::clearRxBuf()
{
    _rxBufValid = false;
    _rxBufLen = 0;
}

bool RH_TCP::checkForEvents()
{
    #define RH_TCP_SOCKETBUF_LEN 500
    static uint8_t socketBuf[RH_TCP_SOCKETBUF_LEN]; // Room for several messages
    static uint16_t socketBufLen = 0;

    if (_socket < 0)
	return false;

    // Read at most the amount of space we have left in the buffer
    ssize_t count = read(_socket, socketBuf + socketBufLen, sizeof(socketBuf) - socketBufLen);
    if (count < 0)
    {
	if (errno != EAGAIN)
	{
	    fprintf(stderr,"RH_TCP::checkForEvents read error: %s\n", strerror(errno));
	    close(_socket);
	    _socket = -1;
	    return false;
	}
    }
    else if (count == 0)
    {
	// End of file
	fprintf(stderr,"RH_TCP::checkForEvents unexpected end of file on read\n");
	close(_socket);
	_socket = -1;
	return false;
    }
    else
    {
	socketBufLen += count;
	while (socketBufLen >= 5)
	{
	    RHTcpTypeMessage* message = ((RHTcpTypeMessage*)socketBuf);
	    uint32_t len = ntohl(message->length);
	    uint32_t messageLen = len + sizeof(message->length);
	    if (len > sizeof(socketBuf) - sizeof(message->length))
	    {
		// Bogus length
		fprintf(stderr, "RH_TCP::checkForEvents read ridiculous length: %d. Corrupt message stream? Aborting\n", len);
		close(_socket);
		_socket = -1;
		return false;
	    }
	    if (socketBufLen >= len + sizeof(message->length))
	    {
		// Got at least all of this message
		if (message->type == RH_TCP_MESSAGE_TYPE_PACKET && len >= 5)
		{
		    // REVISIT: need to check if we are actually receiving?
		    // Its a new packet, extract the headers and payload
		    RHTcpPacket* packet = ((RHTcpPacket*)socketBuf);
		    _rxHeaderTo    = packet->to;
		    _rxHeaderFrom  = packet->from;
		    _rxHeaderId    = packet->id;
		    _rxHeaderFlags = packet->flags;
		    uint32_t payloadLen = len - 5;
		    if (payloadLen <= sizeof(_rxBuf))
		    {
			// Enough room in our receiver buffer
			memcpy(_rxBuf, packet->payload, payloadLen);
			_rxBufLen = payloadLen;
			_rxBufFull = true;
		    }
		}
		// check for other message types here
		// Now remove the used message by copying the trailing bytes (maybe start of a new message?)
		// to the top of the buffer
		memcpy(socketBuf, socketBuf + messageLen, sizeof(socketBuf) - messageLen);
		socketBufLen -= messageLen;
	    }
	}
    }
    return true; // No faults
}

void RH_TCP::validateRxBuf()
{
    // The headers have already been extracted
    if (_promiscuous ||
	_rxHeaderTo == _thisAddress ||
	_rxHeaderTo == RH_BROADCAST_ADDRESS)
    {
	_rxGood++;
	_rxBufValid = true;
    }
}

bool RH_TCP::available()
{
    if (_socket < 0)
	return false;
    if (checkForEvents())
	return false;        // Som sort of IO failre
    if (_rxBufFull)
    {
	validateRxBuf();
	_rxBufFull= false;
    }
    return _rxBufValid;
}

// Block until something is available
void RH_TCP::waitAvailable(uint16_t polldelay)
{
    waitAvailableTimeout(0); // 0 = Wait forever, no polldelay
}

// Block until something is available or timeout expires
bool RH_TCP::waitAvailableTimeout(uint16_t timeout, uint16_t polldelay)
{
    int            max_fd;
    fd_set         input;
    int            result;

    FD_ZERO(&input);
    FD_SET(_socket, &input);
    max_fd = _socket + 1;

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
	fprintf(stderr, "RH_TCP::waitAvailableTimeout: select failed %s\n", strerror(errno));
    return result > 0;
}

bool RH_TCP::recv(uint8_t* buf, uint8_t* len)
{
    if (!available())
	return false;

    if (buf && len)
    {
	if (*len > _rxBufLen)
	    *len = _rxBufLen;
	memcpy(buf, _rxBuf, *len);
    }
    clearRxBuf();
    return true;
}

bool RH_TCP::send(const uint8_t* data, uint8_t len)
{
    if (!waitCAD()) 
	return false;  // Check channel activity (prob not possible for this driver?)

    bool ret = sendPacket(data, len);
    delay(10); // Wait for transmit to succeed. REVISIT: depends on length and speed
    return ret;
}

uint8_t RH_TCP::maxMessageLength()
{
    return RH_TCP_MAX_MESSAGE_LEN;
}

void RH_TCP::setThisAddress(uint8_t address)
{
    RHGenericDriver::setThisAddress(address);
    sendThisAddress(_thisAddress);
}

bool RH_TCP::sendThisAddress(uint8_t thisAddress)
{
    if (_socket < 0)
	return false;
    RHTcpThisAddress m;
    m.length = htonl(2);
    m.type = RH_TCP_MESSAGE_TYPE_THISADDRESS;
    m.thisAddress = thisAddress;
    ssize_t sent = write(_socket, &m, sizeof(m));
    return sent > 0;
}

bool RH_TCP::sendPacket(const uint8_t* data, uint8_t len)
{
    if (_socket < 0)
	return false;
    RHTcpPacket m;
    m.length = htonl(len + 5); // 5 octets of header
    m.type  = RH_TCP_MESSAGE_TYPE_PACKET;
    m.to    = _txHeaderTo;
    m.from  = _txHeaderFrom;
    m.id    = _txHeaderId;
    m.flags = _txHeaderFlags;
    memcpy(m.payload, data, len);
    ssize_t sent = write(_socket, &m, len + 9); // length + 5 octets header
    return sent > 0;
}

#endif
