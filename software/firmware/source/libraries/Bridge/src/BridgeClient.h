/*
  Client.h - Base class that provides Client
  Copyright (c) 2011 Adrian McEwen.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef pi_client_h
#define pi_client_h

#if defined(ARDUINO)
#include <Arduino.h>
#include "Client.h"
#endif /* ARDUINO */

#if defined(RASPBERRY_PI) || defined(LUCKFOX_LYRA)
#include <raspi/raspi.h>
#include "raspi/Client.h"
#endif /* RASPBERRY_PI */

class BridgeClient : public Client {
  protected:
    int sockfd;
    bool _connected;

  public:
    BridgeClient *next;
    BridgeClient():sockfd(-1),_connected(false),next(NULL){}
    BridgeClient(int fd):sockfd(fd),_connected(true),next(NULL){}
    ~BridgeClient();
    virtual int connect(IPAddress ip, uint16_t port);
    virtual int connect(const char *host, uint16_t port);
    virtual size_t write(uint8_t data);
    virtual size_t write(const uint8_t *buf, size_t size);
    virtual int available();
    virtual int read();
    virtual int read(uint8_t *buf, size_t size);
    virtual int peek(){return 0;}
    virtual void flush(){}
    virtual void stop();
    virtual uint8_t connected();
    virtual operator bool(){ return connected(); }
    virtual bool operator==(const bool value) { return bool() == value; }
    virtual bool operator!=(const bool value) { return bool() != value; }
    virtual bool operator==(const BridgeClient&);
    virtual bool operator!=(const BridgeClient& rhs) { return !this->operator==(rhs); };

    int fd(){return sockfd;}
    IPAddress remoteIP();
    uint16_t remotePort();
    int setSocketOption(int option, char* value, size_t len);
    int setOption(int option, int *value);
    int getOption(int option, int *value);
    int setTimeout(uint32_t seconds);
    int setNoDelay(bool nodelay);
    bool getNoDelay();

    static IPAddress remoteIP(int fd);
    static uint16_t remotePort(int fd);
    
    friend class BridgeServer;
    using Print::write;
};

#endif
