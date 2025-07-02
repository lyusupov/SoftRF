/*
  Server.h - Server class for Raspberry Pi
  Copyright (c) 2016 Hristo Gochkov  All right reserved.

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

#ifndef pi_server_h
#define pi_server_h

#if defined(ARDUINO)
#include <Arduino.h>
#include "Server.h"
#endif /* ARDUINO */

#if defined(RASPBERRY_PI) || defined(LUCKFOX_LYRA)
#include <raspi/raspi.h>
#include "raspi/Server.h"
#endif /* RASPBERRY_PI */

#include "BridgeClient.h"

typedef void(*BridgeServerHandler)(Client&);

class BridgeServer : public Server {
  private:
    int sockfd;
    int pollfd;
    struct epoll_event *events;
    uint16_t _port;
    uint8_t _max_clients;
    bool _listening;
    BridgeServerHandler _cb;
    BridgeClient *clients;

    //cleans all disconnected clients
    void cleanup();
    int setSocketOption(int option, char* value, size_t len);

  public:
    void listenOnLocalhost(){}

    BridgeServer(uint16_t port=80, uint8_t max_clients=8):sockfd(-1),pollfd(0),events(NULL),_port(port),_max_clients(max_clients),_listening(false),_cb(NULL),clients(NULL){}
    ~BridgeServer(){ end();}
    BridgeClient available();
    BridgeClient accept(){return available();}
    virtual void begin();
    virtual size_t write(const uint8_t *data, size_t len);
    virtual size_t write(uint8_t data){
      return write(&data, 1);
    }
    using Print::write;

    void end();
    operator bool(){return _listening;}
    int setTimeout(uint32_t seconds);
    BridgeClient *clientByFd(int fd){
      BridgeClient *c = clients;
      while(c != NULL && c->fd() != fd) c = c->next;
      return c;
    }
    void stopAll();
};

#endif
