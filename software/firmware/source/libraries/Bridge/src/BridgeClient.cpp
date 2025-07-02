/*
  Client.h - Client class for Raspberry Pi
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



#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <netinet/tcp.h>
#include <sys/ioctl.h>
#include "BridgeClient.h"

int sock_connect(int fd, struct sockaddr *addr, size_t len){
  return connect(fd, addr, len);
}

int sock_write(int fd, void *data, size_t len){
  return write(fd, data, len);
}

int sock_read(int fd, void *data, size_t len){
  return read(fd, data, len);
}

BridgeClient::~BridgeClient(){
  //if(sockfd) close(sockfd);
}

int BridgeClient::connect(IPAddress ip, uint16_t port){
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0){
    return 0;
  }
  uint32_t ip_addr = ip;
  struct sockaddr_in serveraddr;
  bzero((char *) &serveraddr, sizeof(serveraddr));
  serveraddr.sin_family = AF_INET;
  bcopy((const void *)(&ip_addr), (void *)&serveraddr.sin_addr.s_addr, 4);
  serveraddr.sin_port = htons(port);
  if (sock_connect(sockfd, (struct sockaddr*)&serveraddr, sizeof(serveraddr)) < 0){
    return 0;
  }
  _connected = true;
  return 1;
}
int BridgeClient::connect(const char *host, uint16_t port){
  struct hostent *server;
  server = gethostbyname(host);
  if (server == NULL)
    return 0;
  return connect(IPAddress((const uint8_t *)(server->h_addr)), port);
}
int BridgeClient::setSocketOption(int option, char* value, size_t len){
  return setsockopt(sockfd, SOL_SOCKET, option, value, len);
}
int BridgeClient::setTimeout(uint32_t seconds){
  struct timeval tv;
  tv.tv_sec = seconds;
  tv.tv_usec = 0;
  if(setSocketOption(SO_RCVTIMEO, (char *)&tv, sizeof(struct timeval)) < 0)
    return -1;
  return setSocketOption(SO_SNDTIMEO, (char *)&tv, sizeof(struct timeval));
}
int BridgeClient::setOption(int option, int *value){
  return setsockopt(sockfd, IPPROTO_TCP, option, (char *)value, sizeof(int));
}
int BridgeClient::getOption(int option, int *value){
  size_t size = sizeof(int);
  return getsockopt(sockfd, IPPROTO_TCP, option, (char *)value, &size);
}
int BridgeClient::setNoDelay(bool nodelay){
  int flag = nodelay;
  return setOption(TCP_NODELAY, &flag);
}
bool BridgeClient::getNoDelay(){
  int flag = 0;
  getOption(TCP_NODELAY, &flag);
  return flag;
}
size_t BridgeClient::write(uint8_t data){
  return write(&data, 1);
}
size_t BridgeClient::write(const uint8_t *buf, size_t size){
  if(!_connected) return 0;
  int res = send(sockfd, (void*)buf, size, MSG_DONTWAIT | MSG_NOSIGNAL);
  if(res < 0){
    _connected = false;
    res = 0;
  }
  return res;
}
int BridgeClient::read(){
  uint8_t data = 0;
  int res = read(&data, 1);
  if(res < 0){
    return res;
  }
  return data;
}
int BridgeClient::read(uint8_t *buf, size_t size){
  if(!_connected) return -1;
  int res = sock_read(sockfd, 0, 0);
  if(size && res == 0 && available()){
     res = recv(sockfd, buf, size, MSG_DONTWAIT | MSG_NOSIGNAL);
  }
  if(res < 0){
    _connected = false;
  }
  return res;
}
int BridgeClient::available(){
  int count;
  ioctl(sockfd, FIONREAD, &count);
  return count;
}
void BridgeClient::stop(){
  if(sockfd > 0)
    close(sockfd);
  _connected = false;
}
uint8_t BridgeClient::connected(){
  if(!_connected)
    return 0;
  read(0,0);
  return _connected;
}

IPAddress BridgeClient::remoteIP(int fd){
  struct sockaddr_storage addr;
  socklen_t len = sizeof addr;
  getpeername(fd, (struct sockaddr*)&addr, &len);
  struct sockaddr_in *s = (struct sockaddr_in *)&addr;
  return IPAddress((uint32_t)(s->sin_addr.s_addr));
}

uint16_t BridgeClient::remotePort(int fd){
  struct sockaddr_storage addr;
  socklen_t len = sizeof addr;
  getpeername(fd, (struct sockaddr*)&addr, &len);
  struct sockaddr_in *s = (struct sockaddr_in *)&addr;
  return ntohs(s->sin_port);
}

IPAddress BridgeClient::remoteIP(){
  return remoteIP(sockfd);
}

uint16_t BridgeClient::remotePort(){
  return remotePort(sockfd);
}

bool BridgeClient::operator==(const BridgeClient& rhs) {
  return sockfd == rhs.sockfd && remotePort(sockfd) == remotePort(rhs.sockfd) && remoteIP(sockfd) == remoteIP(rhs.sockfd);
}
