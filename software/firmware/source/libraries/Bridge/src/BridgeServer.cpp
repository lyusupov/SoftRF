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

#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <netinet/tcp.h>
#include <sys/ioctl.h>
#include <sys/epoll.h>
#include "BridgeServer.h"

#define MAXEVENTS 64

void BridgeServer::cleanup(){
  if(clients == NULL) return;
  BridgeClient *c1 = clients;
  while(c1 != NULL && !c1->connected()){
    BridgeClient *c = c1;
    c1 = c1->next;
    delete c;
  }
  clients = c1;
  if(clients == NULL) return;
  BridgeClient *c2 = clients->next;
  while(c2 != NULL){
    if(!c2->connected()){
      BridgeClient *c = c2;
      c2 = c2->next;
      c1->next = c2;
      delete c;
    } else {
      c1 = c2;
      c2 = c2->next;
    }
  }
}

int BridgeServer::setSocketOption(int option, char* value, size_t len){
  return setsockopt(sockfd, SOL_SOCKET, option, value, len);
}

int BridgeServer::setTimeout(uint32_t seconds){
  struct timeval tv;
  tv.tv_sec = seconds;
  tv.tv_usec = 0;
  if(setSocketOption(SO_RCVTIMEO, (char *)&tv, sizeof(struct timeval)) < 0)
    return -1;
  return setSocketOption(SO_SNDTIMEO, (char *)&tv, sizeof(struct timeval));
}

size_t BridgeServer::write(const uint8_t *data, size_t len){
  cleanup();
  BridgeClient *c = clients;
  while(c != NULL){
    c->write(data, len);
    c = c->next;
  }
  return len;
}

void BridgeServer::stopAll(){
  while(clients != NULL){
    BridgeClient *d = clients;
    clients = clients->next;
    d->stop();
    delete d;
  }
}

BridgeClient BridgeServer::available(){
  if(!_listening)
    return BridgeClient();

  int n, i;
  n = epoll_wait(pollfd, events, MAXEVENTS, 10);
  for (i = 0; i < n; i++){
    if ((events[i].events & EPOLLERR) || (events[i].events & EPOLLHUP) || (!(events[i].events & EPOLLIN))){
      /* An error has occured on this fd, or the socket is not ready for reading (why were we notified then?) */
      //Serial.printf("epoll error on fd: %d\n", events[i].data.fd);
      BridgeClient *c = clientByFd(events[i].data.fd);
      if(c != NULL && !c->available()){
        c->stop();
      }
    }
    else if (sockfd == events[i].data.fd){
      /* We have a notification on the listening socket, which means one or more incoming connections. */
      //Serial.printf("incomming connection on fd: %d\n", events[i].data.fd);
      struct sockaddr_in _client;
      int cs = sizeof(struct sockaddr_in);
      int client_sock = accept4(sockfd, (struct sockaddr *)&_client, (socklen_t*)&cs, SOCK_NONBLOCK);
      if(client_sock){
        struct epoll_event event;
        event.data.fd = client_sock;
        event.events = EPOLLIN | EPOLLET;
        epoll_ctl(pollfd, EPOLL_CTL_ADD, client_sock, &event);
        
        BridgeClient *client = new BridgeClient(client_sock);
        int val = 1;
        client->setSocketOption(SO_KEEPALIVE, (char*)&val, sizeof(int));
        if(clients == NULL){
          clients = client;
        } else {
          BridgeClient *c = clients;
          while(c->next != NULL) c = c->next;
          c->next = client;
        }
      }
    }
    else {
      //Serial.printf("incomming data on fd: %d\n", events[i].data.fd);
      //if read() returns 0 then we are disconnected
      /* We have data on the fd waiting to be read. Read and
         display it. We must read whatever data is available
         completely, as we are running in edge-triggered mode
         and won't get a notification again for the same
         data. */
      BridgeClient *c = clientByFd(events[i].data.fd);
      if(c != NULL && !c->available()){
        c->stop();
      }
    }
  }
  cleanup();

  BridgeClient *c = clients;
  while(c != NULL){
    if(c->available()){
      return *c;
    }
    c = c->next;
  }
  return BridgeClient();
}

void BridgeServer::begin(){
  if(_listening)
    return;
  struct sockaddr_in server;
  sockfd = socket(AF_INET , SOCK_STREAM | SOCK_NONBLOCK, 0);
  if (sockfd < 0)
    return;
  server.sin_family = AF_INET;
  server.sin_addr.s_addr = INADDR_ANY;
  server.sin_port = htons(_port);
  if(bind(sockfd, (struct sockaddr *)&server, sizeof(server)) < 0)
    return;
  if(listen(sockfd , _max_clients) < 0)
    return;

  pollfd = epoll_create1(0);
  if (pollfd == -1)
    return;

  struct epoll_event event;
  event.data.fd = sockfd;
  event.events = EPOLLIN | EPOLLET;
  if (epoll_ctl(pollfd, EPOLL_CTL_ADD, sockfd, &event) == -1)
    return;

  events = (epoll_event*)calloc(MAXEVENTS, sizeof event);
  if(events == NULL)
    return;
  _listening = true;
}

void BridgeServer::end(){
  stopAll();
  close(sockfd);
  sockfd = -1;
  _listening = false;
}
