#ifndef pi_udp_h
#define pi_udp_h

#if defined(ARDUINO)
#include <Arduino.h>
#include <Udp.h>
#include <cbuf.h>
#endif /* ARDUINO */

#if defined(RASPBERRY_PI) || defined(LUCKFOX_LYRA)
#include <raspi/raspi.h>
#include <raspi/Udp.h>
#include <raspi/cbuf.h>
#endif /* RASPBERRY_PI */

class BridgeUDP : public UDP {
private:
  int udp_server;
  IPAddress multicast_ip;
  IPAddress remote_ip;
  uint16_t remote_port;
  char buffer[1460];
  size_t buffer_len;
  cbuf * rx_buffer;
public:
  BridgeUDP():udp_server(-1), remote_port(0), buffer_len(0), rx_buffer(0){}
  ~BridgeUDP(){
    stop();
    if(rx_buffer)
      delete rx_buffer;
  }
  virtual uint8_t begin(uint16_t);
  virtual uint8_t beginMulticast(IPAddress a, uint16_t p){
    multicast_ip = a;
    return begin(p);
  }
  virtual void stop();
  virtual int beginPacket();
  virtual int beginPacket(IPAddress ip, uint16_t port);
  virtual int beginPacket(const char *host, uint16_t port);
  virtual int endPacket();
  virtual size_t write(uint8_t);
  virtual size_t write(const uint8_t *buffer, size_t size);
  virtual int parsePacket();
  virtual int available();
  virtual int read();
  virtual int read(unsigned char* buffer, size_t len);
  virtual int read(char* buffer, size_t len);
  virtual int peek();
  virtual void flush();
  virtual IPAddress remoteIP();
  virtual uint16_t remotePort();
};

#endif
