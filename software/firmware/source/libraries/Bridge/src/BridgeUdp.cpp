#include <BridgeUdp.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netdb.h>

uint8_t BridgeUDP::begin(uint16_t port){
  if(udp_server)
    stop();
  if ((udp_server=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1){
    return 0;
  }
  struct sockaddr_in addr;
  memset((char *) &addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  if(bind(udp_server , (struct sockaddr*)&addr, sizeof(addr)) == -1){
    udp_server = -1;
    return 0;
  }
  return 1;
}

void BridgeUDP::stop(){
  if(udp_server == -1)
    return;
  close(udp_server);
  udp_server = -1;
}

int BridgeUDP::beginPacket(){
  if(!remote_port)
    return 0;
  buffer_len = 0;
  bzero(buffer, 1500);
  return 1;
}

int BridgeUDP::beginPacket(IPAddress ip, uint16_t port){
  remote_ip = ip;
  remote_port = port;
  return beginPacket();
}

int BridgeUDP::beginPacket(const char *host, uint16_t port){
  struct hostent *server;
  server = gethostbyname(host);
  if (server == NULL)
    return 0;
  return beginPacket(IPAddress((const uint8_t *)(server->h_addr_list[0])), port);
}

int BridgeUDP::endPacket(){
  struct sockaddr_in recipient;
  recipient.sin_addr.s_addr = (uint32_t)remote_ip;
  recipient.sin_family = AF_INET;
  recipient.sin_port = htons(remote_port);
  return sendto(udp_server, buffer, buffer_len, 0, (struct sockaddr*) &recipient, sizeof(recipient));
}

size_t BridgeUDP::write(uint8_t data){
  if(buffer_len == 1500){
    endPacket();
    buffer_len = 0;
    bzero(buffer, 1500);
  }
  buffer[buffer_len++] = data;
  return 1;
}

size_t BridgeUDP::write(const uint8_t *buffer, size_t size){
  size_t i;
  for(i=0;i<size;i++)
    write(buffer[i]);
  return i;
}

int BridgeUDP::parsePacket(){
  if(rx_buffer)
    return 0;
  struct sockaddr_in si_other;
  int slen = sizeof(si_other) , len;
  char buf[2048];
  bzero(buf, 2048);
  if ((len = recvfrom(udp_server, buf, 2048, MSG_DONTWAIT, (struct sockaddr *) &si_other, (socklen_t *)&slen)) == -1){
    return 0;
  }
  remote_ip = IPAddress(si_other.sin_addr.s_addr);
  remote_port = ntohs(si_other.sin_port);
  rx_buffer = new cbuf(len);
  rx_buffer->write(buf, len);
  return len;
}

int BridgeUDP::available(){
  if(!rx_buffer) return 0;
  return rx_buffer->getSize();
}

int BridgeUDP::read(){
  if(!rx_buffer) return -1;
  int out = rx_buffer->read();
  if(!rx_buffer->getSize()){
    cbuf *b = rx_buffer;
    rx_buffer = 0;
    delete b;
  }
  return out;
}

int BridgeUDP::read(unsigned char* buffer, size_t len){
  return read((char *)buffer, len);
}

int BridgeUDP::read(char* buffer, size_t len){
  if(!rx_buffer) return 0;
  int out = rx_buffer->read(buffer, len);
  if(!rx_buffer->getSize()){
    cbuf *b = rx_buffer;
    rx_buffer = 0;
    delete b;
  }
  return out;
}

int BridgeUDP::peek(){
  if(!rx_buffer) return -1;
  return rx_buffer->peek();
}

void BridgeUDP::flush(){
  cbuf *b = rx_buffer;
  rx_buffer = 0;
  delete b;
}

IPAddress BridgeUDP::remoteIP(){
  return remote_ip;
}

uint16_t BridgeUDP::remotePort(){
  return remote_port;
}
