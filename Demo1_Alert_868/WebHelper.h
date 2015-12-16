
#include "Demo1_Alert_868.h"

void Web_setup(void);
void Web_loop(void);

void Hex2Bin(String, byte *);
String Bin2Hex(byte *);

extern ufo_t fo;
extern uint32_t rx_packets_counter;
extern tx_state txready;
extern byte TxBuffer[PKT_SIZE];
extern String TxDataTemplate;
extern WiFiClient client;
extern char UDPpacketBuffer[512];
