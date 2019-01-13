/*
  EasyLinkRx
  
  A basic EasyLink Receive example..
  Receive a packet and print the 16-bit value stored in the payload.
  This Sketch is the counterpart of teh EasyLinkTx example.
  
  Hardware Required:
  * CC1310 LaunchPad
  
  This example code is in the public domain.
*/

#include "EasyLink.h"

EasyLink_RxPacket rxPacket;
EasyLink myLink;

void setup() {
  Serial.begin(115200);
  // begin defaults to EasyLink_Phy_50kbps2gfsk
  myLink.begin();
  Serial.println(myLink.version());
}

uint16_t value;
void loop() {
  // Wait / block for 2 second to receive a packet.
  // rxTimeout is in Radio time and needs to be converted from miliseconds to Radio Time
  rxPacket.rxTimeout = EasyLink_ms_To_RadioTime(2000);
  // Turn the receiver on immediately
  rxPacket.absTime = EasyLink_ms_To_RadioTime(0);

  EasyLink_Status status = myLink.receive(&rxPacket);
  
  if (status == EasyLink_Status_Success) {
    memcpy(&value, &rxPacket.payload, sizeof(uint16_t));
    Serial.print("Packet received with lenght ");
    Serial.print(rxPacket.len);
    Serial.print(" and value ");
    Serial.println(value);
  } else {

    Serial.print("Error receiving packet with status code: ");
    Serial.print(status);
    Serial.print(" (");
    Serial.print(myLink.getStatusString(status));
    Serial.println(")");
  }
}
