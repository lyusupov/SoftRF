/*
  EasyLinkTx
  
  A basic EasyLink Transmit example..
  Read the analog value from A0 (pin 2) and copy the value into the tx packet.
  This Sketch is the counterpart of teh EasyLinkRx example.
  
  Hardware Required:
  * CC1310 LaunchPad
  
  This example code is in the public domain.
*/

#include "EasyLink.h"

EasyLink_TxPacket txPacket;

EasyLink myLink;

void setup() {
  Serial.begin(115200);

  // begin defaults to EasyLink_Phy_50kbps2gfsk
  myLink.begin();
  Serial.println(myLink.version());

  // Set the destination address to 0xaa
  txPacket.dstAddr[0] = 0xaa;
}

void loop() {
  uint16_t value = analogRead(A0);

  // Copy the analog value into the txPacket payload
  memcpy(&txPacket.payload, &value, sizeof(uint16_t));

  // Set the length of the packet
  txPacket.len = sizeof(uint16_t);
  // Transmit immediately
  txPacket.absTime = EasyLink_ms_To_RadioTime(0);

  EasyLink_Status status = myLink.transmit(&txPacket);

  if(status == EasyLink_Status_Success) {
    Serial.println("Packet transmitted successfully");
  } else {
    Serial.print("Transmit failed with status code: ");
    Serial.print(status);
    Serial.print(" (");
    Serial.print(myLink.getStatusString(status));
    Serial.println(")");
  }

  delay(1000);
}
