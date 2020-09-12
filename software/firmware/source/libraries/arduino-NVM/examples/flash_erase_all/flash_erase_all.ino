/*
 * This skech erases all content of your controller. Use it only, when
 * you are able to install all firemware parts via your programmer.
 *
 * There is no chance to to updates over the air or via serial port
 * after erasing the Flash!
 */
 
#include <Flash.h>

void setup() {
  Serial.begin(9600);
}

char num=0;
char yes=10;
void loop() {
  if ((millis()%5000)<100) {
    Serial.println();
    Serial.println("**************************************************");
    Serial.println("* This sketch deletes all Flash memory including *");
    Serial.println("* this sketch, bootloaders and SoftDevices!!!    *");
    Serial.println("* ---------------------------------------------- *");
    Serial.println("* After erasing the Flash, you need to flash     *");
    Serial.println("* a new Sketch with a programmer like J-Link,    *");
    Serial.println("* ST-Link v2 or CMSIS-DAP. Other methods dosn't  *");
    Serial.println("* work!                                          *");
    Serial.println("* ---------------------------------------------- *");
    Serial.println("* If you are shure what you are doing, send a    *");
    Serial.println("* 'Y' character to erase the nRF CPU completely. *");
    Serial.println("* ---------------------------------------------- *");
    Serial.println("*          You may brick your device!            *");
    Serial.println("**************************************************");
    num=0;
  } else {
    if (num++<49) {
      Serial.print('.');
    } else {
      Serial.println('.');
      num=0;
    }
  }
  if (Serial.available() > 0) {
    if (Serial.read() == 'Y') {
      if (yes>0) {
        Serial.print("\r\nYou may brick your device. Please give me ");
        Serial.print(yes, DEC);
        Serial.println(" additional 'Y' characters.");
        yes--;
      } else {
        Serial.println("\r\nYou have been warned! Erase flash. Goodbye.");
        Flash.erase_all();
      }
    }
  }
  delay(50);
}
