/*
 * A simple electronic die.
 *
 * Press the reset button to throw a set of dice.
 *
 */

#include "ESP8266TrueRandom.h"

void setup() {
  Serial.begin(9600);
  Serial.println("Throwing...");
  delay(1000);
  
  Serial.print("6 sided die: ");
  Serial.println(ESP8266TrueRandom.random(1,7));
  Serial.print("4 sided die: ");
  Serial.println(ESP8266TrueRandom.random(1,5));
  Serial.print("8 sided die: ");
  Serial.println(ESP8266TrueRandom.random(1,9));
  Serial.print("10 sided die: ");
  Serial.println(ESP8266TrueRandom.random(1,11));
  Serial.print("12 sided die: ");
  Serial.println(ESP8266TrueRandom.random(1,13));
  Serial.print("20 sided die: ");
  Serial.println(ESP8266TrueRandom.random(1,21));
  Serial.print("100 sided die: ");
  Serial.println(ESP8266TrueRandom.random(1,101));
}

void loop() {
  ; // Do nothing
}
