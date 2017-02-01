#include "ESP8266TrueRandom.h"

unsigned long startTime;
int i;

void setup() {
  Serial.begin(9600);
  
  Serial.println("ESP8266TrueRandom benchmark");
  Serial.println("--------------------");
  Serial.println();

  Serial.print("Arduino clock speed: ");
  Serial.print(F_CPU/1000000);
  Serial.println("MHz");

  Serial.print("randomBit(): ");
  startTime = millis();
  ESP8266TrueRandom.randomBit();
  Serial.print(millis() - startTime);
  Serial.println("ms");
  
  Serial.print("randomByte(): ");
  startTime = millis();
  ESP8266TrueRandom.randomByte();
  Serial.print(millis() - startTime);
  Serial.println("ms");
  
  Serial.print("random(100): ");
  startTime = millis();
  ESP8266TrueRandom.random(100);
  Serial.print(millis() - startTime);
  Serial.println("ms");
  
  Serial.print("random(65536): ");
  startTime = millis();
  ESP8266TrueRandom.random(65536);
  Serial.print(millis() - startTime);
  Serial.println("ms");
  
  Serial.print("random(65537): ");
  startTime = millis();
  ESP8266TrueRandom.random(65537);
  Serial.print(millis() - startTime);
  Serial.println("ms");
}
void loop() {
}
