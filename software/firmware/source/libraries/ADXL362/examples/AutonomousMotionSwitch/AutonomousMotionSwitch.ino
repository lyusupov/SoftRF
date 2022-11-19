#include "ADXL362.h"

//TODO: set the correct pins for your setup
#define SPI_SLAVE_SELECT_PIN 7

#define SPI_INT2_PIN 9
  
ADXL362 acc(SPI_SLAVE_SELECT_PIN);

//TODO: comment out #define ADXL362_DEBUG at top of library .h file after code tested and working
void setup() {
  //TODO: adjust baud rate to your serial monitor setting
  Serial.begin(4800);
  while (!Serial);  

  check(acc.init());
  acc.printRegisters();

  Serial.print(F("\r\nCHIP REVISION : "));
  Serial.println(acc.getRevisionId());  

  //to be able to see that int2 stays high while awake: add a led and (+-) 400ohm resistor in series between earth and the int2 pin (+) of the accelerometer
  setupInterrupt(SPI_INT2_PIN, handleAwake, RISING);
  check(acc.activateAutonomousMotionSwitch(150,250,2000,false,false));
  acc.printRegisters(true);
}

bool awakeInt = false;

void handleAwake() {
   awakeInt = true;
}


void loop() {
  checkInterrupt(&awakeInt,"AWAKE", acc);
}

void check(short code) {
  if (code <= 0) {
    Serial.print(F("\r\n********** ERROR: "));
    Serial.print(code);
    Serial.println(F(" **********"));
    if (code == -110)
      Serial.println(F("Device not connected? Otherwise check noisy power supply. Or wait a while till capacitors are empty: softreset failed at first check for success"));
    else if (code >= -104 && code <= -102)
      Serial.println(F("Check power supply (decoupling, wire length). Wrong registry values read after reset/checkdevice."));
    delay(3000);
  } else {
    Serial.println(F("----------------------------------------"));
  }
}
void printmeasurement(MeasurementInMg m) {
  Serial.print(F(" x : "));
  Serial.print(m.x / 1000.0);
  Serial.print(F(" y : "));
  Serial.print(m.y / 1000.0);
  Serial.print(F(" z : "));
  Serial.print(m.z / 1000.0);
}

void checkInterrupt(bool* intstate, char* text, ADXL362 accelerometer) {
  if (*intstate) {
    Serial.print(F("**INTERRUPT : "));
    Serial.println(text);

    Serial.print(F("awake "));
    Serial.println(accelerometer.isAwake(ad_seq_loop));
    
    *intstate = false;
  }
}

void setupInterrupt(byte pinNr, void (*handlerfunction)(), int mode) {
  pinMode(pinNr, INPUT_PULLUP);
  //this is why no pullup resistor needed for interrupt
  attachInterrupt(digitalPinToInterrupt(pinNr), handlerfunction, mode);
}
