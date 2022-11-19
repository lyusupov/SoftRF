#include "ADXL362.h"

//TODO: set the correct pins for your setup
#define SPI_SLAVE_SELECT_PIN 7

#define SPI_INT1_PIN 8
#define SPI_INT2_PIN 9

//not needed when using (sequantial) loop mode, otherwise only 1 active and 1 inactive will occur
#define ACKNOWLEDGE_INTERRUPTS 
  
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

  /* PARAMETERS:  
   bool awake, bool awakePin2Active) .. otherwise int 2 = inactive interrupt
   */
  setupInterrupts(SPI_INT1_PIN, SPI_INT2_PIN, false, false); //just active and inactive interrupt

  /* PARAMETERS:
    short activateCustomDetection(bandwidth bandwidthInHz, sequentialMode smode, bool autoSleep
                , uint16_t minForceInMg, uint32_t actminTimeInMs
              , uint16_t maxForceInMg, uint32_t inActminTimeInMs
              , status int1Status = ad_status_active, status int2Status = ad_status_inactive, bool activeLow = false, bool absoluteMode = false
              , measurementRange measurementRangeInG = ad_range_2G, noiseMode noiseMode = ad_noise_normal);

    ATTENTION: some parameter combinations will result in an error code as they are not possible for the ADXL362 chip: lookup the code in the library .cpp file what the problem is
  */
  check(acc.activateCustomDetection(ad_bandwidth_hz_6_wakeup_ultralowpower, ad_seq_link, false, 100, 0, 150, 4000, ad_status_active, ad_status_inactive, false, false, ad_range_2G, ad_noise_normal));

  acc.printRegisters(true, true);

  Serial.println(F("Waiting 5 seconds before continuing"));
  delay(5000);
}


int lastactack = millis();
int lastINactack = millis();
int lastAwakeack = millis();

int lastactackms = 0;
int lastINactackms = 0;
int lastAwakeackms = 0;

bool activeInt = false;
bool inActiveInt = false;
bool awakeInt = false;

void handleActive() {
   lastactackms = millis() - lastactack;
   lastactack = millis();
   activeInt = true;
}

void handleInactive() {
   lastINactackms = millis() - lastINactack;
   lastINactack = millis();  
  inActiveInt = true;
}

void handleAwake() {
   lastAwakeackms = millis() - lastAwakeack;
   lastAwakeack = millis();  
   awakeInt = true;
}


void loop() {
  checkInterrupt(&activeInt,"ACTIVE", acc);
  checkInterrupt(&inActiveInt,"INACTIVE", acc);
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

    #ifdef ACKNOWLEDGE_INTERRUPTS
    Serial.print(F("active "));
    Serial.print(accelerometer.isActInterrupt());
    Serial.print(F(" inactive "));
    Serial.print(accelerometer.isInactInterrupt());
    Serial.print(F(" awake "));
    Serial.println(accelerometer.isAwake(ad_seq_loop)); //TODO: adjust when i know exactly when awake bit/interrupt is used

    Serial.print(F("actack "));
    Serial.print(lastactackms);
    lastactack = millis();
    Serial.print(F(" inactack "));
    Serial.print(lastINactackms);
    lastINactack = millis();
    Serial.print(F(" awakeack "));
    Serial.println(lastAwakeackms);
    lastAwakeack = millis();
    #endif
    
    MeasurementInMg xyz = acc.getXYZ(ad_range_2G);
    printmeasurement(xyz);
    Serial.println("");

    *intstate = false;
  }
}

//arduino zero: all except pin 4
void setupInterrupts(byte int1pin, byte int2pin, bool awake, bool awakePin2Active) {
  if (!awake) {
    Serial.println(F("Setting up interrupts active (pin1) and inactive (pin2)"));
    setupInterrupt(int1pin, handleActive, RISING);
    setupInterrupt(int2pin, handleInactive, RISING);    
  } else {
    Serial.print(F("Setting up interrupt awake"));
    setupInterrupt(int1pin, handleAwake, RISING); //HIGH: hang, fires endlessly

    if (awakePin2Active) {
      Serial.println(F(" + active"));
      setupInterrupt(int2pin, handleActive, RISING);        
    } else {
      Serial.println(F(" + INactive"));
      setupInterrupt(int2pin, handleInactive, RISING);        
    }
  }
}

void setupInterrupt(byte pinNr, void (*handlerfunction)(), int mode) {
  pinMode(pinNr, INPUT_PULLUP);
  //this is why no pullup resistor needed for interrupt
  attachInterrupt(digitalPinToInterrupt(pinNr), handlerfunction, mode);
}
