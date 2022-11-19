#include "ADXL362.h"

//TODO: set the correct pin for your setup
#define SPI_SLAVE_SELECT_PIN 7

#define SPI_INT1_PIN 8  
#define SPI_INT2_PIN 9

#define FIFO_SIZE 200
#define FIFO_TEMP_ENABLED true

//After this fifo overrun interrupts will occur as data is not read from the FIFO buffer
#define FIFO_MAX_READS 10

ADXL362 accelerometer(SPI_SLAVE_SELECT_PIN);

//TODO: comment out #define ADXL362_DEBUG at top of library .h file after code tested and working
void setup() {
  //TODO: adjust baudrate to your serial monitor setting or adjust it in the serial monitor
  Serial.begin(4800);
  while (!Serial);  

  check(accelerometer.init());
  accelerometer.printRegisters();

  Serial.print(F("\r\nCHIP REVISION : "));
  Serial.println(accelerometer.getRevisionId());

  setupInterrupt(SPI_INT1_PIN, handleWatermark, RISING);
  setupInterrupt(SPI_INT2_PIN, handleOverrun, RISING);

  //Here ad_fifo_stream can also be used, no overrun interrupt will occur as data in FIFO as always replaced with newest data
  check(accelerometer.configureFIFO(ad_fifo_oldestsaved,FIFO_SIZE, FIFO_TEMP_ENABLED));
  check(accelerometer.configureFIFOInterrupt1(ad_status_fifo_watermark));
  check(accelerometer.configureFIFOInterrupt2(ad_status_fifo_overrun));
  
  check(accelerometer.activateMeasure());
  accelerometer.printRegisters();
}

bool intHandleWatermark = false;
bool intHandleOverrun = false;

void handleWatermark() {
  intHandleWatermark = true;
}

void handleOverrun() {
  intHandleOverrun = true;
}

int nrOfWatermarksHandled=0;
void loop() {
  //intentional overrun will occur as fifo just read # FIFO_MAX_READS times
  checkInterrupt(&intHandleOverrun,"FIFO OVERRUN: fifo buffer full/data missed", accelerometer);
  
  bool watermarkreached = checkInterrupt(&intHandleWatermark,"FIFO WATERMARK", accelerometer);

  if (nrOfWatermarksHandled <= FIFO_MAX_READS && watermarkreached) {
    Serial.print(F("READING FIFO DATA: "));
    uint16_t fifoEntries[FIFO_SIZE];
    uint16_t nrOfEntries = accelerometer.readFIFO(fifoEntries,FIFO_SIZE);
    Serial.print(F("#Entries read from fifo:"));
    Serial.println(nrOfEntries);

    uint16_t* entriesptr = fifoEntries;
    uint16_t buflen = nrOfEntries;
    while (buflen > 0) {
      FifoMeasurement m = accelerometer.parseFIFOMeasurement(ad_range_2G, &entriesptr, &buflen, FIFO_TEMP_ENABLED);
      printmeasurement(m.forceInMg);
      Serial.print(F(" (12bit) temp : "));
      Serial.print(m.tempInC);
      
      Serial.println("");
    }
    watermarkreached = false;
    nrOfWatermarksHandled++;
  }
}


/********************************* supporting / non example helper code  *************************************/

void check(short code) {
  if (code <= 0) {
    Serial.print(F("\r\n********** ERROR: "));
    Serial.print(code);
    Serial.println(F(" **********"));
    if (code == -110)
      Serial.println(F("Device not connected? Otherwise check noisy power supply. Or wait a while till capacitors are empty: Softreset failed at first check for success"));
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


bool checkInterrupt(bool* intstate, char* text, ADXL362 accelerometerelerometer) {
  bool intoccured = false;
  if (*intstate) {
    Serial.print(F("**INTERRUPT : "));
    Serial.println(text);

    Serial.print(F(" overrun "));
    Serial.println(accelerometer.hasStatus(ad_status_fifo_overrun));
    intoccured = true;
    *intstate = false;
  }
  return intoccured;
}

void setupInterrupt(byte pinNr, void (*handlerfunction)(), int mode) {
  pinMode(pinNr, INPUT_PULLUP);
  //this is why no pullup resistor needed for interrupt
  attachInterrupt(digitalPinToInterrupt(pinNr), handlerfunction, mode);
}
