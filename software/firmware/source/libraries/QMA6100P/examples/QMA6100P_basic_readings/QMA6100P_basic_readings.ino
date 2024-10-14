/*example1-BasicReadings*/
#include <Wire.h>
#include <QMA6100P.h>

#define USB_TX_PIN PA12 // D- pin
#define USB_RX_PIN PA11 // D+ pin
#define I2C_SDA_PIN PB11
#define I2C_SCL_PIN PB10
#define ACCEL_INT1 PB5
#define ACCEL_INT2 PB6
#define DB_LED_PIN PA15

bool buffer_enable = false;

QMA6100P qmaAccel;

outputData myData; // Struct for the accelerometer's data

#include <SoftwareSerial.h>

SoftwareSerial softSerial(USB_RX_PIN, USB_TX_PIN);

void setup()
{
  softSerial.begin(38400);
  delay(2000);
  softSerial.println("serial start");

  // put your setup code here, to run once:
  pinMode(DB_LED_PIN, OUTPUT);

  // Configure I2C
  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  Wire.begin();

  if (!qmaAccel.begin())
  {
    softSerial.println("ERROR: Could not communicate with the the QMA6100P. Freezing.");
    while (1)
      ;
  }

  if (!qmaAccel.softwareReset())
    softSerial.println("ERROR: Failed to reset");
    
    delay(5);

  if(!qmaAccel.setRange(SFE_QMA6100P_RANGE32G)){      // 32g for the QMA6100P
    softSerial.println("ERROR: failed to set range");
  }

  if(!qmaAccel.enableAccel()){
    softSerial.println("ERROR: failed to set active mode");
  }   

  if(!qmaAccel.calibrateOffsets()){
    softSerial.println("ERROR: calibration failed");
  }

  //qmaAccel.setOffset( );

  myData.xData = 0;
  myData.yData = 0;
  myData.zData = 0;
  softSerial.println("Ready.");

}

void loop()
{

  // Check if data is ready.
  qmaAccel.getAccelData(&myData);
  qmaAccel.offsetValues(myData.xData, myData.yData, myData.zData);
  softSerial.print("X: ");
  softSerial.print(myData.xData, 2);
  softSerial.print(" Y: ");
  softSerial.print(myData.yData, 2);
  softSerial.print(" Z: ");
  softSerial.print(myData.zData, 2);
  softSerial.println();
  

  delay(20); // Delay should be 1/ODR (Output Data Rate), default is 1/50ODR
}
