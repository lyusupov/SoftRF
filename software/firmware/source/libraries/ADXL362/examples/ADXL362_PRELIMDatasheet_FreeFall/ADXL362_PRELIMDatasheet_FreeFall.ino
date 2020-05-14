/*
 ADXL362_Datasheet_FreeFall.ino -  Free fall 
 example from datasheet for Analog Devices ADXL362 - Micropower 
 3-axis accelerometer
 go to http://www.analog.com/ADXL362 for datasheet
 
 
 License: CC BY-SA 3.0: Creative Commons Share-alike 3.0. Feel free 
 to use and abuse this code however you'd like. If you find it useful
 please attribute, and SHARE-ALIKE!
 
 Created June 2012
 by Anne Mahaffey - hosted on http://annem.github.com/ADXL362

Connect SCLK, MISO, MOSI, and CSB of ADXL362 to
SCLK, MISO, MOSI, and DP 10 of Arduino 
(check http://arduino.cc/en/Reference/SPI for details)
 
*/ 

#include <SPI.h>
#include <ADXL362.h>


ADXL362 xl;

const int16_t slaveSelectPin = 10;

int16_t temp;
int16_t XValue, YValue, ZValue, Temperature;


void setup(){
    Serial.begin(9600);
    pinMode(2, INPUT);  // used to check ADXL362 INT2
    pinMode(slaveSelectPin, OUTPUT);
    SPI.begin();
    SPI.setDataMode(SPI_MODE0);	//CPHA = CPOL = 0    MODE = 0
    delay(1000);
    

    // Free fall threshold   
    digitalWrite(slaveSelectPin, LOW);
    SPI.transfer(0x0A);  //  write
    SPI.transfer(0x23);  //  Reg 23
    SPI.transfer(0x58);  //  Reg 23 = 0x58  
    digitalWrite(slaveSelectPin, HIGH);
    delay(100);
    
    // Free fall time
    digitalWrite(slaveSelectPin, LOW);
    SPI.transfer(0x0A);  //  write
    SPI.transfer(0x25);  //  Reg 25
    SPI.transfer(0x03);  //  Reg 25 = 0x03 = 30ms  
    digitalWrite(slaveSelectPin, HIGH);
    delay(100);
    
    // absolute inactivity detection
    digitalWrite(slaveSelectPin, LOW);
    SPI.transfer(0x0A);  //  write
    SPI.transfer(0x27);  //  Reg 27
    SPI.transfer(0x0C);   
    digitalWrite(slaveSelectPin, HIGH);
    delay(100);
    
    // map inactivity intterupt to Int2
    digitalWrite(slaveSelectPin, LOW);
    SPI.transfer(0x0A);  //  write
    SPI.transfer(0x2B);  //  Reg 2B
    SPI.transfer(0x20);
    digitalWrite(slaveSelectPin, HIGH);
    delay(100);
    
    // map inactivity intterupt to Int2
    digitalWrite(slaveSelectPin, LOW);
    SPI.transfer(0x0A);  //  write
    SPI.transfer(0x2C);  //  Reg 2C
    SPI.transfer(0x83);  //  Reg 2C = 0x83 = 8g, 100Hz ODR
    digitalWrite(slaveSelectPin, HIGH);
    delay(100);
    
    // begin measurement
    digitalWrite(slaveSelectPin, LOW); 
    SPI.transfer(0x0A);  //  write
    SPI.transfer(0x2D);  //  Reg 2D
    SPI.transfer(0x0A);  //  begin meanusre   
    digitalWrite(slaveSelectPin, HIGH);
    delay(100);
    
    Serial.println("initial setup done");
    
    
}

void loop(){
    
    // read all three axis in burst to ensure all measurements correspond to same sample time
    xl.readXYZTData(XValue, YValue, ZValue, Temperature);  
    temp = digitalRead(2);
    Serial.print("Int2 is ");
    Serial.print(temp);
    Serial.print("\t");	 
    delay(10);                // Arbitrary delay to make serial monitor easier to observe
}

