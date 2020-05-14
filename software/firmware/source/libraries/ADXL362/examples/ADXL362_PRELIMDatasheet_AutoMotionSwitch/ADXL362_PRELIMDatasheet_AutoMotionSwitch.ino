/*
 ADXL362_Datasheet_AutoMotionSwitch.ino -  Autonomous motion switch 
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
    
    // Activity threshold  
    digitalWrite(slaveSelectPin, LOW);
    SPI.transfer(0x0A);  //  write
    SPI.transfer(0x20);  //  Reg 20
    SPI.transfer(0xFA);  //  Reg 20 = dec 250
    SPI.transfer(0x00);  //  Reg 21 = 0x00    
    digitalWrite(slaveSelectPin, HIGH);
    delay(100);

    // Inactivity threshold, and inactivity timer   
    digitalWrite(slaveSelectPin, LOW);
    SPI.transfer(0x0A);  //  write
    SPI.transfer(0x23);  //  Reg 23
    SPI.transfer(0x96);  //  Reg 23 = dec 150  
    SPI.transfer(0x00);  //  Reg 24 = 0x00
    SPI.transfer(0x1E);  //  Reg 25 = dec 30
    digitalWrite(slaveSelectPin, HIGH);
    delay(100);
    
    // motion detection, activity/inactivity detection
    digitalWrite(slaveSelectPin, LOW);
    SPI.transfer(0x0A);  //  write
    SPI.transfer(0x27);  //  Reg 27
    SPI.transfer(0x3F);   
    digitalWrite(slaveSelectPin, HIGH);
    delay(100);
    
    // map awake bit to Int2
    digitalWrite(slaveSelectPin, LOW);
    SPI.transfer(0x0A);  //  write
    SPI.transfer(0x2B);  //  Reg 2B
    SPI.transfer(0x49);
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
    delay(100);                // Arbitrary delay to make serial monitor easier to observe
}

