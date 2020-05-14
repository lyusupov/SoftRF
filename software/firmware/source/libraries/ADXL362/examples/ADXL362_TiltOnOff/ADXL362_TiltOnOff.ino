/*
FlipOnOffExample for Analog Devices ADXL362 - Micropower 3-axis accelerometer
go to http://www.analog.com/ADXL362 for datasheet

Arduino will "go to sleep" when circuit is laid down flat on table.
Will "wake up" when circuit is tilted slightly.


License: CC BY-SA 3.0: Creative Commons Share-alike 3.0. Feel free 
to use and abuse this code however you'd like. If you find it useful
please attribute, and SHARE-ALIKE!

Created June 2012
by Anne Mahaffey - hosted on http://annem.github.com/ADXL362

Connect SCLK, MISO, MOSI, and CSB of ADXL362 to
SCLK, MISO, MOSI, and DP 10 of Arduino 
( See http://arduino.cc/en/Reference/SPI for details)

Connect INT1 of ADXL362 to DP2 of Arduino
( See http://arduino.cc/en/Reference/AttachInterrupt for details)

*/ 


// Download LowPower library from http://github.com/rocketscream/Low-Power
#include <LowPower.h>

#include <SPI.h>
#include <ADXL362.h>

ADXL362 xl;

//  Setup interrupt on Arduino
//  See interrupt example at http://arduino.cc/en/Reference/AttachInterrupt
//
int16_t interruptPin = 2;          //Setup ADXL362 interrupt output to Interrupt 0 (digital pin 2)
int16_t interruptStatus = 0;

int16_t XValue, YValue, ZValue, Temperature;




void setup(){

    // Startup, soft reset
    Serial.begin(9600);
    xl.begin();                //soft reset
    delay(1000);    
    
	
    // Setup digital pin 7 for LED observation of awake/asleep  
    pinMode(7, OUTPUT);    
    digitalWrite(7, HIGH);
    
   
    //  Setup Activity and Inactivity thresholds
    xl.setupDCActivityInterrupt(750, 100);
    xl.setupDCInactivityInterrupt(700, 100);
	Serial.println();
    
	
    //
    // Setup ADXL362 for proper autosleep mode
    //
	
    // Map Awake status to Interrupt 1
    // *** create a function to map interrupts... coming soon
    xl.SPIwriteOneRegister(0x2A, 0x40);   
	
    // Setup Activity/Inactivity register
    xl.SPIwriteOneRegister(0x27, 0x35); // Absolute Activity, Absolute Inactivity, Loop Mode  
        
    // turn on Autosleep bit
    byte POWER_CTL_reg = xl.SPIreadOneRegister(0x2D);
    POWER_CTL_reg = POWER_CTL_reg | (0x04);				// turn on POWER_CTL[2] - Autosleep bit
    xl.SPIwriteOneRegister(0x2D, POWER_CTL_reg);
    
	



 
    //
    // turn on Measure mode
    //
    xl.beginMeasure();                      // DO LAST! enable measurement mode   
    xl.checkAllControlRegs();               // check some setup conditions    
    delay(100);
 

 
    //
    // Setup interrupt function on Arduino
    //    IMPORTANT - Do this last in the setup, after you have fully configured ADXL.  
    //    You don't want the Arduino to go to sleep before you're done with setup
    //
    pinMode(2, INPUT);    
    attachInterrupt(0, interruptFunction, RISING);  // A high on output of ADXL interrupt means ADXL is awake, and wake up Arduino 
}



void loop(){
  //
  //  Check ADXL362 interrupt status to determine if it's asleep
  //
  interruptStatus = digitalRead(interruptPin);

// if ADXL362 is asleep, call LowPower.powerdown  
  if(interruptStatus == 0) { 
    Serial.print("\nADXL went to sleep - Put Arduino to sleep now \n");
    digitalWrite(7, LOW);    // Turn off LED as visual indicator of asleep
    delay(100);
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);     
  }
  
// if ADXL362 is awake, report XYZT data to Serial Monitor
  else{
    delay(10);
    digitalWrite(7, HIGH);    // Turn on LED as visual indicator of awake
    xl.readXYZTData(XValue, YValue, ZValue, Temperature);  	     
  }
  // give circuit time to settle after wakeup
  delay(20);
}

//
// Function called if Arduino detects interrupt activity
//    when rising edge detected on Arduino interrupt
//
void interruptFunction(){
  Serial.println("\nArduino is Awake! \n");
}