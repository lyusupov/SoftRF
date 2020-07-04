#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include "bcm2835_registers.h"

#define NOT_A_PIN 0
#define NOT_A_PORT 0
#define NOT_AN_INTERRUPT -1

#define EXTERNAL_NUM_INTERRUPTS     32
#define NUM_DIGITAL_PINS            32
#define NUM_ANALOG_INPUTS           0

#define analogInputToDigitalPin(p)  (-1)
#define analogInPinToBit(p)         (p)

#define digitalPinToPort(p)         (((p)<32)?0:1)
#define digitalPinToBitMask(p)      (1 << p)
#define portOutputRegister(p)       ((volatile uint32_t *)((p)?&GPSET1:&GPSET0))
#define portModeRegister(p)         ((volatile uint32_t *)((p)?&GPSET1:&GPSET0))

#define digitalPinHasPWM(p)         ((p) == 12 || (p) == 13 || (p) == 18 || (p) == 19)
#define digitalPinToInterrupt(p)    (p)
#define digitalPinToTimer(p)        ((p==12||p==18)?0:(p==13||p==19)?1:-1)

//*portModeRegister(digitalPinToPort(ETHERNET_SHIELD_SPI_CS)) |= digitalPinToBitMask(ETHERNET_SHIELD_SPI_CS);

// A/B - 17 GPIO | A+/B+/2/Zero - 28 GPIO
//                                    RX, TX, D2, D3, D4, D5, D6, D7, D8, D9, D10, D11, D12, D13, A0, A1, A2, A3, A4, A5, SDA, SCL, SCK, MISO, MOSI, RST
//const uint8_t piToArduinoB[26] =  { 15, 14, 4,  18, 7,  22, 23, 24, 25, 17, 8,   10,  9,   11,  28, 29, 30, 31, 2,  3,  2,   3,   11,  9,    10,   27  };
//const uint8_t piToArduinoBP[26] = { 15, 14, 4,  18, 7,  22, 23, 24, 25, 17, 8,   12,  16,  13,  5,  19, 20, 21, 6,  26, 2,   3,   11,  9,    10,   27  };

static const uint8_t ID_SD = 0; //pin 27 //B+
static const uint8_t ID_SC = 1; //pin 28 //B+
static const uint8_t SDA   = 2; //pin 3
static const uint8_t SCL   = 3; //pin 5
//static const uint8_t D4  = 4; //pin 7
//static const uint8_t D5  = 5; //pin 29 //B+
//static const uint8_t D6  = 6; //pin 31 //B+
static const uint8_t SS1   = 7; //pin 26
static const uint8_t SS    = 8; //pin 24
static const uint8_t MISO  = 9; //pin 21
static const uint8_t MOSI  = 10;//pin 19
static const uint8_t SCK   = 11;//pin 23
static const uint8_t PWM00 = 12;//pin 32 //B+
static const uint8_t PWM01 = 13;//pin 33 //B+
static const uint8_t TX    = 14;//pin 8
static const uint8_t RX    = 15;//pin 10
//static const uint8_t D16 = 16;//pin 36 //B+
//static const uint8_t D17 = 17;//pin 11
static const uint8_t PWM10 = 18;//pin 12        //PCM_CLK
static const uint8_t PWM11 = 19;//pin 35 //B+   //PCM_FS
//static const uint8_t D20 = 20;//pin 38 //B+   //PCM_DIN
//static const uint8_t D21 = 21;//pin 40 //B+   //PCM_DOUT
//static const uint8_t D22 = 22;//pin 15
//static const uint8_t D23 = 23;//pin 16
//static const uint8_t D24 = 24;//pin 18
//static const uint8_t D25 = 25;//pin 22
//static const uint8_t D26 = 26;//pin 37 //B+
//static const uint8_t D27 = 27;//pin 13
//static const uint8_t D28 = 28;//pin  //A/B    //PCM_CLK  //ID_SD
//static const uint8_t D29 = 29;//pin  //A/B    //PCM_FS   //ID_SC
//static const uint8_t D30 = 30;//pin  //A/B    //PCM_DIN
//static const uint8_t D31 = 31;//pin  //A/B    //PCM_DOUT

#define SERIAL_PORT_USBVIRTUAL  Console
#define SERIAL_PORT_MONITOR     Console
#define SERIAL_PORT_HARDWARE    Serial

#endif
