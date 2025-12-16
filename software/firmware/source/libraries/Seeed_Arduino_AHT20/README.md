# Seeed_Arduino_AHT20 [![Build Status](https://travis-ci.com/Seeed-Studio/Seeed_Arduino_ATH20.svg?branch=master)](https://travis-ci.com/Seeed-Studio/Seeed_Arduino_ATH20)

## Introduction

AHT20 is a new generation of temperature and humidity sensor embedded with a dual-row flat and no-lead SMD package, suitable for the reflow soldering. AHT20 is equipped with a newly designed ASIC chip: an improved MEMS semiconductor capacitive humidity sensor, and a standard on-chip temperature sensor. 

## Usage 

```c++
// ARDUINO DEMO FOR GROVE-AHT20
//
#include <Wire.h>
#include "AHT20.h"

AHT20 AHT;

void setup()
{
    Serial.begin(115200);
    Serial.println("AHT20 DEMO");
    AHT.begin();
}

void loop()
{
    float humi, temp;
    
    int ret = AHT.getSensor(&humi, &temp);
    
    if(ret)     // GET DATA OK
    {
        Serial.print("humidity: ");
        Serial.print(humi*100);
        Serial.print("%\t temerature: ");
        Serial.println(temp);
    }
    else        // GET DATA FAIL
    {
        Serial.println("GET DATA FROM AHT20 FAIL");
    }
    
    delay(100);
}

// END FILE
```

## API

### void begin()

Initializing the AHT20

```c++
    AHT.begin();
```

### bool getSensor(float *h, float *t)

get all data of sensor


```c++
    int ret = AHT.getSensor(&humi, &temp);
```

### bool getTemperature(float *t)

get Temperature of sensor


```c++
    int ret = AHT.getTemperature(&temp);
```

### bool getHumidity(float *h)

get Humidity of sensor

```c++
    int ret = AHT.getHumidity(&humi);
```