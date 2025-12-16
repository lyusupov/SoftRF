#ifndef __AHT20_H__
#define __AHT20_H__

#include <Arduino.h>
#include <Wire.h>

class AHT20{
    
private:

    bool startSensor();
public:

    void begin();
    bool getSensor(float *h, float *t);
    bool getTemperature(float *t);
    bool getHumidity(float *h);
};

#endif