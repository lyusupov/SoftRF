#include <Wire.h>
#include "pcf8563.h"

PCF8563_Class rtc;

void setup()
{
    Serial.begin(115200);
    Wire.begin(21, 22);
    rtc.begin();
    rtc.setDateTime(2019, 4, 1, 12, 33, 59);
}

void loop()
{
    Serial.println(rtc.formatDateTime(PCF_TIMEFORMAT_YYYY_MM_DD_H_M_S));
    delay(1000);
}




