/*
 * PitotStaticHelper.cpp
 * Copyright (C) 2019 Bruce Meacham
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "PitotStaticHelper.h"
#include "Arduino.h"

long pitotOffset;
const int sampleCount = 10;

void PitotStaticSetup()
{
    // Initialize the Pitot's analog in and calibrate the pitot
    pinMode(PITOT_GPIO_INPUT, INPUT);
    pitotOffset = 0;
    for(int i=0;i<10*sampleCount;i++)
    {
         pitotOffset += analogRead(PITOT_GPIO_INPUT)-512;
    }
    pitotOffset /= 10;

    // Initialize a UART for the WT-109B sensors
    Serial.begin(115200);
}

// Get airspeed in kph
int GetAirspeedKph()
{
    long sensorValue;

    // Read from the ADC and convert dynamic pressure to airspeed in kph 
    for(int i = 0; i < sampleCount; i++)
        sensorValue += analogRead(PITOT_GPIO_INPUT);

    // Average it out
    sensorValue = (sensorValue-pitotOffset) / sampleCount; 

    float P=(5*sensorValue)/1024.0-2.5;                        // this Millibars
    return (int)(2.0 * sqrt (((P - 0.14)*4000.0)/1.2));        //note: this could be mph ?
}

unsigned char buf[11];

unsigned char _readDataFrame()
{
    // invalidate the buffer straight away
    if(buf[0] = ((unsigned char)Serial.read()) != 0x55) 
        return 0xFF;

    // read a 8 byte frame (with headers and checksum)
    // and calculate the checksum
    unsigned char checkSum = 0;
    for (int i = 0; i < 9; i++)
        checkSum += (buf[i] = (unsigned char)Serial.read());

    // If it all works out... validate the header
    return (checkSum ==  buf[10]) ? buf[1] : 0xFF;
}

// Get pitch roll yaw temp and baro pressure
AttitudeTempBaro GetAttitudeTempBaro()
{
    AttitudeTempBaro _atbT;
    unsigned char buf[11];
    const int retryCount = 100; // how many failed attempts until we get all the data

    unsigned char readAll = 0;

    for(int i = retryCount+1 ; i > 0; i--)
    {
        switch (_readDataFrame())
        {
            case 0x53:
                _atbT.Pitch = (short)(long(buf [3]<<8 | buf [2])*180)>>15;
                _atbT.Roll = (short)(long(buf [5]<<8 | buf [4])*180)>>15;
                _atbT.Yaw = (short)(long(buf [7]<<8 | buf [6])*180)>>15;
                _atbT.Temperature = (short(buf [9]<<8| buf [8]))/340.0+36.25;
                readAll |= 1;
                break;
            case 0x56:
                _atbT.PressureAltitude = long(buf [5]<<24 | buf [4]<<16 | buf [3]<<8 | buf [2]);
                readAll |= 2;
                break;
            default:
                break;
        }

        if (readAll == 0x03)
            return _atbT;
    }

    return _atbT;
}
