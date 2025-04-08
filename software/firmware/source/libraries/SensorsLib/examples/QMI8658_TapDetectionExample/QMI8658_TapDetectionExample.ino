/**
 *
 * @license MIT License
 *
 * Copyright (c) 2024 lewis he
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @file      QMI8658_TapDetectionExample.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2024-09-26
 *
 */
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "SensorQMI8658.hpp"
#ifdef ARDUINO_T_BEAM_S3_SUPREME
#include <XPowersAXP2101.tpp>   //PMU Library https://github.com/lewisxhe/XPowersLib.git
#endif


// #define USE_I2C              //Using the I2C interface

#ifdef USE_I2C
#ifndef SENSOR_SDA
#define SENSOR_SDA  17
#endif

#ifndef SENSOR_SCL
#define SENSOR_SCL  18
#endif

#else   /* SPI interface */

#ifndef SPI_MOSI
#define SPI_MOSI   (35)
#endif

#ifndef SPI_SCK
#define SPI_SCK    (36)
#endif

#ifndef SPI_MISO
#define SPI_MISO   (37)
#endif

#ifndef IMU_CS
#define IMU_CS      34      // IMU CS PIN
#endif

#endif  /* USE_I2C*/


#ifndef IMU_IRQ
#define IMU_IRQ     33      // IMU INT PIN
#endif

#ifndef OLED_SDA
#define OLED_SDA    22      // Display Wire SDA Pin
#endif

#ifndef OLED_SCL
#define OLED_SCL    21      // Display Wire SCL Pin
#endif

SensorQMI8658 qmi;


bool interruptFlag = false;

void setFlag(void)
{
    interruptFlag = true;
}


void beginPower()
{
    // T_BEAM_S3_SUPREME The PMU voltage needs to be turned on to use the sensor
#if defined(ARDUINO_T_BEAM_S3_SUPREME)
    XPowersAXP2101 power;
    power.begin(Wire1, AXP2101_SLAVE_ADDRESS, 42, 41);
    power.disableALDO1();
    power.disableALDO2();
    delay(250);
    power.setALDO1Voltage(3300); power.enableALDO1();
    power.setALDO2Voltage(3300); power.enableALDO2();
#endif
}

void tapEventCallback()
{
    SensorQMI8658::TapEvent event = qmi.getTapStatus();
    switch (event) {
    case SensorQMI8658::SINGLE_TAP:
        Serial.println("Single-TAP");
        break;
    case SensorQMI8658::DOUBLE_TAP:
        Serial.println("Double-TAP");
        break;
    default:
        break;
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial);

    beginPower();

    bool ret = false;
#ifdef USE_I2C
    ret = qmi.begin(Wire, QMI8658_L_SLAVE_ADDRESS, SENSOR_SDA, SENSOR_SCL);
#else
#if defined(SPI_MOSI) && defined(SPI_SCK) && defined(SPI_MISO)
    ret = qmi.begin(SPI, IMU_CS, SPI_MOSI, SPI_MISO, SPI_SCK);
#else
    ret = qmi.begin(SPI, IMU_CS);
#endif
#endif

    if (!ret) {
        Serial.println("Failed to find QMI8658 - check your wiring!");
        while (1) {
            delay(1000);
        }
    }

    /* Get chip id*/
    Serial.print("Device ID:");
    Serial.println(qmi.getChipID(), HEX);

    //** The recommended output data rate for detection is higher than 500HZ
    qmi.configAccelerometer(
        /*
         * ACC_RANGE_2G
         * ACC_RANGE_4G
         * ACC_RANGE_8G
         * ACC_RANGE_16G
         * */
        SensorQMI8658::ACC_RANGE_4G,
        /*
         * ACC_ODR_1000H
         * ACC_ODR_500Hz
         * ACC_ODR_250Hz
         * ACC_ODR_125Hz
         * ACC_ODR_62_5Hz
         * ACC_ODR_31_25Hz
         * ACC_ODR_LOWPOWER_128Hz
         * ACC_ODR_LOWPOWER_21Hz
         * ACC_ODR_LOWPOWER_11Hz
         * ACC_ODR_LOWPOWER_3H
        * */
        SensorQMI8658::ACC_ODR_500Hz);

    // Enable the accelerometer
    qmi.enableAccelerometer();

    //* Priority definition between the x, y, z axes of acceleration.
    uint8_t priority = SensorQMI8658::PRIORITY0;   //(X > Y> Z)
    //* Defines the maximum duration (in sample) for a valid peak.
    //* In a valid peak, the linear acceleration should reach or be higher than the PeakMagThr
    //* and should return to quiet (no significant movement) within UDMThr, at the end of PeakWindow.
    uint8_t peakWindow = 20; //20 @500Hz ODR
    //* Defines the minimum quiet time before the second Tap happen.
    //* After the first Tap is detected, there should be no significant movement (defined by UDMThr) during the TapWindow.
    //* The valid second tap should be detected after TapWindow and before DTapWindow.
    uint16_t tapWindow = 50; //50 @500Hz ODR
    //* Defines the maximum time for a valid second Tap for Double Tap,
    //* count start from the first peak of the valid first Tap.
    uint16_t dTapWindow = 250; //250 @500Hz ODR
    //* Defines the ratio for calculating the average of the movement
    //* magnitude. The bigger of Gamma, the bigger weight of the latest  data.
    float alpha = 0.0625;
    //* Defines the ratio for calculating the average of the movement
    //* magnitude. The bigger of Gamma, the bigger weight of the latest data.
    float gamma = 0.25;
    //* Threshold for peak detection.
    float peakMagThr = 0.8; //0.8g square
    //* Undefined Motion threshold. This defines the threshold of the
    //* Linear Acceleration for quiet status.
    float UDMTh = 0.4; //0.4g square

    qmi.configTap(priority, peakWindow, tapWindow,
                  dTapWindow, alpha, gamma, peakMagThr, UDMTh);

    // Enable the Tap Detection and enable the interrupt
    qmi.enableTap(SensorQMI8658::INTERRUPT_PIN_1);

    // Set the Tap Detection callback function
    qmi.setTapEventCallBack(tapEventCallback);

    /*
     * When the QMI8658 is configured as Wom, the interrupt level is arbitrary,
     * not absolute high or low, and it is in the jump transition state
     */
    attachInterrupt(IMU_IRQ, setFlag, CHANGE);
}


void loop()
{
    if (interruptFlag) {
        interruptFlag = false;
        qmi.update();
    }
    delay(50);
}