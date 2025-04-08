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
 * @file      QMI8658_ReadFromFifoExample.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2024-09-25
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

const uint16_t buffer_size = 128;
IMUdata accel[buffer_size];
IMUdata gyro[buffer_size];
bool disable_fifo = false;
uint32_t timestamp = 0;

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

void setup()
{
    Serial.begin(115200);
    while (!Serial);

    beginPower();

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

    /*
    * Get chip id
    * */
    Serial.print("Device ID:"); Serial.println(qmi.getChipID(), HEX);


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
        SensorQMI8658::ACC_ODR_1000Hz,
        /*
        *  LPF_MODE_0     //2.66% of ODR
        *  LPF_MODE_1     //3.63% of ODR
        *  LPF_MODE_2     //5.39% of ODR
        *  LPF_MODE_3     //13.37% of ODR
        *  LPF_OFF        // OFF Low-Pass Fitter
        * */
        SensorQMI8658::LPF_MODE_0);




    qmi.configGyroscope(
        /*
        * GYR_RANGE_16DPS
        * GYR_RANGE_32DPS
        * GYR_RANGE_64DPS
        * GYR_RANGE_128DPS
        * GYR_RANGE_256DPS
        * GYR_RANGE_512DPS
        * GYR_RANGE_1024DPS
        * */
        SensorQMI8658::GYR_RANGE_64DPS,
        /*
         * GYR_ODR_7174_4Hz
         * GYR_ODR_3587_2Hz
         * GYR_ODR_1793_6Hz
         * GYR_ODR_896_8Hz
         * GYR_ODR_448_4Hz
         * GYR_ODR_224_2Hz
         * GYR_ODR_112_1Hz
         * GYR_ODR_56_05Hz
         * GYR_ODR_28_025Hz
         * */
        SensorQMI8658::GYR_ODR_896_8Hz,
        /*
        *  LPF_MODE_0     //2.66% of ODR
        *  LPF_MODE_1     //3.63% of ODR
        *  LPF_MODE_2     //5.39% of ODR
        *  LPF_MODE_3     //13.37% of ODR
        *  LPF_OFF        // OFF Low-Pass Fitter
        * */
        SensorQMI8658::LPF_MODE_0);



    qmi.configFIFO(
        /*
        * FIFO_MODE_BYPASS      -- Disable FIFO
        * FIFO_MODE_FIFO        -- Will not overwrite
        * FIFO_MODE_STREAM      -- Cover
        */
        SensorQMI8658::FIFO_MODE_FIFO,
        /*
         * FIFO_SAMPLES_16
         * FIFO_SAMPLES_32
         * FIFO_SAMPLES_64
         * FIFO_SAMPLES_128
        * */
        SensorQMI8658::FIFO_SAMPLES_128,


        /*
        * INTERRUPT_PIN_1,
        * INTERRUPT_PIN_2,
        * INTERRUPT_PIN_DISABLE
        * * */
        SensorQMI8658::INTERRUPT_PIN_1,     //*Route FIFO interrupt to INT1 Pin

        //* Number of samples to trigger interrupt
        16
    );


    /*
    * If both the accelerometer and gyroscope sensors are turned on at the same time,
    * the output frequency will be based on the gyroscope output frequency.
    * The example configuration is 896.8HZ output frequency,
    * so the acceleration output frequency is also limited to 896.8HZ
    * */

    qmi.enableAccelerometer();

    qmi.enableGyroscope();



#if IMU_IRQ > 0
// If you want to enable interrupts, then turn on the interrupt enable
    qmi.enableINT(SensorQMI8658::INTERRUPT_PIN_1, true);
    qmi.enableINT(SensorQMI8658::INTERRUPT_PIN_2, false);
#endif


    delay(3000);


    Serial.println("Read data now...");


    // Close FIFO after sampling for 5 seconds
    timestamp = millis() + 5000;
}


void loop()
{
    if (disable_fifo) {
        return ;
    }

    if (millis() >  timestamp) {
        disable_fifo = true;
        // Disable FIFO
        qmi.configFIFO(SensorQMI8658::FIFO_MODE_BYPASS);
        Serial.println(">>>>>>>>>>>>>>>>>>>>>>>>>>>>  Disable FIFO...");
    }

    // Get the number of samples from the FIFO
    uint16_t samples_num = qmi.readFromFifo(accel, buffer_size, gyro, buffer_size);
    if (samples_num == 0) {
        return;
    }

    Serial.print("--------------[");
    Serial.print(millis());
    Serial.print("]:data size:");
    Serial.print(samples_num);
    Serial.println("--------------");

    for (int i = 0; i < buffer_size; ++i) {

        Serial.print('[');
        Serial.print(i);
        Serial.println(']');

        if (qmi.isEnableAccelerometer()) {
            Serial.print("\t ACCEL.x:"); Serial.print(accel[i].x); Serial.println(" m2/s");
            Serial.print("\t ACCEL.y:"); Serial.print(accel[i].y); Serial.println(" m2/s");
            Serial.print("\t ACCEL.z:"); Serial.print(accel[i].z); Serial.println(" m2/s");
        }


        if (qmi.isEnableGyroscope()) {
            Serial.print("\t GYRO.x:"); Serial.print(gyro[i].x); Serial.println(" degrees/sec");
            Serial.print("\t GYRO.y:"); Serial.print(gyro[i].y); Serial.println(" degrees/sec");
            Serial.print("\t GYRO.z:"); Serial.print(gyro[i].z); Serial.println(" degrees/sec");
        }
    }
}



