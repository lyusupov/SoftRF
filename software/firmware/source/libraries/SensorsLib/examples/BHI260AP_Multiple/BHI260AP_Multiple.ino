/**
 *
 * @license MIT License
 *
 * Copyright (c) 2025 lewis he
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
 * @file      BHI260AP_Multiple.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2025-02-28
 * @note      Use two BHI260APs. Because I don't have BHI260APs with different addresses, I connect them to different Wire ports and test the software.
 */
#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>
#include <SensorBHI260AP.hpp>
#include <bosch/BoschSensorDataHelper.hpp>

#ifdef ARDUINO_ARCH_ESP32

#ifndef BHI260_SDA1
#define BHI260_SDA1  3
#endif

#ifndef BHI260_SCL1
#define BHI260_SCL1  2
#endif

#ifndef BHI260_IRQ1
#define BHI260_IRQ1  1
#endif

#ifndef BHI260_RST1
#define BHI260_RST1 -1
#endif


#ifndef BHI260_SDA2
#define BHI260_SDA2  11
#endif

#ifndef BHI260_SCL2
#define BHI260_SCL2  12
#endif

#ifndef BHI260_IRQ2
#define BHI260_IRQ2  10
#endif

#ifndef BHI260_RST2
#define BHI260_RST2 -1
#endif

SensorBHI260AP bhy1;
SensorBHI260AP bhy2;

/*
* Define the USING_DATA_HELPER use of data assistants.
* No callback function will be used. Data can be obtained directly through
* the data assistant. Note that this method is not a thread-safe function.
* Please pay attention to protecting data access security.
* */
#define USING_DATA_HELPER

#ifdef USING_DATA_HELPER
SensorOrientation orientation1(bhy1);
SensorOrientation orientation2(bhy2);
#endif

// The firmware runs in RAM and will be lost if the power is off. The firmware will be loaded from RAM each time it is run.
#define BOSCH_APP30_SHUTTLE_BHI260_FW
// #define BOSCH_APP30_SHUTTLE_BHI260_AUX_BMM150FW
// #define BOSCH_APP30_SHUTTLE_BHI260_BME68X
// #define BOSCH_APP30_SHUTTLE_BHI260_BMP390
// #define BOSCH_APP30_SHUTTLE_BHI260_TURBO
// #define BOSCH_BHI260_AUX_BEM280
// #define BOSCH_BHI260_AUX_BMM150_BEM280
// #define BOSCH_BHI260_AUX_BMM150_BEM280_GPIO
// #define BOSCH_BHI260_AUX_BMM150_GPIO
// #define BOSCH_BHI260_GPIO

// Firmware is stored in flash and booted from flash,Depends on BHI260 hardware connected to SPI Flash
// #define BOSCH_APP30_SHUTTLE_BHI260_AUX_BMM150_FLASH
// #define BOSCH_APP30_SHUTTLE_BHI260_BME68X_FLASH
// #define BOSCH_APP30_SHUTTLE_BHI260_BMP390_FLASH
// #define BOSCH_APP30_SHUTTLE_BHI260_FLASH
// #define BOSCH_APP30_SHUTTLE_BHI260_TURBO_FLASH
// #define BOSCH_BHI260_AUX_BEM280_FLASH
// #define BOSCH_BHI260_AUX_BMM150_BEM280_FLASH
// #define BOSCH_BHI260_AUX_BMM150_BEM280_GPIO_FLASH
// #define BOSCH_BHI260_AUX_BMM150_GPIO_FLASH
// #define BOSCH_BHI260_GPIO_FLASH

#include <BoschFirmware.h>

// Force update of current firmware, whether it exists or not.
// Only works when external SPI Flash is connected to BHI260.
// After uploading firmware once, you can change this to false to speed up boot time.
bool force_update_flash_firmware = false;

bool isReadyFlag1 = false;
bool isReadyFlag2 = false;

void dataReady1_ISR()
{
    isReadyFlag1 = true;
}

void dataReady2_ISR()
{
    isReadyFlag2 = true;
}

void print_orientation(uint8_t direction)
{
    char report[256];
    switch (direction) {
    case SensorBHI260AP::DIRECTION_BOTTOM_LEFT:
        sprintf( report, "\r\n  ________________  " \
                 "\r\n |                | " \
                 "\r\n |                | " \
                 "\r\n |                | " \
                 "\r\n |                | " \
                 "\r\n |                | " \
                 "\r\n |  *             | " \
                 "\r\n |________________| \r\n" );

        break;
    case SensorBHI260AP::DIRECTION_TOP_RIGHT:
        sprintf( report, "\r\n  ________________  " \
                 "\r\n |                | " \
                 "\r\n |             *  | " \
                 "\r\n |                | " \
                 "\r\n |                | " \
                 "\r\n |                | " \
                 "\r\n |                | " \
                 "\r\n |________________| \r\n" );
        break;
    case SensorBHI260AP::DIRECTION_TOP_LEFT:
        sprintf( report, "\r\n  ________________  " \
                 "\r\n |                | " \
                 "\r\n |  *             | " \
                 "\r\n |                | " \
                 "\r\n |                | " \
                 "\r\n |                | " \
                 "\r\n |                | " \
                 "\r\n |________________| \r\n" );
        break;
    case SensorBHI260AP::DIRECTION_BOTTOM_RIGHT:
        sprintf( report, "\r\n  ________________  " \
                 "\r\n |                | " \
                 "\r\n |                | " \
                 "\r\n |                | " \
                 "\r\n |                | " \
                 "\r\n |                | " \
                 "\r\n |             *  | " \
                 "\r\n |________________| \r\n" );
        break;
    default:
        sprintf( report, "None of the 3D orientation axes is set in BHI260 - accelerometer.\r\n" );
        break;
    }

    Serial.println(direction);
    Serial.println(report);
}

#ifndef USING_DATA_HELPER
void orientation_process_callback(uint8_t  sensor_id, uint8_t *data_ptr, uint32_t len, uint64_t *timestamp, void *user_data)
{
    uint8_t direction = *data_ptr;
    if (user_data) {
        Serial.println((const char *)user_data);
    }
    print_orientation(direction);
}
#endif

// Firmware update progress callback
void progress_callback(void *user_data, uint32_t total, uint32_t transferred)
{
    float progress = (float)transferred / total * 100;
    Serial.print("Upload progress: ");
    Serial.print(progress);
    Serial.println("%");
}


void initSensor(SensorBHI260AP & sensor, int rst, int sda, int scl, int irq,
                TwoWire &wire, void(*interrupts)(),
                void *user_data
#ifdef USING_DATA_HELPER
                , SensorOrientation&orientation
#endif
               )
{
    // Set the reset pin
    sensor.setPins(rst);

    Serial.println("Initializing Sensors...");

    // Set the firmware array address and firmware size
    sensor.setFirmware(bosch_firmware_image, bosch_firmware_size, bosch_firmware_type, force_update_flash_firmware);

    // Set the firmware update processing progress callback function
    // sensor.setUpdateProcessCallback(progress_callback, NULL);

    // Set the maximum transfer bytes of I2C/SPI,The default size is I2C 32 bytes, SPI 256 bytes.
    // sensor.setMaxiTransferSize(256);

    // Set the processing fifo data buffer size,The default size is 512 bytes.
    // sensor.setProcessBufferSize(1024);

    // Set to load firmware from flash
    sensor.setBootFromFlash(bosch_firmware_type);

    // Using I2C interface
    // BHI260AP_SLAVE_ADDRESS_L = 0x28
    // BHI260AP_SLAVE_ADDRESS_H = 0x29
    if (!sensor.begin(wire, BHI260AP_SLAVE_ADDRESS_L, sda, scl)) {
        Serial.print("Failed to initialize sensor - error code:");
        Serial.println(sensor.getError());
        while (1) {
            delay(1000);
        }
    }

    Serial.println("Initializing the sensor successfully!");

    // Output all sensors info to Serial
    BoschSensorInfo info = sensor.getSensorInfo();
#ifdef PLATFORM_HAS_PRINTF
    info.printInfo(Serial);
#else
    info.printInfo();
#endif

    // The orientation sensor will only report when it changes, so the value is 0 ~ 1
    float sample_rate = 1;
    uint32_t report_latency_ms = 0; /* Report immediately */

#ifdef USING_DATA_HELPER
    orientation.enable(sample_rate, report_latency_ms);
#else
    // Enable direction detection
    sensor.configure(SensorBHI260AP::DEVICE_ORIENTATION, sample_rate, report_latency_ms);
    // Set the direction detection result output processing function
    sensor.onResultEvent(SensorBHI260AP::DEVICE_ORIENTATION, orientation_process_callback, user_data);
#endif
    // Set the specified pin (BHI260_IRQ1) as an input pin.
    pinMode(irq, INPUT_PULLUP);
    // Attach an interrupt service routine (ISR) 'dataReadyISR' to the specified pin (irq).
    attachInterrupt(irq, interrupts, RISING);
}

void setup()
{
    Serial.begin(115200);
    while (!Serial);
    const char *sensor1_user_data = "Sensor1";
    const char *sensor2_user_data = "Sensor2";

    initSensor(bhy1, BHI260_RST1, BHI260_SDA1, BHI260_SCL1, BHI260_IRQ1, Wire, dataReady1_ISR,  (void*)sensor1_user_data
#ifdef USING_DATA_HELPER
               , orientation1
#endif
              );
    initSensor(bhy2, BHI260_RST2, BHI260_SDA2, BHI260_SCL2, BHI260_IRQ2, Wire1, dataReady2_ISR, (void*)sensor2_user_data
#ifdef USING_DATA_HELPER
               , orientation2
#endif
              );

    Serial.println("All sensor init done!");
}


void loop()
{
    // Update sensor fifo
    if (isReadyFlag1) {
        isReadyFlag1 = false;
        Serial.println("isReadyFlag1:");
        bhy1.update();
    }

    if (isReadyFlag2) {
        isReadyFlag2 = false;
        Serial.println("isReadyFlag2:");
        bhy2.update();
    }

#ifdef USING_DATA_HELPER
    if (orientation1.hasUpdated()) {
        print_orientation(orientation1.getOrientation());
    }

    if (orientation2.hasUpdated()) {
        print_orientation(orientation2.getOrientation());
    }
#endif


    delay(50);
}

#else
void setup()
{
    Serial.begin(115200);
}

void loop()
{
    Serial.println("The example only support your esp32 platform"); delay(1000);
}
#endif /*ARDUINO_ARCH_ESP32*/

