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
 * @file      BHI260AP_aux_BMM150_BME280.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2024-07-24
 * @note      Changed from Boschsensortec API https://github.com/boschsensortec/BHY2_SensorAPI
 */
#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>
#include <SensorBHI260AP.hpp>
#include <bosch/BoschSensorDataHelper.hpp>

// #define USE_I2C_INTERFACE        true
// #define USE_SPI_INTERFACE        true

#if !defined(USE_I2C_INTERFACE) && !defined(USE_SPI_INTERFACE)
#define USE_I2C_INTERFACE
#warning "No interface type is selected, use I2C interface"
#endif

#if defined(USE_SPI_INTERFACE)
#ifndef SPI_MOSI
#define SPI_MOSI    33
#endif

#ifndef SPI_MISO
#define SPI_MISO    34
#endif

#ifndef SPI_SCK
#define SPI_SCK     35
#endif

// If BHI260_IRQ is set to -1, sensor interrupts are not used and the sensor polling method is used instead.
#ifndef BHI260_IRQ
#define BHI260_IRQ  37
#endif

#ifndef BHI260_CS
#define BHI260_CS   36
#endif

#else   //* I2C */

#ifndef BHI260_SDA
#define BHI260_SDA  2
#endif

#ifndef BHI260_SCL
#define BHI260_SCL  3
#endif

// If BHI260_IRQ is set to -1, sensor interrupts are not used and the sensor polling method is used instead.
#ifndef BHI260_IRQ
#define BHI260_IRQ  8
#endif
#endif  /*USE_SPI_INTERFACE*/

#ifndef BHI260_RST
#define BHI260_RST -1
#endif

SensorBHI260AP bhy;

/*
* Define the USING_DATA_HELPER use of data assistants.
* No callback function will be used. Data can be obtained directly through
* the data assistant. Note that this method is not a thread-safe function.
* Please pay attention to protecting data access security.
* */
#define USING_DATA_HELPER

#ifdef USING_DATA_HELPER
SensorTemperature temperature(bhy);
SensorHumidity humidity(bhy);
SensorPressure pressure(bhy);
#endif

// The firmware runs in RAM and will be lost if the power is off. The firmware will be loaded from RAM each time it is run.
// #define BOSCH_APP30_SHUTTLE_BHI260_FW
// #define BOSCH_APP30_SHUTTLE_BHI260_AUX_BMM150FW
// #define BOSCH_APP30_SHUTTLE_BHI260_BME68X
// #define BOSCH_APP30_SHUTTLE_BHI260_BMP390
// #define BOSCH_APP30_SHUTTLE_BHI260_TURBO
// #define BOSCH_BHI260_AUX_BEM280
#define BOSCH_BHI260_AUX_BMM150_BEM280
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
bool force_update_flash_firmware = true;

#if BHI260_IRQ > 0
#define USING_SENSOR_IRQ_METHOD
#endif

#ifdef USING_SENSOR_IRQ_METHOD
bool isReadyFlag = false;

void dataReadyISR()
{
    isReadyFlag = true;
}
#endif /*USING_SENSOR_IRQ_METHOD*/

#ifndef USING_DATA_HELPER
void parse_bme280_sensor_data(uint8_t sensor_id, uint8_t *data_ptr, uint32_t len, uint64_t *timestamp, void *user_data)
{
    float humidity = 0;
    float temperature = 0;
    float pressure = 0;
    switch (sensor_id) {
    case SensorBHI260AP::HUMIDITY:
        bhy2_parse_humidity(data_ptr, &humidity);
        Serial.print("humidity:"); Serial.print(humidity); Serial.println("%");
        break;
    case SensorBHI260AP::TEMPERATURE:
        bhy2_parse_temperature_celsius(data_ptr, &temperature);
        Serial.print("temperature:"); Serial.print(temperature); Serial.println("*C");
        break;
    case SensorBHI260AP::BAROMETER:
        bhy2_parse_pressure(data_ptr, &pressure);
        Serial.print("pressure:"); Serial.print(pressure); Serial.println("Pa");
        break;
    default:
        Serial.println("Unkown.");
        break;
    }
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

void printResult(uint8_t sensor_id, float sample_rate, bool rlst)
{
    const char  *sensorName = bhy.getSensorName(sensor_id);
    Serial.print("Configure ");
    Serial.print(sensorName);
    Serial.print(" sensor ");
    Serial.print(sample_rate, 2);
    Serial.print(" HZ ");
    if (rlst) {
        Serial.print("successfully");
    } else {
        Serial.print("failed");
    }
    Serial.println();
}


void setup()
{
    Serial.begin(115200);
    while (!Serial);

    // Set the reset pin
    bhy.setPins(BHI260_RST);

    Serial.println("Initializing Sensors...");

    // Set the firmware array address and firmware size
    bhy.setFirmware(bosch_firmware_image, bosch_firmware_size, bosch_firmware_type, force_update_flash_firmware);

    // Set the firmware update processing progress callback function
    // bhy.setUpdateProcessCallback(progress_callback, NULL);

    // Set the maximum transfer bytes of I2C/SPI,The default size is I2C 32 bytes, SPI 256 bytes.
    // bhy.setMaxiTransferSize(256);

    // Set the processing fifo data buffer size,The default size is 512 bytes.
    // bhy.setProcessBufferSize(1024);

    // Set to load firmware from flash
    bhy.setBootFromFlash(bosch_firmware_type);

#ifdef USE_I2C_INTERFACE
    // Using I2C interface
    // BHI260AP_SLAVE_ADDRESS_L = 0x28
    // BHI260AP_SLAVE_ADDRESS_H = 0x29
    if (!bhy.begin(Wire, BHI260AP_SLAVE_ADDRESS_L, BHI260_SDA, BHI260_SCL)) {
        Serial.print("Failed to initialize sensor - error code:");
        Serial.println(bhy.getError());
        while (1) {
            delay(1000);
        }
    }
#endif

#ifdef USE_SPI_INTERFACE
    // Using SPI interface
    if (!bhy.begin(SPI, BHI260_CS, SPI_MOSI, SPI_MISO, SPI_SCK)) {
        Serial.print("Failed to initialize sensor - error code:");
        Serial.println(bhy.getError());
        while (1) {
            delay(1000);
        }
    }
#endif

    Serial.println("Initializing the sensor successfully!");

    // Output all sensors info to Serial
    BoschSensorInfo info = bhy.getSensorInfo();
#ifdef PLATFORM_HAS_PRINTF
    info.printInfo(Serial);
#else
    info.printInfo();
#endif

    /*
    * Enable monitoring.
    * sample_rate ​​can only control the rate of the pressure sensor.
    * Temperature and humidity will only be updated when there is a change.
    * * */
    float sample_rate = 2.0;       /* Set pressure sensor to 2Hz update frequency */
    uint32_t report_latency_ms = 0; /* Report immediately */

    /*
    * Enable BME280 function
    * Function depends on BME280.
    * If the hardware is not connected to BME280, the function cannot be used.
    * * */
#ifdef USING_DATA_HELPER
    temperature.enable();
    humidity.enable();
    pressure.enable(sample_rate, report_latency_ms);
#else
    bool rlst = false;
    rlst = bhy.configure(SensorBHI260AP::TEMPERATURE, sample_rate, report_latency_ms);
    printResult(SensorBHI260AP::TEMPERATURE, sample_rate, rlst);
    rlst = bhy.configure(SensorBHI260AP::BAROMETER, sample_rate, report_latency_ms);
    printResult(SensorBHI260AP::BAROMETER, sample_rate, rlst);
    rlst = bhy.configure(SensorBHI260AP::HUMIDITY, sample_rate, report_latency_ms);
    printResult(SensorBHI260AP::HUMIDITY, sample_rate, rlst);
    // Register BME280 data parse callback function
    Serial.println("Register sensor result callback function");
    bhy.onResultEvent(SensorBHI260AP::TEMPERATURE, parse_bme280_sensor_data);
    bhy.onResultEvent(SensorBHI260AP::HUMIDITY, parse_bme280_sensor_data);
    bhy.onResultEvent(SensorBHI260AP::BAROMETER, parse_bme280_sensor_data);
#endif

#ifdef USING_SENSOR_IRQ_METHOD
    // Set the specified pin (BHI260_IRQ) ​​to an input pin.
    // This makes the pin ready to receive external signals.
    // If the interrupt is already connected, if BHI260_IRQ is equal to -1 then the polling method will be used
    pinMode(BHI260_IRQ, INPUT);

    // Attach an interrupt service routine (ISR) to the specified pin (BHI260_IRQ).
    // The ISR 'dataReadyISR' will be called whenever a rising edge is detected on the pin.
    attachInterrupt(BHI260_IRQ, dataReadyISR, RISING);
#endif

}


void loop()
{
    uint32_t s;
    uint32_t ns;

#ifdef USING_SENSOR_IRQ_METHOD
    if (isReadyFlag) {
        isReadyFlag = false;
#endif /*USING_SENSOR_IRQ_METHOD*/

        /* If the interrupt is connected to the sensor and BHI260_IRQ is not equal to -1,
         * the interrupt function will be enabled, otherwise the method of polling the sensor is used
         */
        bhy.update();

#ifdef USING_SENSOR_IRQ_METHOD
    }
#endif /*USING_SENSOR_IRQ_METHOD*/

#ifdef USING_DATA_HELPER
    if (temperature.hasUpdated()) {
        temperature.getLastTime(s, ns);
#ifdef PLATFORM_HAS_PRINTF
        Serial.printf("[T: %" PRIu32 ".%09" PRIu32 "] ", s, ns);
        Serial.printf("Temperature: %.2f *C %.2f *F %.2f K\n",
                      temperature.getCelsius(), temperature.getFahrenheit(), temperature.getKelvin());
#else
        Serial.print("[T: ");
        Serial.print(s);
        Serial.print(".");
        Serial.print(ns);
        Serial.print("]:");
        Serial.print("Temperature: ");
        Serial.print(temperature.getCelsius()); Serial.print(" *C");
        Serial.print(temperature.getFahrenheit()); Serial.print(" *F");
        Serial.print(temperature.getKelvin()); Serial.print(" *F");
        Serial.println();
#endif
    }

    if (humidity.hasUpdated()) {
        humidity.getLastTime(s, ns);
#ifdef PLATFORM_HAS_PRINTF
        Serial.printf("[T: %" PRIu32 ".%09" PRIu32 "] ", s, ns);
        Serial.printf("Humidity: %.2f %%\n", humidity.getHumidity());
#else
        Serial.print("[T: ");
        Serial.print(s);
        Serial.print(".");
        Serial.print(ns);
        Serial.print("]:");
        Serial.print("Humidity: ");
        Serial.print(humidity.getHumidity()); Serial.print(" %");
        Serial.println();
#endif
    }

    if (pressure.hasUpdated()) {
        pressure.getLastTime(s, ns);
#ifdef PLATFORM_HAS_PRINTF
        Serial.printf("[T: %" PRIu32 ".%09" PRIu32 "] ", s, ns);
        Serial.printf("Pressure: %.2f Pascal\n", pressure.getPressure());
#else
        Serial.print("[T: ");
        Serial.print(s);
        Serial.print(".");
        Serial.print(ns);
        Serial.print("]:");
        Serial.print("Humidity: ");
        Serial.print(pressure.getPressure()); Serial.print(" Pascal");
        Serial.println();
#endif
    }
#endif
    delay(50);
}



