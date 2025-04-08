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
 * @file      BHI260AP_Expand_GPIO.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2025-01-04
 * @note      Changed from Boschsensortec API https://github.com/boschsensortec/BHY2_SensorAPI
 */
#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>
#include "SensorBHI260AP.hpp"

#include <Commander.h>  //Deplib https://github.com/CreativeRobotics/Commander
Commander cmd;
void initialiseCommander();

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

// The firmware runs in RAM and will be lost if the power is off. The firmware will be loaded from RAM each time it is run.
// #define BOSCH_APP30_SHUTTLE_BHI260_FW
// #define BOSCH_APP30_SHUTTLE_BHI260_AUX_BMM150FW
// #define BOSCH_APP30_SHUTTLE_BHI260_BME68X
// #define BOSCH_APP30_SHUTTLE_BHI260_BMP390
// #define BOSCH_APP30_SHUTTLE_BHI260_TURBO
// #define BOSCH_BHI260_AUX_BEM280
// #define BOSCH_BHI260_AUX_BMM150_BEM280
// #define BOSCH_BHI260_AUX_BMM150_BEM280_GPIO
// #define BOSCH_BHI260_AUX_BMM150_GPIO
#define BOSCH_BHI260_GPIO

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

// Firmware update progress callback
void progress_callback(void *user_data, uint32_t total, uint32_t transferred)
{
    float progress = (float)transferred / total * 100;
    Serial.print("Upload progress: ");
    Serial.print(progress);
    Serial.println("%");
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

    Serial.println("Initializing Sensors...");

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

    initialiseCommander();

    Serial.println("Hello: Type 'help' to get help");

    cmd.printCommandPrompt();

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

uint32_t check_millis = 0;

void loop()
{
    //Call the update functions using the activeCommander pointer
    cmd.update();
    
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
}


//All commands for 'master'
//COMMAND ARRAY ------------------------------------------------------------------------------
const commandList_t masterCommands[] = {
    {"help",       helpHandler,     "help"},
    {"set gpio",   setGpioLevel,    "set gpio level"},
    {"get gpio",   getGpioLevel,    "get gpio level"},
    {"dis gpio",   disGpioMode,     "disable gpio"},
};

void initialiseCommander()
{
    cmd.begin(&Serial, masterCommands, sizeof(masterCommands));
    cmd.commandPrompt(ON); //enable the command prompt
    cmd.echo(true);     //Echo incoming characters to theoutput port
    cmd.errorMessages(ON); //error messages are enabled - it will tell us if we issue any unrecognised commands
    //Error messaged do NOT work for quick set and get commands
}

bool helpHandler(Commander &Cmdr)
{
    Serial.println("Help:");
    Serial.println("\tCustom firmware valid gpio : 1, 4, 5, 6, 14, 15, 16, 17, 18, 19, 20");
    Serial.println("\tset gpio [gpio num] [level]");
    Serial.println("\tget gpio [gpio num] [pullup]");
    Serial.println("\tdis gpio [gpio num]");
    return 0;
}

/*
* GPIO Comparison Table
* M1SCX = N.A   ! INVALID PIN
* M1SDX = N.A   ! INVALID PIN
* M1SDI = N.A   ! INVALID PIN
* M2SCX = 14    ! OK
* M2SDX = 15    ! OK
* M2SDI = 16    ! OK
* MCSB1 = 1     ! OK
* MCSB2 = 4     ! OK
* M3SCL = 17    ! OK
* M3SDA = 18    ! OK
* MCSB3 = 5     ! OK
* MCSB4 = 6     ! OK
* JTAG_CLK = 19 ! OK
* JTAG_DIO = 20 ! OK
* RESV1 = 2     ! INVALID PIN
* RESV2 = 3     ! INVALID PIN
* RESV3 = N.A   ! INVALID PIN
* */
bool setGpioLevel(Commander &Cmdr)
{
    int values[2] = {0, 0};
    int items = Cmdr.countItems();
    if (items < 2) {
        return false;
    }
    for (int n = 0; n < 2; n++) {
        Cmdr.getInt(values[n]);
    }
    uint8_t pin = values[0];
    uint8_t level = values[1];
    bhy.digitalWrite(pin, level);
    return 0;
}

bool getGpioLevel(Commander &Cmdr)
{
    int values[2] = {0, 0};
    int items = Cmdr.countItems();
    if (items < 1) {
        return 0;
    }
    if (items > 2 )items = 2;
    for (int n = 0; n < items; n++) {
        Cmdr.getInt(values[n]);
    }
    bool pullup = false;
    uint8_t pin = values[0];
    if (items == 2 ) {
        pullup = values[1];
    }
    uint8_t level = bhy.digitalRead(pin, pullup);
    Serial.print("Get GPIO : "); Serial.print(pin);
    Serial.print(" level is "); Serial.println(level);
    return 0;
}

bool disGpioMode(Commander &Cmdr)
{
    int values[1] = {0};
    int items = Cmdr.countItems();
    if (items < 1) {
        return 0;
    }
    Cmdr.getInt(values[0]);
    uint8_t pin = values[0];
    bhy.disableGpio(pin);
    return 0;
}
