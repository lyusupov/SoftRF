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
 * @file      BHI260AP_Klio_Selflearning.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2025-02-02
 * @note      Changed from Boschsensortec API https://github.com/boschsensortec/BHY2_SensorAPI
 */
#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>
#include "SensorBHI260AP.hpp"
#include "SensorBHI260AP_Klio.hpp"

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
SensorBHI260AP_Klio klio(bhy);

// The firmware runs in RAM and will be lost if the power is off. The firmware will be loaded from RAM each time it is run.
#define BOSCH_BHI260_KLIO

// Firmware is stored in flash and booted from flash,Depends on BHI260 hardware connected to SPI Flash
// #define BOSCH_BHI260_KLIO_FLASH
// #define BOSCH_BHI260_KLIO_TURBO_FLASH

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

/**
 * @brief Callback function for handling KLIO sensor recognition events.
 *
 * This function is invoked when the KLIO sensor successfully recognizes a specific pattern.
 * Its main purpose is to print information about the recognized pattern, including the pattern ID
 * and the recognition count, to the serial monitor. This allows developers to monitor the recognition
 * process effectively.
 *
 * @param pattern_id The unique identifier of the recognized pattern. Each predefined or learned
 *                   pattern in the system has a distinct ID, and this parameter indicates which
 *                   specific pattern has been recognized.
 * @param count A floating-point value representing the recognition count. This could be the number
 *              of times the pattern has been recognized, a confidence level associated with the
 *              recognition, or some other metric depending on the implementation of the recognition
 *              algorithm.
 * @param user_data A pointer to user-defined data. It can be used to pass additional context or
 *                  information from the calling code to this callback function. In this implementation,
 *                  it is not used, but it is included to maintain compatibility with the callback function signature.
 */
void recognition_event_callback(uint8_t pattern_id, float count, void *user_data)
{
    Serial.print("<-Recognition[Id:");
    Serial.print(pattern_id);
    Serial.print(" Count:");
    Serial.print(count);
    Serial.print("]");
}

/**
 * @brief Callback function for KLIO sensor learning events.
 *
 * This function is invoked whenever there is a change in the learning process of the KLIO sensor.
 * It handles different learning - related events, logs information about the learning progress,
 * and takes actions based on the learning results.
 *
 * @param reason The reason for the learning change. It is an enumeration value from the
 *               SensorBHI260AP_Klio::LeaningChangeReason type, indicating why the learning state has changed.
 * @param progress The current progress of the learning process, represented as an unsigned 32 - bit integer.
 *                 This value typically ranges from 0 to 100, indicating the percentage of the learning completion.
 * @param learn_index The index of the learned pattern. If the learning is invalid, it will be set to
 *                    SensorBHI260AP_Klio::INVALID_LEARNING_INDEX. Otherwise, it represents the index of the successfully learned pattern.
 * @param user_data A pointer to user - defined data. It can be used to pass additional context information
 *                  from the calling code to this callback function. In this implementation, it may not be used actively.
 */
void learning_event_callback(SensorBHI260AP_Klio::LeaningChangeReason reason, uint32_t progress, int learn_index, void *user_data)
{
    // Print the learning event details to the serial monitor, including the progress, reason, and learned pattern index.
    Serial.print("->Learning [Progress:");
    Serial.print(progress);
    Serial.print(" Reason:");
    Serial.print(static_cast<uint8_t>(reason));
    Serial.print(" ID:");
    Serial.print(learn_index);
    Serial.println("]");

    // Check if the learning index is valid (not equal to INVALID_LEARNING_INDEX).
    if (learn_index != SensorBHI260AP_Klio::INVALID_LEARNING_INDEX) {
        // Create a buffer to store the learned pattern data. The buffer size is 252 bytes.
        uint8_t tmp_buf[252];
        // Store the size of the buffer. This variable will be updated with the actual size of the learned pattern.
        uint16_t bufsize = sizeof(tmp_buf);

        // Try to retrieve the learned pattern from the sensor using the getLearnPattern function.
        // The result indicates whether the retrieval is successful.
        bool learn_success = klio.getLearnPattern(tmp_buf, &bufsize);
        if (!learn_success) {
            // If the retrieval fails, print an error message and the specific error reason.
            Serial.print("Read learnt failed. Reason:");
            Serial.println(klio.errorToString());
        } else {
            // If the retrieval is successful, print a success message and the details of the learned pattern.
            Serial.println("Learning the action successfully");
            Serial.println("PATTERN LEARNT: ");
            Serial.print("const uint8_t * learn_pattern = { ");
            // Iterate through the buffer and print the pattern data in hexadecimal format.
            for (uint16_t i = 0; i < bufsize; i++) {
                if (i > 0 && i % 8 == 0) {
                    // Print a new line every 8 bytes for better readability.
                    Serial.println();
                }
                Serial.print("0x"); Serial.print(tmp_buf[i], HEX);
                if (i < bufsize - 1) {
                    // Add a comma and a space after each byte except the last one.
                    Serial.print(", ");
                }
            }
            Serial.println(" \n};\n");
        }
        // Print a message indicating that the learned pattern will be written.
        Serial.println("Write the learning pattern.");
        // Define an example pattern ID.
        uint8_t examples_id = 1;
        // Try to write the learned pattern to the sensor using the writePattern function.
        if (!klio.writePattern(examples_id, tmp_buf, bufsize)) {
            // If the write operation fails, print an error message.
            Serial.println("Klio write pattern failed!");
        }
        // Print messages indicating that the action recognition will start.
        Serial.println("Start recognizing actions");
        Serial.println("Please perform the learned action instructions and the sensor will start to recognize the number of actions.");
        // Start the recognition process for the specified pattern ID.
        klio.recognition(&examples_id, 1);
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial);

    // Set the reset pin
    bhy.setPins(BHI260_RST);

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

    // Try to initialize the KLIO sensor.
    // The begin() method is called on the 'klio' object to set up the sensor.
    if (!klio.begin()) {
        while (1) {
            Serial.println("Failed to initialize Klio sensor. Are you currently using a firmware that includes Klio sensor functionality?");
            delay(1000);
        }
    }

    // Call the getMaxPatterns() method of the klio object to get the maximum number of patterns allowed by the KLIO sensor.
    // This method returns a value of type uint8_t representing the maximum number of patterns and stores it in the variable max_patterns.
    uint8_t max_patterns = klio.getMaxPatterns();
    Serial.print("Klio sensor max patterns:");
    Serial.println(max_patterns);

    // Set the callback function for learning events.
    // The setLearningCallback() method is used to register a function that will be called
    // whenever a learning - related event occurs in the KLIO sensor.
    // 'learning_event_callback' is the name of the callback function,
    // and 'nullptr' is passed as the user data pointer, meaning no additional user - specific data is provided.
    klio.setLearningCallback(learning_event_callback, nullptr);

    // Set the callback function for recognition events.
    // Similar to the learning callback, the setRecognitionCallback() method registers a function
    // that will be invoked when a recognition - related event happens in the KLIO sensor.
    // 'recognition_event_callback' is the callback function, and 'nullptr' is used as the user data pointer.
    klio.setRecognitionCallback(recognition_event_callback, nullptr);

    // Start the learning process of the KLIO sensor.
    // The learning() method initiates the sensor's functionality to start learning patterns or behaviors.
    klio.learning();

    // Define the sample rate for data reading.
    // The variable'sample_rate' is set to 25.0, which means the sensor will read out data
    // at a frequency of 25 Hertz (25 times per second).
    float sample_rate = 25.0;

    // Define the report latency in milliseconds.
    // The variable'report_latency_ms' is set to 0, indicating that the sensor should report
    // the measured data immediately without any delay.
    uint32_t report_latency_ms = 0;

    // Enable the KLIO sensor with he specified sample rate and report latency.
    // The enable() method activates the sensor and configures it to operate at the given sample rate
    // and report latency. This allows the sensor to start collecting and reporting data according to the settings.
    klio.enable(sample_rate, report_latency_ms);

#ifdef USING_SENSOR_IRQ_METHOD
    // Set the specified pin (BHI260_IRQ) ​​to an input pin.
    // This makes the pin ready to receive external signals.
    // If the interrupt is already connected, if BHI260_IRQ is equal to -1 then the polling method will be used
    pinMode(BHI260_IRQ, INPUT);

    // Attach an interrupt service routine (ISR) to the specified pin (BHI260_IRQ).
    // The ISR 'dataReadyISR' will be called whenever a rising edge is detected on the pin.
    attachInterrupt(BHI260_IRQ, dataReadyISR, RISING);
#endif

    Serial.println("Please repeat the movements you want to learn and the sensor will start recording.");
}


void loop()
{
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

    delay(50);
}
