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
 * @file      BoschSensorInfo.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2025-01-22
 *
 */
#pragma once

#include <stdint.h>
#include <stdio.h>
#include "bhy2_parse.h"
#include "bhy2_defs.h"
#include "common/common.h"
#ifdef ARDUINO
#include <Stream.h>
#endif

class BoschSensorInfo
{
public:
    uint16_t kernel_version;
    uint16_t user_version;
    uint16_t rom_version;
    uint8_t product_id;
    uint8_t host_status;
    uint8_t feat_status;
    uint8_t boot_status;
    uint8_t sensor_error;
    bhy2_dev *dev;
    struct bhy2_sensor_info *info;

    BoschSensorInfo() : kernel_version(0), user_version(0), rom_version(0),
        product_id(0), host_status(0), feat_status(0), boot_status(0), sensor_error(0), dev(nullptr), info(nullptr)
    {
        info = (struct bhy2_sensor_info *)calloc(BHY2_SENSOR_ID_MAX, sizeof(struct bhy2_sensor_info));
    }

    ~BoschSensorInfo()
    {
        if (info) {
            free(info);
        }
    }

    BoschSensorInfo(const BoschSensorInfo &other)
        : kernel_version(other.kernel_version),
          user_version(other.user_version),
          rom_version(other.rom_version),
          product_id(other.product_id),
          host_status(other.host_status),
          feat_status(other.feat_status),
          boot_status(other.boot_status),
          sensor_error(other.sensor_error),
          dev(other.dev),
          info(nullptr)
    {
        if (other.info) {
            info = (struct bhy2_sensor_info *)calloc(BHY2_SENSOR_ID_MAX, sizeof(struct bhy2_sensor_info));
            for (int i = 0; i < BHY2_SENSOR_ID_MAX; ++i) {
                info[i] = other.info[i];
            }
        }
    }

    BoschSensorInfo &operator=(const BoschSensorInfo &other)
    {
        if (this != &other) {
            if (info) {
                free(info);
            }
            kernel_version = other.kernel_version;
            user_version = other.user_version;
            rom_version = other.rom_version;
            product_id = other.product_id;
            host_status = other.host_status;
            feat_status = other.feat_status;
            boot_status = other.boot_status;
            sensor_error = other.sensor_error;
            dev = other.dev;
            info = nullptr;
            if (other.info) {
                info = (struct bhy2_sensor_info *)calloc(BHY2_SENSOR_ID_MAX, sizeof(struct bhy2_sensor_info));
                for (int i = 0; i < BHY2_SENSOR_ID_MAX; ++i) {
                    info[i] = other.info[i];
                }
            }
        }
        return *this;
    }

    BoschSensorInfo(BoschSensorInfo &&other) noexcept
        : kernel_version(other.kernel_version),
          user_version(other.user_version),
          rom_version(other.rom_version),
          product_id(other.product_id),
          host_status(other.host_status),
          feat_status(other.feat_status),
          boot_status(other.boot_status),
          sensor_error(other.sensor_error),
          dev(other.dev),
          info(other.info)
    {
        other.info = nullptr;
        other.dev = nullptr;
    }

    BoschSensorInfo &operator=(BoschSensorInfo &&other) noexcept
    {
        if (this != &other) {
            if (info) {
                free(info);
            }
            kernel_version = other.kernel_version;
            user_version = other.user_version;
            rom_version = other.rom_version;
            product_id = other.product_id;
            host_status = other.host_status;
            feat_status = other.feat_status;
            boot_status = other.boot_status;
            sensor_error = other.sensor_error;
            dev = other.dev;
            info = other.info;
            other.info = nullptr;
            other.dev = nullptr;
        }
        return *this;
    }

#if defined(ARDUINO) && !defined(ARDUINO_ARCH_MBED) && !defined(ARDUINO_ARCH_ZEPHYR)
    void printVirtualSensorList(Stream &stream)
    {
        if (!dev) {
            return;
        }
        if (!info) {
            return;
        }
        if (feat_status & BHY2_FEAT_STATUS_OPEN_RTOS_MSK) {
            stream.printf("Virtual sensor list.\n");
            stream.printf("Sensor ID |                          Sensor Name |  ID | Ver |  Min rate |  Max rate |\n");
            stream.printf("----------+--------------------------------------+-----+-----+-----------+-----------|\n");
            for (uint8_t i = 0; i < BHY2_SENSOR_ID_MAX; i++) {
                if (bhy2_is_sensor_available(i, dev)) {
                    if (i < BHY2_SENSOR_ID_CUSTOM_START) {
                        stream.printf(" %8u | %36s ", i, get_sensor_name(i));
                    }
                    stream.printf("| %3u | %3u | %9.4f | %9.4f |\n",
                                  (unsigned int)info[i].driver_id,
                                  (unsigned int)info[i].driver_version,
                                  info[i].min_rate.f_val,
                                  info[i].max_rate.f_val);
                }
            }
        }
    }

    void printBootStatus(Stream &stream)
    {
        stream.printf("Boot Status : 0x%02x: ", boot_status);
        if (boot_status & BHY2_BST_FLASH_DETECTED) {
            stream.printf("\tFlash detected. \n");
        }
        if (boot_status & BHY2_BST_FLASH_VERIFY_DONE) {
            stream.printf("\tFlash verify done. \n");
        }
        if (boot_status & BHY2_BST_FLASH_VERIFY_ERROR) {
            stream.printf("Flash verification failed. \n");
        }
        if (boot_status & BHY2_BST_NO_FLASH) {
            stream.printf("\tNo flash installed. \n");
        }
        if (boot_status & BHY2_BST_HOST_INTERFACE_READY) {
            stream.printf("\tHost interface ready. \n");
        }
        if (boot_status & BHY2_BST_HOST_FW_VERIFY_DONE) {
            stream.printf("\tFirmware verification done. \n");
        }
        if (boot_status & BHY2_BST_HOST_FW_VERIFY_ERROR) {
            stream.printf("\tFirmware verification error. \n");
        }
        if (boot_status & BHY2_BST_HOST_FW_IDLE) {
            stream.printf("\tFirmware halted. \n");
        }
    }

    void printInfo(Stream &stream)
    {
        stream.printf("Product ID     : %02x\n", product_id);
        stream.printf("Kernel version : %04u\n", kernel_version);
        stream.printf("User version   : %04u\n", user_version);
        stream.printf("ROM version    : %04u\n", rom_version);
        stream.printf("Power state    : %s\n", (host_status & BHY2_HST_POWER_STATE) ? "sleeping" : "active");
        stream.printf("Host interface : %s\n", (host_status & BHY2_HST_HOST_PROTOCOL) ? "SPI" : "I2C");
        stream.printf("Feature status : 0x%02x\n", feat_status);
        printBootStatus(stream);
        if (sensor_error) {
            stream.printf( "%s\n", get_sensor_error_text(sensor_error));
        }
        printVirtualSensorList(stream);
    }
#else
    void printVirtualSensorList()
    {
        if (!dev) {
            return;
        }
        if (!info) {
            return;
        }
        if (feat_status & BHY2_FEAT_STATUS_OPEN_RTOS_MSK) {
            printf("Virtual sensor list.\n");
            printf("Sensor ID |                          Sensor Name |  ID | Ver |  Min rate |  Max rate |\n");
            printf("----------+--------------------------------------+-----+-----+-----------+-----------|\n");
            for (uint8_t i = 0; i < BHY2_SENSOR_ID_MAX; i++) {
                if (bhy2_is_sensor_available(i, dev)) {
                    if (i < BHY2_SENSOR_ID_CUSTOM_START) {
                        printf(" %8u | %36s ", i, get_sensor_name(i));
                    }
                    printf("| %3u | %3u | %9.4f | %9.4f |\n",
                           (unsigned int)info[i].driver_id,
                           (unsigned int)info[i].driver_version,
                           info[i].min_rate.f_val,
                           info[i].max_rate.f_val);
                }
            }
        }
    }

    void printBootStatus()
    {
        printf("Boot Status : 0x%02x: ", boot_status);
        if (boot_status & BHY2_BST_FLASH_DETECTED) {
            printf("\tFlash detected. \n");
        }
        if (boot_status & BHY2_BST_FLASH_VERIFY_DONE) {
            printf("\tFlash verify done. \n");
        }
        if (boot_status & BHY2_BST_FLASH_VERIFY_ERROR) {
            printf("Flash verification failed. \n");
        }
        if (boot_status & BHY2_BST_NO_FLASH) {
            printf("\tNo flash installed. \n");
        }
        if (boot_status & BHY2_BST_HOST_INTERFACE_READY) {
            printf("\tHost interface ready. \n");
        }
        if (boot_status & BHY2_BST_HOST_FW_VERIFY_DONE) {
            printf("\tFirmware verification done. \n");
        }
        if (boot_status & BHY2_BST_HOST_FW_VERIFY_ERROR) {
            printf("\tFirmware verification error. \n");
        }
        if (boot_status & BHY2_BST_HOST_FW_IDLE) {
            printf("\tFirmware halted. \n");
        }
    }

    void printInfo()
    {
        printf("Product ID     : %02x\n", product_id);
        printf("Kernel version : %04u\n", kernel_version);
        printf("User version   : %04u\n", user_version);
        printf("ROM version    : %04u\n", rom_version);
        printf("Power state    : %s\n", (host_status & BHY2_HST_POWER_STATE) ? "sleeping" : "active");
        printf("Host interface : %s\n", (host_status & BHY2_HST_HOST_PROTOCOL) ? "SPI" : "I2C");
        printf("Feature status : 0x%02x\n", feat_status);
        if (sensor_error) {
            printf( "%s\n", get_sensor_error_text(sensor_error));
        }
        printBootStatus();
        printVirtualSensorList();
    }
#endif
};
