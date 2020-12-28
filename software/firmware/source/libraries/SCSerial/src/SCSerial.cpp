/*
 * SCSerial.cpp
 *
 * UART emulation by Sensor Controller in TI CC13X0
 *
 * Copyright (C) 2020-2021 Linar Yusupov.  All rights reserved.
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

#include "SCSerial.h"

#include "ex_include.h"
#include "scif.h"

// Display error message if the SCIF driver has been generated with incorrect operating system setting
#ifndef SCIF_OSAL_NONE_H
    #error "SCIF driver has incorrect operating system configuration for this example. Please change to 'None' in the Sensor Controller Studio project panel and re-generate the driver."
#endif

// Display error message if the SCIF driver has been generated with incorrect target chip package
#ifndef SCIF_TARGET_CHIP_PACKAGE_QFN48_7X7_RGZ
    #error "SCIF driver has incorrect target chip package configuration for this example. Please change to 'QFN48 7x7 RGZ' in the Sensor Controller Studio project panel and re-generate the driver."
#endif

SCSerial::SCSerial() {

}

SCSerial::~SCSerial() {
    end();
}

void SCSerial::begin(uint32_t baud)
{
#if !defined(DeviceFamily_CC13X2) && !defined(DeviceFamily_CC13X2_V1) && !defined(DeviceFamily_CC13X2_V2) && !defined(DeviceFamily_CC26X2) && !defined(DeviceFamily_CC26X2_V1) && !defined(DeviceFamily_CC26X2_V2)
    // In this example, we keep the AUX domain access permanently enabled
    scifOsalEnableAuxDomainAccess();
#endif

    // Initialize and start the Sensor Controller
    scifInit(&scifDriverSetup);

    // Start the UART emulator
    scifExecuteTasksOnceNbl(1 << (SCIF_UART_EMULATOR_TASK_ID));

    // Enable baud rate generation
    scifUartSetBaudRate(baud);

    enableRx(true);
}

void SCSerial::end()
{
    enableRx(false);

    // Stop the UART Emulator and wait for it to take effect
    scifUartStopEmulator();
    scifWaitOnNbl(10000);
    
    // Disable baud rate generation
    scifUartSetBaudRate(0);

    // Uninitialize the SCIF driver (includes I/O configuration)
    scifUninit();
}

void SCSerial::enableRx(bool on) {

    if (on) {
      // Enable RX
      scifUartSetRxTimeout(20);
      scifUartSetRxEnableReqIdleCount(1);
      scifUartRxEnable(1);

      // Enable events
      scifUartSetEventMask(0xF);
    } else {
      scifUartRxEnable(0);
    }
}

void SCSerial::enableTx(bool on) {
    /* TBD */
}

int SCSerial::available() {
    return scifUartGetRxFifoCount();
}


int SCSerial::peek(void)
{
    /* TBD */
    return (-1);
}


int SCSerial::read() {
    if (available()) {
      return (char) scifUartRxGetChar();
    } else {
      return (-1);
    }
}

size_t SCSerial::write(uint8_t byte) {
    scifUartTxPutChar((char) byte);;
    return sizeof(byte);
}

void SCSerial::flush()
{
    /* TBD */
}
