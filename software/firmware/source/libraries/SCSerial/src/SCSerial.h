/*
 * SCSerial.h
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

#ifndef _SCSerial_H_
#define _SCSerial_H_

#include <Stream.h>

class SCSerial : public Stream {
public:
    SCSerial();
    ~SCSerial();

    void begin(uint32_t baud);

    virtual int available();
    void enableRx(bool on);
    void enableTx(bool on);
    virtual int read();
    virtual size_t write(uint8_t byte);
    virtual void flush();
    virtual int peek(void);
    void end();

    using Print::write;
};

#endif // _SCSerial_H_
