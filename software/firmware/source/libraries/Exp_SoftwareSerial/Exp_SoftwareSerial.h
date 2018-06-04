/*
Exp_SoftwareSerial.h

Exp_SoftwareSerial.cpp - Implementation of the Arduino software serial for ESP8266/ESP32.
Copyright (c) 2015-2016 Peter Lerup. All rights reserved.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

*/

#ifndef Exp_SoftwareSerial_h
#define Exp_SoftwareSerial_h

#include <inttypes.h>
#include <Stream.h>

// If only one tx or rx wanted then use this as parameter for the unused pin
constexpr int SW_SERIAL_UNUSED_PIN = -1;

// This class is compatible with the corresponding AVR one,
// the constructor however has an optional rx buffer size.
// Baudrates up to 115200 can be used.


class Exp_SoftwareSerial : public Stream {
public:
	Exp_SoftwareSerial(int receivePin, int transmitPin, bool inverse_logic = false, int bufSize = 64, int isrBufSize = 0);
	virtual ~Exp_SoftwareSerial();

	void begin(long baud);
	long baudRate();
	// Transmit control pin
	void setTransmitEnablePin(int transmitEnablePin);
	// Enable or disable interrupts during tx
	void enableIntTx(bool on);

	bool overflow();

	int available() override;
	int peek() override;
	int read() override;
	void flush() override;
	size_t write(uint8_t byte) override;
	size_t write(const uint8_t *buffer, size_t size) override;
	operator bool() const { return m_rxValid || m_txValid; }

	// Disable or enable interrupts on the rx pin
	void enableRx(bool on);
	// One wire control
	void enableTx(bool on);

	void rxRead();

	// AVR compatibility methods
	bool listen() { enableRx(true); return true; }
	void end() { stopListening(); }
	bool isListening() { return m_rxEnabled; }
	bool stopListening() { enableRx(false); return true; }

	void onReceive(std::function<void(int available)> handler);
	void perform_work();

	using Print::write;

private:
	void waitBitCycles(long unsigned deadline);
	bool isValidGPIOpin(int pin);
	void rxBits();

	// Member variables
	bool m_oneWire;
	int m_rxPin = SW_SERIAL_UNUSED_PIN;
	int m_txPin = SW_SERIAL_UNUSED_PIN;
	int m_txEnablePin = SW_SERIAL_UNUSED_PIN;
	bool m_rxValid = false;
	bool m_rxEnabled = false;
	bool m_txValid = false;
	bool m_txEnableValid = false;
	bool m_invert;
	bool m_overflow = false;
	long m_bitCycles;
	bool m_intTxEnabled;
	int m_inPos, m_outPos;
	int m_bufSize = 0;
	uint8_t *m_buffer = 0;
	// the ISR stores the relative bit times in the buffer. The inversion corrected level is used as sign bit (2's complement):
	// 1 = positive including 0, 0 = negative.
	volatile int m_isrInPos, m_isrOutPos;
	int m_isrBufSize = 0;
	volatile long unsigned *m_isrBuffer = 0;
	volatile bool m_isrOverflow = false;
	long unsigned m_lastCycle = 0;
	int m_rxCurBit; // 0 - 7: data bits. -1: start bit. 8: stop bit.
	uint8_t m_rxCurByte = 0;

	std::function<void(int available)> receiveHandler = 0;
};

#endif
