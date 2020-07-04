/*
  HardwareSerial.cpp - Hardware serial library for Wiring
  Copyright (c) 2015 Hristo Gochkov.  All right reserved.

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

#ifdef RASPBERRY_PI
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "Arduino.h"

#include "HardwareSerial.h"
#include "cbuf.h"


#define UART_BUFFER_SIZE 1024

static cbuf _rx_buffer(1024);
static volatile bool _uart_check_run = false;

//called by the interrupt check thread
void uart_check_fifos(){
  if(!_uart_check_run) return;
  while((UART0FR & (1 << UART0RXFE)) == 0)
    _rx_buffer.write(UART0DR);
}

size_t HardwareSerial::write(uint8_t c){
  while(UART0FR & _BV(UART0TXFF));
  UART0DR = c;
  return 1;
}

void HardwareSerial::begin(uint32_t baud){
  uint32_t divider = 12000000/baud;
  pinMode(14, GPF0); // TXD
  pinMode(15, GPF0); // RXD
  UART0IBRD = divider >> 6;
  UART0FBRD = divider & 0x3F;
  UART0LCRH = (UART0WLEN_8BIT << UART0WLEN) | _BV(UART0FEN);
  UART0CR   = _BV(UART0EN) | _BV(UART0TXE) | _BV(UART0RXE);
  _uart_check_run = true;
}

void HardwareSerial::end(){
  _uart_check_run = false;
  UART0LCRH = 0;
  UART0CR = 0;
  pinMode(14, GPFI); // TXD
  pinMode(15, GPFI); // RXD
}

int HardwareSerial::available(void){
  return _rx_buffer.getSize();
}

int HardwareSerial::peek(void){
  return _rx_buffer.peek();
}

int HardwareSerial::read(void){
  return _rx_buffer.read();
}

void HardwareSerial::flush(){
  _rx_buffer.flush();
}

HardwareSerial Serial1;

#endif // RASPBERRY_PI


