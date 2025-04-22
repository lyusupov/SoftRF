// RHCRC.h
//
// Definitions for RadioHead compatible CRC outines.
//
// These routines originally derived from Arduino source code. See RHCRC.cpp
// for copyright information
// $Id: RHCRC.h,v 1.1 2014/06/24 02:40:12 mikem Exp $

#ifndef RHCRC_h
#define RHCRC_h

#include <RadioHead.h>

extern uint16_t RHcrc16_update(uint16_t crc, uint8_t a);
extern uint16_t RHcrc_xmodem_update (uint16_t crc, uint8_t data);
extern uint16_t RHcrc_ccitt_update (uint16_t crc, uint8_t data);
extern uint8_t  RHcrc_ibutton_update(uint8_t crc, uint8_t data);

#endif
