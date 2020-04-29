/*
  EasyLink.h - Library for transmitting / receiving packets
  on the CC13xx using the EasyLink SDK.
  Created by Robert Wessels and Tony Cave, July 8, 2016.
  Released into the public domain.
*/

#include "Energia.h"
#include "EasyLink.h"

//#define EASYLINK_DEBUG 1

#if defined(EASYLINK_DEBUG)
#define DEBUG_START() Serial.begin(115200)
#define DEBUG(c) Serial.print(c)
#define DEBUGLN(l) Serial.println(l)
#else
#define DEBUG_START
#define DEBUG(c)
#define DEBUGLN(l)
#endif

struct ring_buffer
{
	unsigned char buffer[EASYLINK_MAX_DATA_LENGTH];
	volatile unsigned int head;
	volatile unsigned int tail;
};


ring_buffer rx_buffer  =  { { 0 }, 0, 0 };
ring_buffer tx_buffer  =  { { 0 }, 0, 0 };

EasyLink::EasyLink()
{
    _rx_buffer = &rx_buffer;
    _tx_buffer = &tx_buffer;
}

const char * EasyLink::version()
{
    return EASYLINK_API_VERSION;
}

EasyLink_Status EasyLink::begin(EasyLink_PhyType mode)
{
#ifdef ENERGIA_ARCH_CC13X2
   EasyLink_Params easyLink_params;
   EasyLink_Params_init(&easyLink_params);
   easyLink_params.ui32ModType = mode;
   return EasyLink_init(&easyLink_params);
#else
   return EasyLink_init(mode);
#endif
}

void EasyLink::beginTransmission(uint8_t dst) {
    _tx_buffer->head = 0;
    _tx_buffer->tail = 0;
    memset(dstAddr, 0, 8);
    dstAddr[0] = dst;
}

EasyLink_Status EasyLink::transmit(EasyLink_TxPacket *txPacket, EasyLink_TxDoneCb handle)
{

    if(txPacket == NULL || txPacket->len > EASYLINK_MAX_DATA_LENGTH) {
        return EasyLink_Status_Param_Error;
    }

    if(handle != NULL) {
        return EasyLink_transmitAsync(txPacket, handle);
    } else {
        return EasyLink_transmit(txPacket);
    }
}

EasyLink_Status EasyLink::endTransmission(EasyLink_TxDoneCb txCallback) {
    EasyLink_TxPacket txPacket;

    memcpy(&txPacket, dstAddr, 8);
    memcpy(&txPacket.payload, _tx_buffer->buffer, _tx_buffer->head);
    txPacket.absTime = 0;
    txPacket.len = _tx_buffer->head;

    if(txCallback != NULL) {
        return EasyLink_transmitAsync(&txPacket, txCallback);
    } else {
        return EasyLink_transmit(&txPacket);
    }
}

EasyLink_Status EasyLink::receive(EasyLink_ReceiveCb handle)
{
    /* Perform nonblocking receive */
    if(handle != NULL) {
        return EasyLink_receiveAsync(handle, 0);
    }

     return EasyLink_Status_Param_Error;
}

EasyLink_Status EasyLink::receive(EasyLink_RxPacket *rxPacket, uint32_t timeout)
{
    if(rxPacket == NULL) {
        return EasyLink_Status_Param_Error;
    }

    if(timeout) {
        rxPacket->rxTimeout = EasyLink_ms_To_RadioTime(2000);
        rxPacket->absTime = EasyLink_ms_To_RadioTime(0);
    }

    return EasyLink_receive(rxPacket);
}

EasyLink_Status EasyLink::receive(void (*userFunc)(void))
{
    memset(_rx_buffer, 0, sizeof(ring_buffer));
    EasyLink_Status status;
    EasyLink_RxPacket rxPacket;

    if(userFunc == NULL) {
        status = EasyLink_receive(&rxPacket);
        if (status != EasyLink_Status_Success) {
            return status;
        }

        memcpy(_rx_buffer->buffer, &rxPacket.payload, rxPacket.len);
        _rx_buffer->head = rxPacket.len;
        return status;
    }

    return EasyLink_Status_Success;
}
/*
 * Stream class virtual functions implementations
 */

void EasyLink::flush()
{
    while (_tx_buffer->head != _tx_buffer->tail);
}

size_t EasyLink::write(uint8_t c)
{
    /* If the buffer is full, indicate this by returning 0 bytes written */
    if(_tx_buffer->head + 1 > EASYLINK_MAX_DATA_LENGTH) {
        return 0;
    }

    _tx_buffer->buffer[_tx_buffer->head] = c;
    _tx_buffer->head++;
    return 1;
}

int EasyLink::available(void)
{
    return (unsigned int)(EASYLINK_MAX_DATA_LENGTH + _rx_buffer->head - _rx_buffer->tail) % EASYLINK_MAX_DATA_LENGTH;
}

int EasyLink::peek(void)
{
    if (_rx_buffer->head == _rx_buffer->tail) {
        return -1;
    } else {
        return _rx_buffer->buffer[_rx_buffer->tail];
    }
}

int EasyLink::read(void)
{
    // if the head isn't ahead of the tail, we don't have any characters
    if (_rx_buffer->head == _rx_buffer->tail) {
        return -1;
    } else {
        unsigned char c = _rx_buffer->buffer[_rx_buffer->tail];
        _rx_buffer->tail = (unsigned int)(_rx_buffer->tail + 1) % EASYLINK_MAX_DATA_LENGTH;
        return c;
    }
}
