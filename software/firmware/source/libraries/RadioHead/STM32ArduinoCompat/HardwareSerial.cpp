// ArduinoCompat/HardwareSerial.cpp
//
// Author: mikem@airspayce.com

#include <RadioHead.h>
#if (RH_PLATFORM == RH_PLATFORM_STM32STD)
#include <HardwareSerial.h>
#include <stm32f4xx_usart.h>

// Preinstantiated Serial objects
HardwareSerial Serial1(USART1);
HardwareSerial Serial2(USART2);
HardwareSerial Serial3(USART3);
HardwareSerial Serial4(UART4);
HardwareSerial Serial5(UART5);
HardwareSerial Serial6(USART6);

///////////////////////////////////////////////////////////////
// RingBuffer
///////////////////////////////////////////////////////////////

RingBuffer::RingBuffer()
    : _head(0),
      _tail(0),
      _overruns(0),
      _underruns(0)
{
}

bool    RingBuffer::isEmpty()
{
    return _head == _tail;
}	

bool    RingBuffer::isFull()
{
    return ((_head + 1) % ARDUINO_RINGBUFFER_SIZE) == _tail;
}

bool    RingBuffer::write(uint8_t ch)
{
    if (isFull())
    {
	_overruns++;
	return false;
    }
    _buffer[_head] = ch;
    if (++_head >= ARDUINO_RINGBUFFER_SIZE)
	_head = 0;
    return true;
}

uint8_t RingBuffer::read()
{
    if (isEmpty())
    {
	_underruns++;
	return 0; // What else can we do?
    }
    uint8_t ret = _buffer[_tail];
    if (++_tail >= ARDUINO_RINGBUFFER_SIZE)
	_tail = 0;
    return ret;
}

///////////////////////////////////////////////////////////////
// HardwareSerial
///////////////////////////////////////////////////////////////

// On STM32F4 Discovery, USART 1 is not very useful conflicts with the Green lED
HardwareSerial::HardwareSerial(USART_TypeDef* usart)
    : _usart(usart)
{
}

void HardwareSerial::begin(unsigned long baud)
{    
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef  GPIO_InitStructure_TX;
    GPIO_InitTypeDef  GPIO_InitStructure_RX;

    // Common GPIO structure init:
    GPIO_InitStructure_TX.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure_TX.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure_TX.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure_TX.GPIO_PuPd  = GPIO_PuPd_UP;

    GPIO_InitStructure_RX.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure_RX.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure_RX.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure_RX.GPIO_PuPd  = GPIO_PuPd_UP;
    // CTS or SCLK outputs are not supported.

    USART_InitStructure.USART_BaudRate            = baud * 25/8; // Why?
    // Only 8N1 is currently supported
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;  
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;   
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

    // Different for each USART:
    if (_usart == USART1)
    {
	// Initialise the clocks for this USART and its RX, TX pins port
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9,  GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

	GPIO_InitStructure_TX.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure_RX.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOA, &GPIO_InitStructure_TX);
	GPIO_Init(GPIOA, &GPIO_InitStructure_RX);
	// Initialise the USART
	USART_Init(USART1, &USART_InitStructure);
	// Enable the RXNE interrupt
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	// Enable global interrupt
	NVIC_EnableIRQ(USART1_IRQn);
    }
    else if (_usart == USART2)
    {
	// Initialise the clocks for this USART and its RX, TX pins port
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

	GPIO_InitStructure_TX.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure_RX.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStructure_TX);
	GPIO_Init(GPIOA, &GPIO_InitStructure_RX);
	// Initialise the USART
	USART_Init(USART2, &USART_InitStructure);
	// Enable the RXNE interrupt
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	// Enable global interrupt
	NVIC_EnableIRQ(USART2_IRQn);
    }
    else if (_usart == USART3)
    {
	// Initialise the clocks for this USART and its RX, TX pins port
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);

	GPIO_InitStructure_TX.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure_RX.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOD, &GPIO_InitStructure_TX);
	GPIO_Init(GPIOD, &GPIO_InitStructure_RX);
	// Initialise the USART
	USART_Init(USART3, &USART_InitStructure);
	// Enable the RXNE interrupt
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	// Enable global interrupt
	NVIC_EnableIRQ(USART3_IRQn);
    }
    else if (_usart == UART4)
    {
	// Initialise the clocks for this USART and its RX, TX pins port
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4);

	GPIO_InitStructure_TX.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure_RX.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOA, &GPIO_InitStructure_TX);
	GPIO_Init(GPIOA, &GPIO_InitStructure_RX);
	// Initialise the USART
	USART_Init(UART4, &USART_InitStructure);
	// Enable the RXNE interrupt
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
	// Enable global interrupt
	NVIC_EnableIRQ(UART4_IRQn);
    }
    else if (_usart == UART5)
    {
	// Initialise the clocks for this USART and its RX, TX pins port
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource2,  GPIO_AF_UART5);

	GPIO_InitStructure_TX.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure_RX.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOC, &GPIO_InitStructure_TX);
	GPIO_Init(GPIOD, &GPIO_InitStructure_RX);
	// Initialise the USART
	USART_Init(UART5, &USART_InitStructure);
	// Enable the RXNE interrupt
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
	// Enable global interrupt
	NVIC_EnableIRQ(UART5_IRQn);
    }
    else if (_usart == USART6)
    {
	// Initialise the clocks for this USART and its RX, TX pins port
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);

	GPIO_InitStructure_TX.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure_RX.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init(GPIOC, &GPIO_InitStructure_TX);
	GPIO_Init(GPIOC, &GPIO_InitStructure_RX);
	// Initialise the USART
	USART_Init(USART6, &USART_InitStructure);
	// Enable the RXNE interrupt
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
	// Enable global interrupt
	NVIC_EnableIRQ(USART6_IRQn);
    }

    USART_Cmd(_usart, ENABLE);
}

void HardwareSerial::end()
{
    USART_Cmd(_usart, DISABLE);
    USART_DeInit(_usart);
}

int HardwareSerial::available(void)
{
    return !_rxRingBuffer.isEmpty();
}

int HardwareSerial::read(void)
{
    return _rxRingBuffer.read();
}

size_t HardwareSerial::write(uint8_t ch)
{
    _txRingBuffer.write(ch); // Queue it
    USART_ITConfig(_usart, USART_IT_TXE, ENABLE); // Enable the TX interrupt
    return 1;
}

extern "C"
{
    void USART1_IRQHandler(void)
    {
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
	    Serial1._rxRingBuffer.write(USART_ReceiveData(USART1));
	}
	if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
	{
	    // Transmitter is empty, maybe send another char?
	    if (Serial1._txRingBuffer.isEmpty())
		USART_ITConfig(USART1, USART_IT_TXE, DISABLE); // No more to send, disable the TX interrupt
	    else
		USART_SendData(USART1, Serial1._txRingBuffer.read());
	}
    }
    void USART2_IRQHandler(void)
    {
	if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
	    // Newly received char, try to put it in our rx buffer
	    Serial2._rxRingBuffer.write(USART_ReceiveData(USART2));
	}
	if (USART_GetITStatus(USART2, USART_IT_TXE) != RESET)
	{
	    // Transmitter is empty, maybe send another char?
	    if (Serial2._txRingBuffer.isEmpty())
		USART_ITConfig(USART2, USART_IT_TXE, DISABLE); // No more to send, disable the TX interrupt
	    else
		USART_SendData(USART2, Serial2._txRingBuffer.read());
	}
    }
    void USART3_IRQHandler(void)
    {
	if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
	    // Newly received char, try to put it in our rx buffer
	    Serial3._rxRingBuffer.write(USART_ReceiveData(USART3));
	}
	if (USART_GetITStatus(USART3, USART_IT_TXE) != RESET)
	{
	    // Transmitter is empty, maybe send another char?
	    if (Serial3._txRingBuffer.isEmpty())
		USART_ITConfig(USART3, USART_IT_TXE, DISABLE); // No more to send, disable the TX interrupt
	    else
		USART_SendData(USART3, Serial3._txRingBuffer.read());
	}
    }
    void UART4_IRQHandler(void)
    {
	if (USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
	{
	    // Newly received char, try to put it in our rx buffer
	    Serial4._rxRingBuffer.write(USART_ReceiveData(UART4));
	}
	if (USART_GetITStatus(UART4, USART_IT_TXE) != RESET)
	{
	    // Transmitter is empty, maybe send another char?
	    if (Serial4._txRingBuffer.isEmpty())
		USART_ITConfig(UART4, USART_IT_TXE, DISABLE); // No more to send, disable the TX interrupt
	    else
		USART_SendData(UART4, Serial4._txRingBuffer.read());
	}
    }
    void UART5_IRQHandler(void)
    {
	if (USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)
	{
	    // Newly received char, try to put it in our rx buffer
	    Serial5._rxRingBuffer.write(USART_ReceiveData(UART5));
	}
	if (USART_GetITStatus(UART5, USART_IT_TXE) != RESET)
	{
	    // Transmitter is empty, maybe send another char?
	    if (Serial5._txRingBuffer.isEmpty())
		USART_ITConfig(UART5, USART_IT_TXE, DISABLE); // No more to send, disable the TX interrupt
	    else
		USART_SendData(UART5, Serial5._txRingBuffer.read());
	}
    }
    void USART6_IRQHandler(void)
    {
	if (USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)
	{
	    // Newly received char, try to put it in our rx buffer
	    Serial6._rxRingBuffer.write(USART_ReceiveData(USART6));
	}
	if (USART_GetITStatus(USART6, USART_IT_TXE) != RESET)
	{
	    // Transmitter is empty, maybe send another char?
	    if (Serial6._txRingBuffer.isEmpty())
		USART_ITConfig(USART6, USART_IT_TXE, DISABLE); // No more to send, disable the TX interrupt
	    else
		USART_SendData(USART6, Serial6._txRingBuffer.read());
	}
    }
}

#endif
