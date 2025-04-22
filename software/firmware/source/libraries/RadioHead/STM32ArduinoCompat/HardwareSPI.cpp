// ArduinoCompat/HardwareSPI.cpp
//
// Interface between Arduino-like SPI interface and STM32F4 Discovery and similar
// using STM32F4xx_DSP_StdPeriph_Lib_V1.3.0

#include <RadioHead.h>
#if (RH_PLATFORM == RH_PLATFORM_STM32STD)

#include <wirish.h>
#include <HardwareSPI.h>
#include "stm32f4xx.h"
#include "stm32f4xx_spi.h"
extern "C"
{
#include "gdb_stdio.h"
}

// Symbolic definitions for the SPI pins we intend to use
// Currently we only support SPI1
#define SPIx                           SPI1
#define SPIx_CLK                       RCC_APB2Periph_SPI1
#define SPIx_CLK_INIT                  RCC_APB2PeriphClockCmd
#define SPIx_IRQn                      SPI2_IRQn
#define SPIx_IRQHANDLER                SPI2_IRQHandler

#define SPIx_SCK_PIN                   GPIO_Pin_5
#define SPIx_SCK_GPIO_PORT             GPIOA
#define SPIx_SCK_GPIO_CLK              RCC_AHB1Periph_GPIOA
#define SPIx_SCK_SOURCE                GPIO_PinSource5
#define SPIx_SCK_AF                    GPIO_AF_SPI1

#define SPIx_MISO_PIN                  GPIO_Pin_6
#define SPIx_MISO_GPIO_PORT            GPIOA
#define SPIx_MISO_GPIO_CLK             RCC_AHB1Periph_GPIOA
#define SPIx_MISO_SOURCE               GPIO_PinSource6
#define SPIx_MISO_AF                   GPIO_AF_SPI1

#define SPIx_MOSI_PIN                  GPIO_Pin_7
#define SPIx_MOSI_GPIO_PORT            GPIOA
#define SPIx_MOSI_GPIO_CLK             RCC_AHB1Periph_GPIOA
#define SPIx_MOSI_SOURCE               GPIO_PinSource7
#define SPIx_MOSI_AF                   GPIO_AF_SPI1

HardwareSPI::HardwareSPI(uint32_t spiPortNumber) :
    _spiPortNumber(spiPortNumber)
{
}

void HardwareSPI::begin(SPIFrequency frequency, uint32_t bitOrder, uint32_t mode)
{
  GPIO_InitTypeDef GPIO_InitStructure;
//  NVIC_InitTypeDef NVIC_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;

  /* Peripheral Clock Enable -------------------------------------------------*/
  /* Enable the SPI clock */
  RCC_APB2PeriphClockCmd(SPIx_CLK, ENABLE);
  
  /* Enable GPIO clocks */
  RCC_AHB1PeriphClockCmd(SPIx_SCK_GPIO_CLK | SPIx_MISO_GPIO_CLK | SPIx_MOSI_GPIO_CLK, ENABLE);

  /* SPI GPIO Configuration --------------------------------------------------*/
  /* GPIO Deinitialisation */
  GPIO_DeInit(SPIx_SCK_GPIO_PORT);
  GPIO_DeInit(SPIx_MISO_GPIO_PORT);
  GPIO_DeInit(SPIx_MOSI_GPIO_PORT);
  
  /* Connect SPI pins to AF5 */  
  GPIO_PinAFConfig(SPIx_SCK_GPIO_PORT, SPIx_SCK_SOURCE, SPIx_SCK_AF);
  GPIO_PinAFConfig(SPIx_MISO_GPIO_PORT, SPIx_MISO_SOURCE, SPIx_MISO_AF);    
  GPIO_PinAFConfig(SPIx_MOSI_GPIO_PORT, SPIx_MOSI_SOURCE, SPIx_MOSI_AF);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;

  /* SPI SCK pin configuration */
  GPIO_InitStructure.GPIO_Pin = SPIx_SCK_PIN;
  GPIO_Init(SPIx_SCK_GPIO_PORT, &GPIO_InitStructure);
  
  /* SPI  MISO pin configuration */
  GPIO_InitStructure.GPIO_Pin =  SPIx_MISO_PIN;
  GPIO_Init(SPIx_MISO_GPIO_PORT, &GPIO_InitStructure);  

  /* SPI  MOSI pin configuration */
  GPIO_InitStructure.GPIO_Pin =  SPIx_MOSI_PIN;
  GPIO_Init(SPIx_MOSI_GPIO_PORT, &GPIO_InitStructure);
 
  /* SPI configuration -------------------------------------------------------*/
  SPI_I2S_DeInit(SPIx);
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  if (mode == SPI_MODE0)
  {
      SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
      SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  }
  else if (mode == SPI_MODE1)
  {
      SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
      SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  }
  else if (mode == SPI_MODE2)
  {
      SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
      SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  }
  else if (mode == SPI_MODE3)
  {
      SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
      SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  }

  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  // Prescaler is divided into PCLK2 (84MHz) to get SPI baud rate/clock speed
  // 256 => 328.125kHz
  // 128 => 656.25kHz
  // 64 => 1.3125MHz
  // 32 => 2.625MHz
  // 16 => 5.25MHz
  // 8  => 10.5MHz
  // 4  => 21.0MHz
  switch (frequency)
  {
      case SPI_21_0MHZ:
	  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
	  break;
      case SPI_10_5MHZ:
	  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	  break;
      case SPI_5_25MHZ:
	  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
	  break;
      case SPI_2_625MHZ:
      default:
	  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
	  break;
      case SPI_1_3125MHZ:
	  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
	  break;
      case SPI_656_25KHZ:
	  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
	  break;
      case SPI_328_125KHZ:
	  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	  break;
	  
  }

  if (bitOrder == LSBFIRST)
      SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_LSB;
  else
      SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;

  /* Initializes the SPI communication */
  SPI_Init(SPIx, &SPI_InitStructure);
  /* Enable SPI1  */
  SPI_Cmd(SPIx, ENABLE);
}

void HardwareSPI::end(void)
{
    SPI_DeInit(SPIx);
}

uint8_t HardwareSPI::transfer(uint8_t data)
{
    // Wait for TX empty
    while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET)
	;
    SPI_SendData(SPIx, data);
    // Wait for RX not empty
    while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET)
	;
    return SPI_ReceiveData(SPIx);
}

#endif
