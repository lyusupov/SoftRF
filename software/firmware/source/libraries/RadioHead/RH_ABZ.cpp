// RH_ABZ.cpp
//
// Copyright (C) 2020 Mike McCauley
// $Id: RH_ABZ.cpp,v 1.1 2020/06/15 23:39:39 mikem Exp $

#if (RH_PLATFORM == RH_PLATFORM_STM32L0) && (defined STM32L082xx || defined STM32L072xx)

#include <RH_ABZ.h>

// Pointer to the _only_ permitted ABZ instance (there is only one radio connected to this device)
RH_ABZ* RH_ABZ::_thisDevice;

// The muRata cmwx1zzabz module has its builtin SX1276 radio connected to the processor's SPI1 port,
// but the Arduino compatible SPI interface in Grumpy Pizzas Arduino Core is configured for SPI1 or SPI2
// depending on the exact board variant selected.
// So here we define our own Arduino compatible SPI interface
// so we are _sure_ to get the one connected to the radio, independent of the board variant selected
#include <stm32l0_spi.h>
static const stm32l0_spi_params_t RADIO_SPI_PARAMS = {
    STM32L0_SPI_INSTANCE_SPI1,
    0,
    STM32L0_DMA_CHANNEL_NONE,
    STM32L0_DMA_CHANNEL_NONE,
    {
        STM32L0_GPIO_PIN_PA7_SPI1_MOSI,
        STM32L0_GPIO_PIN_PA6_SPI1_MISO,
        STM32L0_GPIO_PIN_PB3_SPI1_SCK,
        STM32L0_GPIO_PIN_NONE,
    },
};

// Create and configure an Arduino compatible SPI interface. This will be referred to in RHHardwareSPI.cpp
// and used as the SPI interface to the radio.
static stm32l0_spi_t RADIO_SPI;
SPIClass radio_spi(&RADIO_SPI, &RADIO_SPI_PARAMS);

// Glue code between the 'C' DIO0 interrupt and the C++ interrupt handler in RH_RF95
void RH_INTERRUPT_ATTR RH_ABZ::isr()
{
    _thisDevice->handleInterrupt();
}

RH_ABZ::RH_ABZ():
    RH_RF95(RH_INVALID_PIN, RH_INVALID_PIN)
{
}

bool RH_ABZ::init()
{
    _thisDevice = this;

    // REVISIT: RESET THE RADIO???
    
    // The SX1276 radio DIO0 is connected to STM32 pin PB4
    // It will later be configured as an interrupt
    stm32l0_gpio_pin_configure(STM32L0_GPIO_PIN_PB4,     (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_PULLDOWN | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_INPUT));

    // Here we configure the interrupt handler for DIO0 to call the C++
    // interrupt handler in RH_RF95, in a roundabout way
#ifdef STM32L0_EXTI_CONTROL_PRIORITY_CRITICAL
    stm32l0_exti_attach(STM32L0_GPIO_PIN_PB4, (STM32L0_EXTI_CONTROL_PRIORITY_CRITICAL | STM32L0_EXTI_CONTROL_EDGE_RISING), (stm32l0_exti_callback_t)isr, NULL); // STM32L0_EXTI_CONTROL_PRIORITY_CRITICAL not in 0.0.10
#else
    stm32l0_exti_attach(STM32L0_GPIO_PIN_PB4, STM32L0_EXTI_CONTROL_EDGE_RISING, (stm32l0_exti_callback_t)isr, NULL);
#endif
    // The SX1276 radio slave select (NSS) is connected to STM32 pin PA15
    stm32l0_gpio_pin_configure(STM32L0_GPIO_PIN_PA15,      (STM32L0_GPIO_PARK_HIZ | STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_OUTPUT));
    
    // muRata cmwx1zzabz module has an antenna switch which must be driven the right way to connect the antenna
    // to the appropriate SX1276 pins.
    // Antenna switch might be something like NJG180K64, but not sure.
    // See Application note: AN-ZZABZ-001 P. 20/20
    // in typeABZ_hardware_design_guide_revC.pdf
    // with 3 pins connected to  STM32L0_GPIO_PIN_PA1, STM32L0_GPIO_PIN_PC2, STM32L0_GPIO_PIN_PC1
    // which select RX, RFO or PA_BOOST respecitvely
    // See modeWillChange() for implementation of pin twiddling when the transmitter is on
    //
    // We use native STM32 calls because the various different variants in the Grumpy Pizza
    // Arduino core and various forks of that core have inconsistent definitions of the Arduino compatible
    // pins. We want to be sure we get the right ones for the muRata modules connections to the Radio
    stm32l0_gpio_pin_configure(STM32L0_GPIO_PIN_PA1, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_OSPEED_LOW | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_OUTPUT));
    stm32l0_gpio_pin_configure(STM32L0_GPIO_PIN_PC2, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_OSPEED_LOW | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_OUTPUT));
    stm32l0_gpio_pin_configure(STM32L0_GPIO_PIN_PC1, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_OSPEED_LOW | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_OUTPUT));

    return RH_RF95::init();
}

bool RH_ABZ::deinit()
{
    setModeIdle();
    stm32l0_exti_detach(STM32L0_GPIO_PIN_PB4);
    return true;
}

void  RH_ABZ::selectSlave()
{
    stm32l0_gpio_pin_write(STM32L0_GPIO_PIN_PA15, 0);
}

void  RH_ABZ::deselectSlave()
{
   stm32l0_gpio_pin_write(STM32L0_GPIO_PIN_PA15, 1);
}
    
bool RH_ABZ::modeWillChange(RHMode mode)
{
    if (mode == RHModeTx)
    {
	// Tell the antenna switch to connect to one of the transmitter output pins
	stm32l0_gpio_pin_write(STM32L0_GPIO_PIN_PA1, 0);                // RX
	stm32l0_gpio_pin_write(STM32L0_GPIO_PIN_PC2, _useRFO ? 1 : 0);  // RFO
	stm32l0_gpio_pin_write(STM32L0_GPIO_PIN_PC1, _useRFO ? 0 : 1);  // BOOST
    }
    else
    {
	// Enabling the RX from the antenna switch improves reception RSSI by about 5
	stm32l0_gpio_pin_write(STM32L0_GPIO_PIN_PA1, 1); // RX 
	stm32l0_gpio_pin_write(STM32L0_GPIO_PIN_PC2, 0); // RFO
	stm32l0_gpio_pin_write(STM32L0_GPIO_PIN_PC1, 0); // BOOST
    }
    return true;
}

#endif
