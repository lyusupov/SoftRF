// RH_ABZ.h
//
// Definitions for SX1276 radio in muRata CMWX1ZZABZ (TypeABZ) module
// as used in GrumpyOldPizza Grasshopper-L082CZ, EcoNode SmartTrap  etc with
// GrumpyOldPizza / ArduinoCore-stm32l0 installed per https://github.com/GrumpyOldPizza/ArduinoCore-stm32l0
//
// For data refer to muRata Application note: AN-ZZABZ-001
// muRata Preliminary Specification Number : SP-ABZ-093-E
// $p/EcoNode
//
// Author: Mike McCauley (mikem@airspayce.com)
// Copyright (C) 2020 Mike McCauley
// $Id: RH_ABZ.h,v 1.1 2020/06/15 23:39:39 mikem Exp $
// 
#ifndef RH_ABZ_h
#define RH_ABZ_h

#include <RH_RF95.h>

/////////////////////////////////////////////////////////////////////
/// \class RH_ABZ RH_ABZ.h <RH_ABZ.h>
/// \brief Driver to send and receive unaddressed, unreliable datagrams via 
/// radio transceiver in a muRata cmwx1zzabz module, which includes an STM32L0 processor,
/// a SX1276 LoRa radio and an antenna switch.
///
/// Requires the Grumpy Old Pizza Arduino Core installed per https://github.com/GrumpyOldPizza/ArduinoCore-stm32l0
///
/// Works with EcoNode SmartTrap, Tlera Grasshopper and family. Almost any board equipped with a muRata cmwx1zzabz module
/// should work. Tested with EcoNode SmartTrap, Arduino 1.8.9, GrumpyOldPizza Arduino Core for STM32L0.
/// When building for EcoNode SmartTrap in Arduino IDE, select board type Grasshopper-L082CZ.
/// This chip and GrumpyOldPizza Arduino Core for STM32L0 are now supported by PlatformIO: 
/// https://docs.platformio.org/en/latest/platforms/ststm32.html#arduino-stm32l0-configuration-system
///
/// \par Overview
///
/// This class is a specialisation of the RH_RF95 class, but with overridden functions to ensure that RH_RF95 communicates
/// with the SX1276 radio on the correct SPI interface (STM32L0_SPI_INSTANCE_SPI1). The class configures the SPI and
/// interfaces with the radio using native stm32l0 calls. It also uses stm32l0 calls to intialise the radio interface pins,
/// antenna interface pins, and to toggle the radio NSS pin for SPI communicaitons. The reason for this speicalisaiton is that
/// all the STM32L0 variants in the Grumpy Pizzas Arduino Core define the Arduino compatible SPI interface and the
/// Arduino compatible IO pins in varying and inconsistent ways. So we use native stm32l0 calls to make _sure_ we get the right
/// pins and interfaces.
///
/// All the comments in the RH_RF95 class concerning modulation, packet formats etc apply equally to this module.
///
/// \par Temperature Controlled Crystal Oscillator (TCXO)
///
/// The muRata cmwx1zzabz module includes a TCXO. Pins to enable the TCXO and to connect to 32MHz output to the radio
/// are exposed on the module. Some boards (Econode SmartTrap for example) permanently power the TCXO and permanenetly
/// connect it to the radio. Other boards (Grasshopper for example) have the TCXO enable connected to a GPIO pin, allowing
/// the TCXO to be controlled by software. Different boards may use different GPIO pins to control the TCXO.
///
/// The SX1276 radio can be configured to use the TCXO, and the Arduino Core defaults the radio to using TCXO.
/// Therefore it is important that you ensure the TCXO is powered up, at least when you want the radio to operate.
/// If the TCXO is not powered, the radio will not work.
///
/// On the Tlera boards supported the Arduino Core, you can call SX1276SetBoardTcxo() to enable or disable the TCXO
/// by controlling the correct pin for your board.
/// By default the core disables TCXO at the end of initialisation, so by the time your sketch starts to run
/// the TCXO is powered off.
/// You will almost certainly need to call
/// \code
/// SX1276SetBoardTcxo(true); 
/// \endcode
/// in your setup() or at other times when you want the radio to operate.
///
/// If you have a board where the TCXO is permanently powered, this is unnecessary.
///
/// \par Connecting and configuring the radio
///
/// There is no special configuration for the SX1276 radio in the  muRata cmwx1zzabz module: the CPU, radio and
/// antenna switch are all hardwired within the module can, and cannot be changed. Initialise the radio like this
/// with the default constructor:
/// \code
///  RH_ABZ driver;
/// \endcode
///
/// \par Range
///
/// We made some primitive range tests with 2 identical EcoNode SmartTrap at 868MHz, 20dBm transmit power with
/// modem config RH_RF95::Bw125Cr45Sf2048, using the abz_client and abz_server sketches included in this distribution.
/// We monitored for reliabilty of 2-way communications (ie how reliably can the client get a reply from the server,
/// which is a 2-way comms that depends on both send and receive. The SmartTrap has a simple small helical antenna.
/// The environment was a beach in a developed area, one node was stationary
/// on a rock about 1m above sand level. The mobile node was handl-held at 1 m above ground level. All measurements were line-of-sight.
///
/// \code
/// Location     Distance (km)        % successful round trip
/// Elephant Rock          0          100
/// Dune St                1.48       100
/// Shell St               1.71       100
/// Sand St                1.93       100
/// Sea St                 2.15         0
/// Short St               2.36        50
/// John St                2.59        50
/// Surf St                2.80       100
/// Matters St             2.98       100
/// Mills St               3.92       100
/// North Kirra            4.62         0
/// North Kirra SLSC       4.81        80
/// Haig St                5.20         0
/// Kirra SLSC             5.91         0
/// \endcode
///
/// \par Transmitter Power
///
/// We have made some actual power measurements against
/// programmed power on an EcoNode SmartTrap.
/// - EcoNode SmartTrap at 868MHz
/// - 15cm RG316 soldered direct to SmartTrap antenna pin
/// - SMA/BNC connector
/// - 12db attenuator (calibrated as 13.5dB at 868MHz)
/// - SMA/BNC connector
/// - 30cm RG316
/// - MiniKits AD8307 HF/VHF Power Head (calibrated against Rohde&Schwartz 806.2020 test set)
/// - Tektronix TDS220 scope to measure the Vout from power head
/// \code
/// useRFO==false (ie uses PA_BOOST for higher power)
/// Program power           Measured Power
///    dBm                         dBm
///      2                           2.7
///      5                           5
///      7                           6.5
///     10                           9.5
///     13                          12.5
///     15                          14.5
///     16                          15.5
///     17                          16.5 
///     18                          17.2 
///     19                          17.6 
///     20                          18.2
///
/// useRFO==true (ie no PA_BOOST)
/// Program power           Measured Power
///    dBm                         dBm
///    0                           -5.5
///    2                           -2.5
///    4                           -0.5
///    6                            2
///    8                            4
///   10                            6.5
///   12                            9
///   13                           10.5
///   14                           11.5
///   15                           12.5
/// \endcode

/// In the the Grumpy Old Pizza Arduino Core, there is a function for turning the
/// TCXO power source on and off, which depends on exactly which board is being compiled for
/// If the Radio in your boards has its TCXO connected to a programmable power pin,
/// and if you enable TCXO on the radio (by default it is on these boards)
/// then this function needs to be called to enable the TCXO before the radio will work.
extern "C" void SX1276SetBoardTcxo( bool state ); 

class RH_ABZ : public RH_RF95
{
public:
    /// Constructor
    RH_ABZ();

    /// Initialise the Driver transport hardware and software. Leaves the radio in idle mode,
    /// with default configuration of: 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
    /// \return true if initialisation succeeded.
    virtual bool    init();

    /// Deinitialise the interrupt handler, allowing the radio to be temporarily used by another stack.
    /// If you wish to use the radio again after this call, you wil need to call init() again.
    /// \return true if deinitialisation succeeded.
    bool deinit();
    
protected:
    /// Called by RH_RF95 when the radio mode is about to change to a new setting.
    /// Configures the antenna switch to connect to the right radio pin.
    /// \param[in] mode RHMode the new mode about to take effect
    /// \return true if the subclasses changes successful
    virtual bool modeWillChange(RHMode mode);

    /// Called by RHSPIDriver when the SPI is about to talk to the radio.
    /// Uses native spi32l0 calls to enable the radio NSS pin
    virtual void selectSlave();
    
    /// Called by RHSPIDriver when the SPI is finished talking to the radio.
    /// Uses native spi32l0 calls to disable the radio NSS pin
    virtual void deselectSlave();

private:
    /// Glue code between the DIO0 interrupt the interrupt handler in RH_RF95
    static void RH_INTERRUPT_ATTR isr();

    /// Pointer to the one and only instance permitted, for interrupt linkage
    static RH_ABZ* _thisDevice;

};

/// @example abz_client.ino
/// @example abz_server.ino

#endif
