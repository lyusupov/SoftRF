/*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//!
//! \defgroup EasyLink

//!
//! \ingroup EasyLink
//@{

//*****************************************************************************
//  @file       EasyLink.h
//
//  @brief      EasyLink RF API for CC13xx/CC26xx family
//
// # Overview #
// The EasyLink API should be used in application code. The EasyLink API is
// intended to abstract the RF Driver in order to give a simple API for
// customers to use as is or extend to suit their application[Use Cases]
// (@ref USE_CASES).
//
// # General Behavior #
// Before using the EasyLink API:
//   - The EasyLink Layer is initialized by calling EasyLink_init(). This
//     initialises and opens the RF driver and configuring a modulation scheme
//     passed to EasyLink_init.
//   - The RX and TX can operate independently of each other.
//   .
// The following is true for receive operation:
//   - RX is enabled by calling EasyLink_receive() or EasyLink_receiveAsync().
//   - Entering RX can be immediate or scheduled.
//   - EasyLink_receive() is blocking and EasyLink_receiveAsync() is nonblocking.
//   - the EasyLink API does not queue messages so calling another API function
//     while in EasyLink_receiveAsync() will return EasyLink_Status_Busy_Error
//   - an Async operation can be cancelled with EasyLink_abort()
//   .
// The following apply for transmit operation:
//   - TX is enabled by calling EasyLink_transmit() or EasyLink_transmitAsync().
//   - TX can be immediate or scheduled.
//   - EasyLink_transmit() is blocking and EasyLink_transmitAsync() is nonblocking
//   - EasyLink_transmit() for a scheduled command, or if TX can not start
//   - the EasyLink API does not queue messages so calling another API function
//     while in EasyLink_transmitAsync() will return EasyLink_Status_Busy_Error
//   - an Async operation can be cancelled with EasyLink_abort()
//
// # Error handling #
//    The EasyLink API will return EasyLink_Status containing success or error
//    code. The EasyLink_Status code are:
//    EasyLink_Status_Success
//    EasyLink_Status_Config_Error
//    EasyLink_Status_Param_Error
//    EasyLink_Status_Mem_Error
//    EasyLink_Status_Cmd_Error
//    EasyLink_Status_Tx_Error
//    EasyLink_Status_Rx_Error
//    EasyLink_Status_Rx_Timeout
//    EasyLink_Status_Busy_Error
//    EasyLink_Status_Aborted
//   .
//
// # Power Management #
// The TI-RTOS power management framework will try to put the device into the most
// power efficient mode whenever possible. Please see the technical reference
// manual for further details on each power mode.
//
// The EasyLink Layer uses the power management offered by the RF driver Refer to the RF
// drive documentation for more details.
//
// # Supported Functions #
// | Generic API function          | Description                                       |
// |-------------------------------|---------------------------------------------------|
// | EasyLink_init()               | Init's and opens the RF driver and configures the |
// |                               | specified modulation                              |
// | EasyLink_transmit()           | Blocking Transmit                                 |
// | EasyLink_transmitAsync()      | Nonblocking Transmit                              |
// | EasyLink_receive()            | Blocking Receive                                  |
// | EasyLink_receiveAsync()       | Nonblocking Receive                               |
// | EasyLink_abort()              | Aborts a non blocking call                        |
// | EasyLink_EnableRxAddrFilter() | Enables/Disables RX filtering on the Addr         |
// | EasyLink_GetIeeeAddr()        | Gets the IEEE Address                             |
// | EasyLink_SetFreq()            | Sets the frequency                                |
// | EasyLink_GetFreq()            | Gets the frequency                                |
// | EasyLink_SetRfPwr()           | Sets the Tx Power                                 |
// | EasyLink_GetRfPwr()           | Gets the Tx Power                                 |
//
// # Frame Structure #
// The EasyLink implements a basic header for transmitting and receiving data. This header supports
// addressing for a star or point-to-point network with acknowledgements.
//
// Packet structure:
//     _________________________________________________________
//    |           |                   |                         |
//    | 1B Length | 1-64b Dst Address |         Payload         |
//    |___________|___________________|_________________________|
//
//
//
//  # Not Supported Functionality #
//
//
//*****************************************************************************
#ifndef Easylink__include
#define Easylink__include

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

#define EASYLINK_API_VERSION "EasyLink-v2.00.00"

/// \brief defines the largest Tx/Rx payload that the interface can support
#define EASYLINK_MAX_DATA_LENGTH        128

/// \brief defines the Tx/Rx Max Address Size
#define EASYLINK_MAX_ADDR_SIZE        8

/// \brief defines the Max number of Rx Address filters
#define EASYLINK_MAX_ADDR_FILTERS     3

/// \brief macro to convert from Radio Time Ticks to ms
#define EasyLink_RadioTime_To_ms(radioTime) ((1000 * radioTime) / 4000000)

/// \brief macro to convert from ms to Radio Time Ticks
#define EasyLink_ms_To_RadioTime(ms) (ms*(4000000/1000))

/// \brief EasyLink Status and error codes
typedef enum
{
    EasyLink_Status_Success         = 0, ///Success
    EasyLink_Status_Config_Error    = 1, ///Configuration error
    EasyLink_Status_Param_Error     = 2, ///Param error
    EasyLink_Status_Mem_Error       = 3, ///Memory Error
    EasyLink_Status_Cmd_Error       = 4, ///Memory Error
    EasyLink_Status_Tx_Error        = 5, ///Tx Error
    EasyLink_Status_Rx_Error        = 6, ///Rx Error
    EasyLink_Status_Rx_Timeout      = 7, ///Rx Error
    EasyLink_Status_Rx_Buffer_Error = 8, ///Rx Buffer Error
    EasyLink_Status_Busy_Error      = 9, ///Busy Error
    EasyLink_Status_Aborted         = 10 ///Cmd stopped or aborted
} EasyLink_Status;


/// \brief Phy Type passed to EasyLink_init
typedef enum
{
    EasyLink_Phy_Custom = 0, ///Customer Phy specific settings exported from SmartRF Studio
    EasyLink_Phy_50kbps2gfsk = 1, ///Phy settings for Sub1G 50kbps data rate, IEEE 802.15.4g GFSK.
    EasyLink_Phy_625bpsLrm = 2, ///Phy settings for Sub1G 625bps data rate, Long Range Mode.
    EasyLink_Phy_2_4_200kbps2gfsk = 3, ///Phy settings for 2.4Ghz 200kbps data rate, IEEE 802.15.4g GFSK.
} EasyLink_PhyType;

/// \brief Advance configuration options
typedef enum
{
    EasyLink_Ctrl_AddSize = 0, ///Set the number of bytes in Addr for both Addr
    ///Filter and Tx/Rx operations
    EasyLink_Ctrl_Idle_TimeOut = 1, ///Set the time for Radio to return to
                                    ///idle after. Must be set before calling
                                    //EasyLink_init.
    EasyLink_Ctrl_MultiClient_Mode = 2, ///Set Multiclient mode for application
                                        ///that will use multiple RF clients.
                                        ///Must be set before calling
                                        ///EasyLink_init.
    EasyLink_Ctrl_AsyncRx_TimeOut = 3,  ///Relative time in ticks from Async
                                        ///Rx start to TimeOut. A value of
                                        ///0 means no timeout
    EasyLink_Ctrl_Test_Tone = 4, ///Enable/Disable Test mode for Tone
    EasyLink_Ctrl_Test_Signal = 5, ///Enable/Disable Test mode for Signal
} EasyLink_CtrlOption;

/// \brief Structure for the TX Packet
typedef struct
{
        uint8_t dstAddr[8];     /// Dst Address
        uint32_t absTime;        ///Absolute time to Tx packet (0 for immediate)
        ///Layer will use last SeqNum used + 1
        uint8_t len;             ///Payload Length
        uint8_t payload[EASYLINK_MAX_DATA_LENGTH];       ///Payload
} EasyLink_TxPacket;

/// \brief Structure for the RX'ed Packet
typedef struct
{
        uint8_t dstAddr[8];      ///Dst Address of RX'ed packet
        int8_t rssi;             ///rssi of RX'ed packet
        uint32_t absTime;        ///Absolute time to turn on Rx when passed
                                 ///(0 for immediate), Or Absolute time that packet was Rx
                                 ///when returned.
        uint32_t rxTimeout;      ///Relative time in ticks from Rx start to Rx TimeOut
                                 ///a value of 0 means no timeout
        uint8_t len;             ///length of RX'ed packet
        uint8_t payload[EASYLINK_MAX_DATA_LENGTH]; ///payload of RX'ed packet
} EasyLink_RxPacket;

/** \brief EasyLink Callback function type for Received packet, registered
 *   with EasyLink_ReceiveAsync
 */
typedef void (*EasyLink_ReceiveCb)(EasyLink_RxPacket * rxPacket,
        EasyLink_Status status);

/** \brief EasyLink Callback function type for Tx Done registered with
 *  EasyLink_TransmitAsync
 */
typedef void (*EasyLink_TxDoneCb)(EasyLink_Status status);

//*****************************************************************************
//
//! \brief Initializes the radio with specified Phy settings
//!
//! This function configures the radio phy settings. If the ui32ModType
//! is EasyLink_Phy_Custom then the configuration is taken from srf_settings.h.
//! If a specific phy configuration is required (and not supported by any of
//! the defined Phy types in EasyLink_PhyType then you can cut and past the
//! RF setting from the SmartRF Studio code export tool. This will copy and use
//! the RF_prop, RF_cmdPropRadioDivSetup and RF_cmdFs commands, as well as the
//! Synchword from the RF_cmdPropTx and RF_cmdPropRx commands.
//!
//! \param ui32ModType is a set to:
//! - \ref EasyLink_Phy_50kbps2gfsk
//! - \ref EasyLink_Phy_625bpsLrm
//! - \ref EasyLink_Phy_Custom
//!
//! \return EasyLink_Status
//
//*****************************************************************************
extern EasyLink_Status EasyLink_init(EasyLink_PhyType ui32ModType);

//*****************************************************************************
//
//! \brief Gets the absolute radio time
//!
//! This function returns the absolute radio time and can be used for
//! monitoring or Tx/Rx events using the EasyLink_TxPacket_t and
//! EasyLink_RxPacket_t absTime field.
//!
//!
//! \return absolute time
//
//*****************************************************************************
extern uint32_t EasyLink_getAbsTime(void);

//*****************************************************************************
//
//! \brief Sends a Packet with blocking call.
//!
//! This function is a blocking call to send a packet. If the Tx is
//! successfully scheduled then the function will block until the Tx is
//! complete.
//!
//! \param txPacket - The descriptor for the packet to be Tx'ed.
//!
//! \return EasyLink_Status
//
//*****************************************************************************
extern EasyLink_Status EasyLink_transmit(EasyLink_TxPacket *txPacket);

//*****************************************************************************
//
//! \brief Sends a Packet with non blocking call.
//!
//! This function is a non blocking call to send a packet. If the Tx is
//! successfully scheduled then the callback will be call once the Tx is
//! complete. The Tx will timeout if EasyLink_Ctrl_AsyncTx_TimeOut
//! ctrl message is used to set the timeout to something other than 0.
//!
//! \param txPacket - The descriptor for the packet to be Tx'ed.
//! \param cb     - The tx done function pointer.
//!
//! \return EasyLink_Status
//
//*****************************************************************************
extern EasyLink_Status EasyLink_transmitAsync(EasyLink_TxPacket *txPacket,
        EasyLink_TxDoneCb cb);

//*****************************************************************************
//
//! \brief Blocking call that waits for an Rx Packet.
//!
//! This function is a blocking call to wait for an Rx packet.
//!
//! \param rxPacket - The descriptor for the packet to be Rx'ed.
//!
//! \return EasyLink_Status
//
//*****************************************************************************
extern EasyLink_Status EasyLink_receive(EasyLink_RxPacket *rxPacket);

//*****************************************************************************
//
//! \brief Enables Asynchronous Packet Rx with non blocking call.
//!
//! This function is a non blocking call to Rx a packet. The Rx is turned on
//! and the Callback is called once a packet is received. The Rx will timeout
//! if EasyLink_Ctrl_AsyncRx_TimeOut ctrl message is used to set the timeout
//! to something other than 0.
//!
//! \param cb        - The rx function pointer.
//! \param absTime   - Start time of Rx (0: now !0: absolute radio time to
//!                    start Rx)
//!
//! \return EasyLink_Status
//
//*****************************************************************************
extern EasyLink_Status EasyLink_receiveAsync(EasyLink_ReceiveCb cb, uint32_t absTime);

//*****************************************************************************
//
//! \brief Abort a previously call Async Tx/Rx.
//!
//! This function is a blocking call to abort a previous Async Tx/Rx
//!
//! \return EasyLink_Status
//
//*****************************************************************************
extern EasyLink_Status EasyLink_abort(void);


//*****************************************************************************
//
//! \brief Sets the Frequency
//!
//! This function set the radio to the specified frequency. Note that this will
//! be rounded to the nearest frequency supported by the Frequency Synthesizer.
//!
//! \param ui16Freq Frequency in units of kHz
//!
//! \return EasyLink_Status
//
//*****************************************************************************
extern EasyLink_Status EasyLink_setFrequency(uint32_t ui16Freq);

//*****************************************************************************
//
//! \brief Gets the Frequency
//!
//! This function gets Frequency in units of kHz. This function will return the
//! value set in the Frequency Synthesizer, and may not be the same as set
//! using easyLink_setFrequency API. Note that this value does not include any
//! offsets for deviations due to factors such as temperature and hence this API
//! should not be used to get an accurate measure of frequency.

//!
//! \return Frequency in units of kHz
//
//*****************************************************************************
extern uint32_t EasyLink_getFrequency(void);

//*****************************************************************************
//
//! \brief Enables the address filter
//!
//! This function enables the address filter to filter out address that are not
//! in the address table provided.
//!
//! \param pui8AddrFilterTable A uint8 pointer to a variable size 2d array
//!  containing the addresses to filter on.
//! \param ui8AddrSize The size of the address elements
//! \param ui8NumAddrs The number of address elements
//!
//! \return EasyLink_Status
//
//*****************************************************************************
extern EasyLink_Status EasyLink_enableRxAddrFilter(uint8_t* pui8AddrFilterTable,
        uint8_t ui8AddrSize, uint8_t ui8NumAddrs);

//*****************************************************************************
//
//! \brief Gets the IEEE address
//!
//! This function gets the IEEE address
//!
//! \param ieeeAddr pointer to an 8 element byte array to write the IEEE
//! address to.
//!
//! \return EasyLink_Status
//
//*****************************************************************************
extern EasyLink_Status EasyLink_getIeeeAddr(uint8_t *ieeeAddr);

//*****************************************************************************
//
//! \brief Sets the TX Power
//!
//! This function sets the Tx Power
//!
//! \param i8Power the tx power in dBm's to be set. integers of -10 and between
//!        0-14 dBm are accepted. Values above 14 are rounded to 14 and below 0
//!        are rounded to -10
//!
//! \return EasyLink_Status
//
//*****************************************************************************
extern EasyLink_Status EasyLink_setRfPwr(int8_t i8Power);

//*****************************************************************************
//
//! \brief Gets the TX Power
//!
//! This function gets the Tx Power in dBm, values ranging from -10 to 14 dBm
//! should be expect
//!
//! \return power in dBm
//
//*****************************************************************************
extern int8_t EasyLink_getRfPwr(void);

//*****************************************************************************
//
//! \brief Sets advanced configuration options
//!
//! This function allows setting some of the advanced configuration options
//!
//! \param Ctrl - The control option to be set
//! \param ui32Value - The value to set the control option to
//
//! \return EasyLink_Status
//!
//*****************************************************************************
extern EasyLink_Status EasyLink_setCtrl(EasyLink_CtrlOption Ctrl,
        uint32_t ui32Value);

//*****************************************************************************
//
//! \brief Gets advanced configuration options
//!
//! This function allows getting some of the advanced configuration options
//!
//! \param Ctrl - The control option to get
//! \param pui32Value - Pointer to return the control option value
//!
//! \return EasyLink_Status
//!
//*****************************************************************************
extern EasyLink_Status EasyLink_getCtrl(EasyLink_CtrlOption Ctrl,
        uint32_t* pui32Value);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // Easylink__include

//*****************************************************************************
//
//! Close the Doxygen group.
//! @}
//
//*****************************************************************************
