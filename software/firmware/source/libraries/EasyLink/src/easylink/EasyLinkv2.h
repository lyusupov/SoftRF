#ifdef ENERGIA_ARCH_CC13X2
/*
 * Copyright (c) 2015-2019, Texas Instruments Incorporated
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

/*!
\defgroup EasyLink EasyLink

@{

@file       EasyLink.h

@brief      EasyLink RF API for CC13xx/CC26xx family

# Overview #
The EasyLink API should be used in application code. The EasyLink API is
intended to abstract the RF Driver in order to give a simple API for
customers to use as is or extend to suit their application use cases.

# General Behavior #
Before using the EasyLink API:
- The EasyLink Layer is initialized by calling EasyLink_init(). This
  initializes and opens the RF driver and configuring a modulation scheme
  passed to EasyLink_init().
- The RX and TX can operate independently of each other.

The following is true for receive operation:
- RX is enabled by calling EasyLink_receive() or EasyLink_receiveAsync().
- Entering RX can be immediate or scheduled.
- EasyLink_receive() is blocking and EasyLink_receiveAsync() is nonblocking.
- the EasyLink API does not queue messages so calling another API function
  while in EasyLink_receiveAsync() will return ::EasyLink_Status_Busy_Error
- an Async operation can be cancelled with EasyLink_abort()

The following apply for transmit operation:
- TX is enabled by calling EasyLink_transmit() or EasyLink_transmitAsync().
- TX can be immediate or scheduled.
- EasyLink_transmit() is blocking and EasyLink_transmitAsync() is nonblocking
- EasyLink_transmit() for a scheduled command, or if TX can not start
- the EasyLink API does not queue messages so calling another API function
  while in EasyLink_transmitAsync() will return ::EasyLink_Status_Busy_Error
- an Async operation can be cancelled with EasyLink_abort()

# Error handling #
The EasyLink API will return ::EasyLink_Status containing success or error
code. The EasyLink_Status code are:

- EasyLink_Status_Success
- EasyLink_Status_Config_Error
- EasyLink_Status_Param_Error
- EasyLink_Status_Mem_Error
- EasyLink_Status_Cmd_Error
- EasyLink_Status_Tx_Error
- EasyLink_Status_Rx_Error
- EasyLink_Status_Rx_Timeout
- EasyLink_Status_Busy_Error
- EasyLink_Status_Aborted
- EasyLink_Status_Cmd_Rejected

# Power Management #
The TI-RTOS power management framework will try to put the device into the most
power efficient mode whenever possible. Please see the technical reference
manual for further details on each power mode.

The EasyLink Layer uses the power management offered by the RF driver Refer to the RF
drive documentation for more details.

\htmlonly

# Supported Functions #


| Generic API function          | Description                                        |
|-------------------------------|----------------------------------------------------|
| EasyLink_init()               | Init's and opens the RF driver and configures the  |
|                               | specified settings based on EasyLink_Params struct |
| EasyLink_transmit()           | Blocking Transmit                                  |
| EasyLink_transmitAsync()      | Non-blocking Transmit                              |
| EasyLink_transmitCcaAsync()   | Non-blocking Transmit with Clear Channel Assessment|
| EasyLink_receive()            | Blocking Receive                                   |
| EasyLink_receiveAsync()       | Nonblocking Receive                                |
| EasyLink_abort()              | Aborts a non blocking call                         |
| EasyLink_enableRxAddrFilter() | Enables/Disables RX filtering on the Addr          |
| EasyLink_getIeeeAddr()        | Gets the IEEE Address                              |
| EasyLink_setFrequency()       | Sets the frequency                                 |
| EasyLink_getFrequency()       | Gets the frequency                                 |
| EasyLink_setRfPower()         | Sets the Tx Power                                  |
| EasyLink_getRfPower()         | Gets the Tx Power                                  |
| EasyLink_getRssi()            | Gets the RSSI                                      |
| EasyLink_getAbsTime()         | Gets the absolute time in RAT ticks                |
| EasyLink_setCtrl()            | Set RF parameters, test modes or EasyLink options  |
| EasyLink_getCtrl()            | Get RF parameters or EasyLink options              |

\endhtmlonly

# Frame Structure #
The EasyLink implements a basic header for transmitting and receiving data. This header supports
addressing for a star or point-to-point network with acknowledgments.

Packet structure:

\verbatim
 _________________________________________________________
|           |                   |                         |
| 1B Length | 1-64b Dst Address |         Payload         |
|___________|___________________|_________________________|

\endverbatim

*/


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
#include <ti/drivers/rf/RF.h>
#include <stdlib.h>
#include "../syscfg/ti_easylink_config.h"

//! \brief EasyLink API Version
#define EASYLINK_API_VERSION "EasyLink-v2.80.00"

//! \brief defines the Tx/Rx Max Address Size
#define EASYLINK_MAX_ADDR_SIZE              8

//! \brief defines the Max number of Rx Address filters
#define EASYLINK_MAX_ADDR_FILTERS           8

//! \brief defines the whitening mode
#define EASYLINK_WHITENING_MODE             2

//! \brief macro to convert from Radio Time Ticks to ms
#define EasyLink_RadioTime_To_ms(radioTime) (radioTime / (4000000/1000))

//! \brief macro to convert from ms to Radio Time Ticks
#define EasyLink_ms_To_RadioTime(ms) (ms*(4000000/1000))

//! \brief macro to convert from us to Radio Time Ticks
#define EasyLink_us_To_RadioTime(us) (us*(4000000/1000000))

//! \brief EasyLink Status and error codes
typedef enum
{
    EasyLink_Status_Success         = 0, //!< Success
    EasyLink_Status_Config_Error    = 1, //!< Configuration error
    EasyLink_Status_Param_Error     = 2, //!< Param error
    EasyLink_Status_Mem_Error       = 3, //!< Memory Error
    EasyLink_Status_Cmd_Error       = 4, //!< Memory Error
    EasyLink_Status_Tx_Error        = 5, //!< Tx Error
    EasyLink_Status_Rx_Error        = 6, //!< Rx Error
    EasyLink_Status_Rx_Timeout      = 7, //!< Rx Error
    EasyLink_Status_Rx_Buffer_Error = 8, //!< Rx Buffer Error
    EasyLink_Status_Busy_Error      = 9, //!< Busy Error
    EasyLink_Status_Aborted         = 10, //!< Command stopped or aborted
    EasyLink_Status_Cmd_Rejected    = 11, //!< Command Rejected by RF Driver (Scheduling conflict)
} EasyLink_Status;


//! \brief Phy Type defines so we can use them in checks
#define EasyLink_PHY_CUSTOM             0
#define EasyLink_PHY_50KBPS2GFSK        1
#define EasyLink_PHY_625BPSLRM          2
#define EasyLink_PHY_2_4_200KBPS2GFSK   3
#define EasyLink_PHY_5KBPSSLLR          4
#define EasyLink_PHY_2_4_100KBPS2GFSK   5
#define EasyLink_PHY_2_4_250KBPS2GFSK   6
#define EasyLink_PHY_200KBPS2GFSK       7
#define EasyLink_PHY_100KBPS2GFSK_OGN   8
#define EasyLink_PHY_100KBPS2GFSK_LEG   9
#define EasyLink_PHY_38400BPS2GFSK_P3I  10

//! \brief Phy Type passed to EasyLink_init()
typedef enum
{
    EasyLink_Phy_Custom = EasyLink_PHY_CUSTOM,            //!< Customer Phy specific settings exported from SmartRF Studio
    EasyLink_Phy_50kbps2gfsk = EasyLink_PHY_50KBPS2GFSK,  //!< Phy settings for Sub1G 50kbps data rate, IEEE 802.15.4g GFSK.
    EasyLink_Phy_625bpsLrm = EasyLink_PHY_625BPSLRM,      //!< Phy settings for Sub1G 625bps data rate, Long Range Mode.
    EasyLink_Phy_2_4_200kbps2gfsk = EasyLink_PHY_2_4_200KBPS2GFSK,   //!< Phy settings for 2.4Ghz 200kbps data rate, IEEE 802.15.4g GFSK.
    EasyLink_Phy_5kbpsSlLr = EasyLink_PHY_5KBPSSLLR,      //!< SimpleLink Long Range (5 kbps)
    EasyLink_Phy_2_4_100kbps2gfsk = EasyLink_PHY_2_4_100KBPS2GFSK,   //!< Phy settings for 2.4Ghz 100kbps data rate, IEEE 802.15.4g GFSK.
    EasyLink_Phy_2_4_250kbps2gfsk = EasyLink_PHY_2_4_250KBPS2GFSK,   //!< Phy settings for 2.4Ghz 250kbps data rate, IEEE 802.15.4g GFSK.
    EasyLink_Phy_200kbps2gfsk = EasyLink_PHY_200KBPS2GFSK,   //!< Phy settings for 200kbps data rate, IEEE 802.15.4g GFSK.
    EasyLink_Phy_100kbps2gfsk_ogntp = EasyLink_PHY_100KBPS2GFSK_OGN,   //!< Phy settings for 100kbps data rate
    EasyLink_Phy_100kbps2gfsk_legacy = EasyLink_PHY_100KBPS2GFSK_LEG,   //!< Phy settings for 100kbps data rate
    EasyLink_Phy_38400bps2gfsk_p3i = EasyLink_PHY_38400BPS2GFSK_P3I,   //!< Phy settings for 38400 bps data rate
    EasyLink_Num_Phy_Settings,
} EasyLink_PhyType;

//! \brief Advance configuration options
typedef enum
{
    EasyLink_Ctrl_AddSize = 0,           //!< Set the number of bytes in Addr for both Addr
                                         //!< Filter and Tx/Rx operations

    EasyLink_Ctrl_Idle_TimeOut = 1,      //!< Set a timeout value for inactivity on the radio,
                                         //!< i.e. if the radio stays idle for this amount of
                                         //!< time it is automatically powered down

    EasyLink_Ctrl_MultiClient_Mode = 2,  //!< Set Multiclient mode for application
                                         //!< that will use multiple RF clients.
                                         //!< Must be set before calling
                                         //!< EasyLink_init().

    EasyLink_Ctrl_AsyncRx_TimeOut = 3,   //!< Relative time in ticks from Async
                                         //!< Rx start to TimeOut. A value of
                                         //!< 0 means no timeout
    EasyLink_Ctrl_Cmd_Priority = 4,      //!< Set the command priority with a value
                                         //!< from EasyLink_Priority
    EasyLink_Ctrl_Test_Tone = 5,         //!< Enable/Disable Test mode for Tone
    EasyLink_Ctrl_Test_Signal = 6,       //!< Enable/Disable Test mode for Signal
    EasyLink_Ctrl_Rx_Test_Tone = 7,      //!< Enable/Disable Rx Test mode for Tone
} EasyLink_CtrlOption;

//! \brief Activity table
//!
//! +--------------+--------------------------------------+
//! |  Activity    |           Priority                   |
//! +--------------+------------+------------+------------+
//! |              |  Normal    |    High    |   Urgent   |
//! | TX           | 0X03090000 | 0X03090001 | 0X03090002 |
//! | RX           | 0X03070000 | 0X03070001 | 0X03070002 |
//! +--------------+--------------------------------------+
//!
typedef enum{
    EasyLink_Activity_Tx = 0x309,        //!< Activity code for the Tx operation
    EasyLink_Activity_Rx = 0x307,        //!< Activity code for the Rx operation
}EasyLink_Activity;

//! \brief Transmit and Receive Command Priority - These are only applicable in
//! a multi-client use-case
typedef enum{
    EasyLink_Priority_Normal = 0x0,
    EasyLink_Priority_High   = 0x1,
    EasyLink_Priority_Urgent = 0x2,
    EasyLink_Priority_NEntries
}EasyLink_Priority;

//! \brief EasyLink 32-bit Random number generator function type used in the
//! clear channel assessment algorithm.
typedef uint32_t (*EasyLink_GetRandomNumber)(void);

//! \brief Structure for EasyLink_init() and EasyLink_Params_init()
typedef struct {
    EasyLink_PhyType ui32ModType;        //!< PHY type
    RF_ClientCallback   pClientEventCb;  //!< Client event callback function
    RF_ClientEventMask  nClientEventMask;//!< Client event mask
    EasyLink_GetRandomNumber pGrnFxn;    //!< Pointer to function that returns a 32-bit unsigned random number
} EasyLink_Params;

//! \brief Structure for EasyLink_init() containing pointers to all the commands
//! necessary to define an RF setting
typedef struct
{
    EasyLink_PhyType EasyLink_phyType; //!< The PHY type that this RF setting
                                       //!< defines
    RF_Mode *RF_pProp;                 //!< Pointer to RF Mode Command

    union{
#if (defined CONFIG_CC1352P1_LAUNCHXL)  || (defined CONFIG_CC1352P_2_LAUNCHXL)  || \
    (defined CONFIG_CC1352P_4_LAUNCHXL) || (defined CONFIG_CC1352P1F3RGZ)
        rfc_CMD_PROP_RADIO_DIV_SETUP_PA_t *RF_pCmdPropRadioDivSetup;
#else
        rfc_CMD_PROP_RADIO_DIV_SETUP_t *RF_pCmdPropRadioDivSetup;
#endif
        rfc_CMD_PROP_RADIO_SETUP_t *RF_pCmdPropRadioSetup;
    }RF_uCmdPropRadio;              //!< Union containing either a pointer
                                    //!< to a divided radio setup command or
                                    //!< a pointer to a radio setup command

    rfc_CMD_FS_t *RF_pCmdFs;        //!< Pointer to a frequency setup command
    rfc_CMD_PROP_TX_t *RF_pCmdPropTx;        //!< Pointer to a RF TX command
    rfc_CMD_PROP_TX_ADV_t *RF_pCmdPropTxAdv; //!< Pointer to an advanced RF TX
                                             //!< command
    rfc_CMD_PROP_RX_ADV_t *RF_pCmdPropRxAdv; //!< Pointer to an advanced RF RX
                                             //!< command
    RF_TxPowerTable_Entry *RF_pTxPowerTable; //!< Pointer to a Tx power table
    uint8_t RF_txPowerTableSize;             //!< Tx power table size

}EasyLink_RfSetting;

//! \brief Structure for the TX Packet
typedef struct
{
        uint8_t dstAddr[8];              //!<  Destination address
        uint32_t absTime;                //!< Absolute time to Tx packet (0 for immediate)
                                         //!< Layer will use last SeqNum used + 1
        uint8_t len;                     //!< Payload Length
        uint8_t payload[EASYLINK_MAX_DATA_LENGTH];       //!< Payload
} EasyLink_TxPacket;

//! \brief Structure for the RX'ed Packet
typedef struct
{
        uint8_t dstAddr[8];              //!< Dst Address of RX'ed packet
        int8_t rssi;                     //!< rssi of RX'ed packet
        uint32_t absTime;                //!< Absolute time to turn on Rx when passed
                                         //!< (0 for immediate), Or Absolute time that packet was Rx
                                         //!< when returned.
        uint32_t rxTimeout;              //!< Relative time in ticks from Rx start to Rx TimeOut
                                         //!< a value of 0 means no timeout
        uint8_t len;                     //!< length of RX'ed packet
        uint8_t payload[EASYLINK_MAX_DATA_LENGTH]; //!< payload of RX'ed packet
} EasyLink_RxPacket;

//! \brief EasyLink Callback function type for Received packet, registered with EasyLink_ReceiveAsync()
typedef void (*EasyLink_ReceiveCb)(EasyLink_RxPacket * rxPacket,
        EasyLink_Status status);

//! \brief EasyLink Callback function type for Tx Done registered with EasyLink_TransmitAsync()
typedef void (*EasyLink_TxDoneCb)(EasyLink_Status status);

//! \brief Array containing all the supported EasyLink_rfSettings
extern EasyLink_RfSetting EasyLink_supportedPhys[];

//! \brief Size of the EasyLink_supportedPhys array
extern const uint8_t EasyLink_numSupportedPhys;

//*****************************************************************************
//
//! \brief Initializes the radio with specified Phy settings and RF client events
//!
//! This function configures the radio phy settings. If the ui32ModType
//! is ::EasyLink_Phy_Custom then the configuration is taken from srf_settings.h.
//! If a specific phy configuration is required (and not supported by any of
//! the defined Phy types in ::EasyLink_PhyType then you can cut and past the
//! RF setting from the SmartRF Studio code export tool. This will copy and use
//! the RF_prop, RF_cmdPropRadioDivSetup and RF_cmdFs commands, as well as the
//! Sync word from the RF_cmdPropTx and RF_cmdPropRx commands. The function
//! configures the callback for client events. The client events that trigger
//! the callback are set using the nClientEventMask type in ::EasyLink_Params.
//! A pointer to the desired callback should be provided to the pClientEventCb
//! type of the ::EasyLink_Params structure.
//!
//! \param params The descriptor for the Phy mode and event callback settings
//!
//! Default values are:
//!     ui32ModType            = EasyLink_Phy_50kbps2gfsk
//!     RF_ClientCallback      = NULL
//!     RF_ClientEventMask     = 0
//!     pGrnFxn                = (EasyLink_GetRandomNumber)rand
//!
//! \return EasyLink_Status
//
//*****************************************************************************
extern void EasyLink_Params_init(EasyLink_Params *params);

//*****************************************************************************
//
//! \brief Initializes the radio with specified Phy settings, CCA random number
//! generator, and RF client events
//!
//! This function configures the radio phy settings. If the ui32ModType
//! is ::EasyLink_Phy_Custom then the configuration is taken from srf_settings.h.
//! If a specific phy configuration is required (and not supported by any of
//! the defined Phy types in ::EasyLink_PhyType then you can cut and past the
//! RF setting from the SmartRF Studio code export tool. This will copy and use
//! the RF_prop, RF_cmdPropRadioDivSetup and RF_cmdFs commands, as well as the
//! Sync word from the RF_cmdPropTx and RF_cmdPropRx commands. The function
//! configures the callback for client events. The client events that trigger
//! the callback are set using the nClientEventMask type in ::EasyLink_Params.
//! A pointer to the desired callback should be provided to the pClientEventCb
//! type of the ::EasyLink_Params structure.
//!
//! \param params The descriptor for the Phy mode, random number generator, and
//!  event callback settings
//!
//! \return EasyLink_Status
//
//*****************************************************************************

extern EasyLink_Status EasyLink_init(EasyLink_Params *params);


//*****************************************************************************
//
//! \brief Gets the absolute radio time
//!
//! This function returns the absolute radio time and can be used for
//! monitoring or Tx/Rx events using the EasyLink_TxPacket and
//! EasyLink_RxPacket::absTime field.
//!
//! \param pui32AbsTime Pointer to return the signed RSSI value (dBm)
//!
//! \return ::EasyLink_Status
//
//*****************************************************************************
extern EasyLink_Status EasyLink_getAbsTime(uint32_t *pui32AbsTime);

//*****************************************************************************
//
//! \brief Gets the RSSI value of an ongoing Radio Operation
//!
//! This function returns the RSSI from an ongoing receiver radio operation.
//! It is useful in receiver test modes to detect the presence of both
//! modulated and unmodulated carrier waves
//!
//! \param pi8Rssi     Pointer to return the signed RSSI value (dBm)
//!
//! \return ::EasyLink_Status
//!
//! \note if no RSSI is available the function writes -128 to the value
//! argument
//
//*****************************************************************************
extern EasyLink_Status EasyLink_getRssi(int8_t *pi8Rssi);

//*****************************************************************************
//
//! \brief Sends a Packet with blocking call.
//!
//! This function is a blocking call to send a packet. If the Tx is
//! successfully scheduled then the function will block until the Tx is
//! complete.
//!
//! \param txPacket The descriptor for the packet to be Tx'ed.
//!
//! \return ::EasyLink_Status
//
//*****************************************************************************
extern EasyLink_Status EasyLink_transmit(EasyLink_TxPacket *txPacket);

//*****************************************************************************
//
//! \brief Sends a Packet with non blocking call.
//!
//! This function is a non blocking call to send a packet. If the Tx is
//! successfully scheduled then the callback will be call once the Tx is
//! complete.
//!
//! \param txPacket The descriptor for the packet to be Tx'ed.
//! \param cb       The tx done function pointer.
//!
//! \return ::EasyLink_Status
//
//*****************************************************************************
extern EasyLink_Status EasyLink_transmitAsync(EasyLink_TxPacket *txPacket,
        EasyLink_TxDoneCb cb);

//*****************************************************************************
//
//! \brief Sends a Packet with non blocking call if the channel is idle.
//!
//! This function is a non blocking call to send a packet. It will check for a
//! clear channel prior to transmission. If the channel is busy it will backoff
//! for a random period, in time units of EASYLINK_CCA_BACKOFF_TIMEUNITS, before
//! reassessing. It does this a certain number
//! (EASYLINK_MAX_CCA_BACKOFF_WINDOW - EASYLINK_MIN_CCA_BACKOFF_WINDOW)
//! of times before quitting unsuccessfully and running to the callback.
//! If the Tx is successfully scheduled then the callback will be called once
//! the Tx is complete.
//!
//! \param txPacket The descriptor for the packet to be Tx'ed.
//! \param cb       The tx done function pointer.
//!
//! \return ::EasyLink_Status
//
//*****************************************************************************
extern EasyLink_Status EasyLink_transmitCcaAsync(EasyLink_TxPacket *txPacket,
        EasyLink_TxDoneCb cb);

//*****************************************************************************
//
//! \brief Blocking call that waits for an Rx Packet.
//!
//! This function is a blocking call to wait for an Rx packet.
//!
//! \param rxPacket The descriptor for the packet to be Rx'ed.
//!
//! \return ::EasyLink_Status
//
//*****************************************************************************
extern EasyLink_Status EasyLink_receive(EasyLink_RxPacket *rxPacket);

//*****************************************************************************
//
//! \brief Enables Asynchronous Packet Rx with non blocking call.
//!
//! This function is a non blocking call to Rx a packet. The Rx is turned on
//! and the Callback is called once a packet is received. The Rx will timeout
//! if ::EasyLink_Ctrl_AsyncRx_TimeOut ctrl message is used to set the timeout
//! to something other than 0.
//!
//! \param cb        The rx function pointer.
//! \param absTime   Start time of Rx (0: now !0: absolute radio time to
//!                  start Rx)
//!
//! \return ::EasyLink_Status
//
//*****************************************************************************
extern EasyLink_Status EasyLink_receiveAsync(EasyLink_ReceiveCb cb, uint32_t absTime);

//*****************************************************************************
//
//! \brief Abort a previously call Async Tx/Rx.
//!
//! This function is a blocking call to abort a previous Async Tx/Rx
//!
//! \return ::EasyLink_Status
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
//! \param ui32Frequency Frequency in units of kHz
//!
//! \return ::EasyLink_Status
//
//*****************************************************************************
extern EasyLink_Status EasyLink_setFrequency(uint32_t ui32Frequency);

//*****************************************************************************
//
//! \brief Gets the Frequency
//!
//! This function gets Frequency in units of kHz. This function will return the
//! value set in the Frequency Synthesizer, and may not be the same as set
//! using EasyLink_setFrequency(). Note that this value does not include any
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
//! \return ::EasyLink_Status
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
//! \return ::EasyLink_Status
//
//*****************************************************************************
extern EasyLink_Status EasyLink_getIeeeAddr(uint8_t *ieeeAddr);

//*****************************************************************************
//
//! \brief Sets the TX Power
//!
//! This function sets the Tx Power
//!
//! \param i8TxPowerDbm  The Tx power in dBm's to be set.
//!        -# All platforms other than the CC1352P: Value of -10 dBm or values
//!        in the range of 0-14 dBm are accepted. Values above 14 are set to
//!        14 dBm while those below 0 are set to -10 dBm
//!        -# CC1352P Default PA: -20 to 14 dBm. Values above 14 dBm will be
//!        set to 14 dBm, while values below -20 dBm will cause a configuration
//!        error
//!        -# CC1352P High PA: 14 to 20 dBm. Values above 20 dBm will be
//!        set to 20 dBm, while values below 14 dBm will cause a configuration
//!        error
//!
//! \note  The PA mode is chosen at build time, run-time switching from high
//!        PA to default PA (or vice versa) is not supported.
//!
//! \return ::EasyLink_Status
//
//*****************************************************************************
extern EasyLink_Status EasyLink_setRfPower(int8_t i8TxPowerDbm);

//*****************************************************************************
//
//! \brief Gets the TX Power
//!
//! This function gets the Tx Power in dBm, values ranging from -10 to 14 dBm
//! should be expect
//!
//! \param pi8TxPowerDbm  Pointer to return the transmission power value (dBm)
//!
//! \return ::EasyLink_Status
//
//*****************************************************************************
extern EasyLink_Status EasyLink_getRfPower(int8_t *pi8TxPowerDbm);

//*****************************************************************************
//
//! \brief Sets advanced configuration options
//!
//! This function allows setting some of the advanced configuration options
//!
//! \param Ctrl        The control option to be set
//! \param ui32Value   The value to set the control option to
//!
//! \return ::EasyLink_Status
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
//! \param Ctrl         The control option to get
//! \param pui32Value   Pointer to return the control option value
//!
//! \return ::EasyLink_Status
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
#endif //DeviceFamily_CC13X2
