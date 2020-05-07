#ifdef ENERGIA_ARCH_CC13XX
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

/***** Includes *****/
#include "EasyLink.h"

/* Drivers */
#include <ti/drivers/rf/RF.h>
#include <driverlib/rf_data_entry.h>
#include <driverlib/rf_prop_mailbox.h>
#include <driverlib/rf_prop_cmd.h>
#include <driverlib/chipinfo.h>

#include "smartrf_settings/smartrf_settings_predefined.h"
#include "smartrf_settings/smartrf_settings.h"

#include <ti/sysbios/knl/Semaphore.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

#include <inc/hw_ccfg.h>
#include <inc/hw_ccfg_simple_struct.h>

#include "Board.h"

#include <uat.h>

union setupCmd_t{
    rfc_CMD_PROP_RADIO_DIV_SETUP_t divSetup;
    rfc_CMD_PROP_RADIO_SETUP_t setup;
};

#define EASYLINK_MAX_ADDR_SIZE           8
#define EASYLINK_MAX_ADDR_FILTERS        3

//Primary IEEE address location
#define EASYLINK_PRIMARY_IEEE_ADDR_LOCATION   0x500012F0
//Secondary IEEE address location
#define EASYLINK_SECONDARY_IEEE_ADDR_LOCATION 0x0001FFC8

#define EASYLINK_RF_EVENT_MASK  ( RF_EventLastCmdDone | RF_EventCmdError | \
             RF_EventCmdAborted | RF_EventCmdStopped | RF_EventCmdCancelled )

#define EASYLINK_RF_CMD_HANDLE_INVALID -1

#define RF_MODE_MULTIPLE 0x05

#define EasyLink_CmdHandle_isValid(handle) (handle >= 0)

/***** Prototypes *****/
static EasyLink_TxDoneCb txCb;
static EasyLink_ReceiveCb rxCb;

/***** Variable declarations *****/

static RF_Object rfObject;
static RF_Handle rfHandle;

//Rx buffer includes data entry structure, hdr (len=1byte), dst addr (max of 8 bytes) and data
//which must be aligned to 4B
#if defined(__TI_COMPILER_VERSION__)
    #pragma DATA_ALIGN (rxBuffer, 4);
        static uint8_t rxBuffer[sizeof(rfc_dataEntryGeneral_t) + 1 + EASYLINK_MAX_ADDR_SIZE + EASYLINK_MAX_DATA_LENGTH];
#elif defined(__IAR_SYSTEMS_ICC__)
    #pragma data_alignment = 4
        static uint8_t rxBuffer[sizeof(rfc_dataEntryGeneral_t) + 1 + EASYLINK_MAX_ADDR_SIZE + EASYLINK_MAX_DATA_LENGTH];
#elif defined(__GNUC__)
        static uint8_t rxBuffer[sizeof(rfc_dataEntryGeneral_t) + 1 + EASYLINK_MAX_ADDR_SIZE + EASYLINK_MAX_DATA_LENGTH];
#else
    #error This compiler is not supported.
#endif

static dataQueue_t dataQueue;
static rfc_propRxOutput_t rxStatistics;

//Tx buffer includes hdr (len=1byte), dst addr (max of 8 bytes) and data
static uint8_t txBuffer[1 + EASYLINK_MAX_ADDR_SIZE + EASYLINK_MAX_DATA_LENGTH];

//Addr size for Filter and Tx/Rx operations
//Set default to 1 byte addr to work with SmartRF
//studio default settings
static uint8_t addrSize = 1;

//Indicating that the API is initialized
static uint8_t configured = 0;
//Indicating that the API suspended
static uint8_t suspended = 0;

//RF Params alowing configuration of the inactivity timeout, which is the time
//it takes for the radio to shut down when there are no commands in the queue
static RF_Params rfParams;
static bool rfParamsConfigured = 0;

//Flag used to indicate the muli client operation is enabled
static bool rfModeMultiClient = false;

//Async Rx timeout value
static uint32_t asyncRxTimeOut = 0;

//local commands, contents will be defined by modulation type
static union setupCmd_t EasyLink_cmdPropRadioSetup;
static rfc_CMD_FS_t EasyLink_cmdFs;
static RF_Mode EasyLink_RF_prop;
static rfc_CMD_PROP_TX_t EasyLink_cmdPropTx;
static rfc_CMD_PROP_RX_ADV_t EasyLink_cmdPropRxAdv;

// The table for setting the Rx Address Filters
static uint8_t addrFilterTable[EASYLINK_MAX_ADDR_FILTERS * EASYLINK_MAX_ADDR_SIZE] = {0xaa};

//Mutex for locking the RF driver resource
static Semaphore_Handle busyMutex;

//Handle for last Async command, which is needed by EasyLink_abort
static RF_CmdHandle asyncCmdHndl = EASYLINK_RF_CMD_HANDLE_INVALID;

//Callback for Async Tx complete
static void txDoneCallback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e)
{
    EasyLink_Status status;

    //Release now so user callback can call EasyLink API's
    Semaphore_post(busyMutex);
    asyncCmdHndl = EASYLINK_RF_CMD_HANDLE_INVALID;

    if (e & RF_EventLastCmdDone)
    {
        status = EasyLink_Status_Success;
    }
    else if ( (e & RF_EventCmdAborted) || (e & RF_EventCmdCancelled ) )
    {
        status = EasyLink_Status_Aborted;
    }
    else
    {
        status = EasyLink_Status_Tx_Error;
    }

    if (txCb != NULL)
    {
        txCb(status);
    }
}

//Callback for Async Rx complete
static void rxDoneCallback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e)
{
    EasyLink_Status status = EasyLink_Status_Rx_Error;
    //create rxPacket as a static so that the large payload buffer it is not
    //allocated from the stack
    static EasyLink_RxPacket rxPacket;
    rfc_dataEntryGeneral_t *pDataEntry;
    pDataEntry = (rfc_dataEntryGeneral_t*) rxBuffer;

    //Release now so user callback can call EasyLink API's
    Semaphore_post(busyMutex);
    asyncCmdHndl = EASYLINK_RF_CMD_HANDLE_INVALID;

    if (e & RF_EventLastCmdDone)
    {
        //Check command status
        if (EasyLink_cmdPropRxAdv.status == PROP_DONE_OK)
        {
            //Check that data entry status indicates it is finished with
            if (pDataEntry->status != DATA_ENTRY_FINISHED)
            {
                status = EasyLink_Status_Rx_Error;
            }
            else if ( (rxStatistics.nRxOk == 1) ||
                     //or filer disabled and ignore due to addr mistmatch
                     ((EasyLink_cmdPropRxAdv.pktConf.filterOp == 1) &&
                      (rxStatistics.nRxIgnored == 1)) )
            {
#if 0
                //copy length from pDataEntry
                rxPacket.len = *(uint8_t*)(&pDataEntry->data) - addrSize;
                //copy address from packet payload (as it is not in hdr)
                memcpy(&rxPacket.dstAddr, (&pDataEntry->data + 1), addrSize);
                //copy payload
                memcpy(&rxPacket.payload, (&pDataEntry->data + 1 + addrSize), rxPacket.len);
#else
                rxPacket.len = LONG_FRAME_BYTES;
                //copy payload
                memcpy(&rxPacket.payload, (&pDataEntry->data), rxPacket.len);
#endif
                rxPacket.rssi = rxStatistics.lastRssi;
                rxPacket.absTime = rxStatistics.timeStamp;

                status = EasyLink_Status_Success;
            }
            else if ( rxStatistics.nRxBufFull == 1)
            {
                status = EasyLink_Status_Rx_Buffer_Error;
            }
            else if ( rxStatistics.nRxStopped == 1)
            {
                status = EasyLink_Status_Aborted;
            }
            else
            {
                status = EasyLink_Status_Rx_Error;
            }
        }
        else if ( EasyLink_cmdPropRxAdv.status == PROP_DONE_RXTIMEOUT)
        {
            status = EasyLink_Status_Rx_Timeout;
        }
        else
        {
            status = EasyLink_Status_Rx_Error;
        }
    }
    else if ( (e == RF_EventCmdAborted) || e == RF_EventCmdStopped )
    {
        status = EasyLink_Status_Aborted;
    }

    if (rxCb != NULL)
    {
        rxCb(&rxPacket, status);
    }
}

//Callback for Async TX Test mode
static void asyncCmdCallback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e)
{
    Semaphore_post(busyMutex);
    asyncCmdHndl = EASYLINK_RF_CMD_HANDLE_INVALID;
}

static EasyLink_Status enableTestMode(EasyLink_CtrlOption mode)
{
    EasyLink_Status status = EasyLink_Status_Cmd_Error;
    //This needs to be static as it is used by the RF driver and Modem after
    //this function exits
    static rfc_CMD_TX_TEST_t txTestCmd = {0};

    if ( (!configured) || suspended)
    {
        return EasyLink_Status_Config_Error;
    }
    if ( (mode != EasyLink_Ctrl_Test_Tone) &&
        (mode != EasyLink_Ctrl_Test_Signal) )
    {
        return EasyLink_Status_Param_Error;
    }
    //Check and take the busyMutex
    if ( (Semaphore_pend(busyMutex, 0) == FALSE) || (EasyLink_CmdHandle_isValid(asyncCmdHndl)) )
    {
        return EasyLink_Status_Busy_Error;
    }

    txTestCmd.commandNo = CMD_TX_TEST;
    txTestCmd.startTrigger.triggerType = TRIG_NOW;
    txTestCmd.startTrigger.pastTrig = 1;
    txTestCmd.startTime = 0;

    txTestCmd.config.bFsOff = 1;
    txTestCmd.syncWord = EasyLink_cmdPropTx.syncWord;
    txTestCmd.config.whitenMode = EasyLink_cmdPropRadioSetup.setup.formatConf.whitenMode;

    //set tone (unmodulated) or signal (modulated)
    if (mode == EasyLink_Ctrl_Test_Tone)
    {
        txTestCmd.config.bUseCw = 1;
    }
    else
    {
        txTestCmd.config.bUseCw = 0;
    }

    //generate continuous test signal
    txTestCmd.endTrigger.triggerType = TRIG_NEVER;

    /* Post command and store Cmd Handle for future abort */
    asyncCmdHndl = RF_postCmd(rfHandle, (RF_Op*)&txTestCmd, RF_PriorityNormal,
            asyncCmdCallback, EASYLINK_RF_EVENT_MASK);

    /* Has command completed? */
    uint16_t count = 0;
    while (txTestCmd.status != ACTIVE)
    {
        //The command did not complete as fast as expected, sleep for 10ms
        Task_sleep(10000 / Clock_tickPeriod);

        if (count++ > 500)
        {
            //Should not get here, if we did Something went wrong with the
            //the RF Driver, get out of here and return an error.
            //The next command will likely lock up.
            break;
        }
    }

    if (txTestCmd.status == ACTIVE)
    {
        status = EasyLink_Status_Success;
    }

    return status;
}

EasyLink_Status EasyLink_init(EasyLink_PhyType ui32ModType)
{
    if (configured)
    {
        //Already configure, check and take the busyMutex
        if (Semaphore_pend(busyMutex, 0) == FALSE)
        {
            return EasyLink_Status_Busy_Error;
        }
        RF_close(rfHandle);
    }

    if (!rfParamsConfigured)
    {
        RF_Params_init(&rfParams);
        //set default InactivityTimeout to 1000us
        rfParams.nInactivityTimeout = EasyLink_ms_To_RadioTime(1);
        rfParamsConfigured = 1;
    }

    if (ui32ModType == EasyLink_Phy_Custom)
    {
        if(ChipInfo_GetChipType() == CHIP_TYPE_CC2650)
        {
            memcpy(&EasyLink_cmdPropRadioSetup.setup, &RF_cmdPropRadioDivSetup, sizeof(rfc_CMD_PROP_RADIO_SETUP_t));
        }
        else
        {
            memcpy(&EasyLink_cmdPropRadioSetup.divSetup, &RF_cmdPropRadioDivSetup, sizeof(rfc_CMD_PROP_RADIO_DIV_SETUP_t));
        }
        memcpy(&EasyLink_cmdFs, &RF_cmdFs, sizeof(rfc_CMD_FS_t));
        memcpy(&EasyLink_RF_prop, &RF_prop, sizeof(RF_Mode));
        memcpy(&EasyLink_cmdPropRxAdv, &RF_cmdPropRxAdv, sizeof(rfc_CMD_PROP_RX_ADV_t));
        memcpy(&EasyLink_cmdPropTx, &RF_cmdPropTx, sizeof(rfc_CMD_PROP_TX_t));
    }
#if 0
    else if ( (ui32ModType == EasyLink_Phy_50kbps2gfsk) && (ChipInfo_GetChipType() != CHIP_TYPE_CC2650) )
    {
        memcpy(&EasyLink_cmdPropRadioSetup.divSetup,
                RF_pCmdPropRadioDivSetup_fsk,
                sizeof(rfc_CMD_PROP_RADIO_DIV_SETUP_t));
        memcpy(&EasyLink_cmdFs, RF_pCmdFs_preDef, sizeof(rfc_CMD_FS_t));
        memcpy(&EasyLink_RF_prop, RF_pProp_fsk, sizeof(RF_Mode));
        memcpy(&EasyLink_cmdPropRxAdv, RF_pCmdPropRxAdv_preDef, sizeof(rfc_CMD_PROP_RX_ADV_t));
        memcpy(&EasyLink_cmdPropTx, RF_pCmdPropTx_preDef, sizeof(rfc_CMD_PROP_TX_t));
    }
    else if ( (ui32ModType == EasyLink_Phy_625bpsLrm) && (ChipInfo_GetChipType() != CHIP_TYPE_CC2650) )
    {
        memcpy(&EasyLink_cmdPropRadioSetup.divSetup,
                RF_pCmdPropRadioDivSetup_lrm,
                sizeof(rfc_CMD_PROP_RADIO_DIV_SETUP_t));
        memcpy(&EasyLink_cmdFs, RF_pCmdFs_preDef, sizeof(rfc_CMD_FS_t));
        memcpy(&EasyLink_RF_prop, RF_pProp_lrm, sizeof(RF_Mode));
        memcpy(&EasyLink_cmdPropRxAdv, RF_pCmdPropRxAdv_preDef, sizeof(rfc_CMD_PROP_RX_ADV_t));
        memcpy(&EasyLink_cmdPropTx, RF_pCmdPropTx_preDef, sizeof(rfc_CMD_PROP_TX_t));
    }
    else if ( (ui32ModType == EasyLink_Phy_2_4_200kbps2gfsk) && (ChipInfo_GetChipType() == CHIP_TYPE_CC2650) )
    {
        memcpy(&EasyLink_cmdPropRadioSetup.setup,
                RF_pCmdPropRadioSetup_2_4G_fsk,
                sizeof(rfc_CMD_PROP_RADIO_SETUP_t));
        memcpy(&EasyLink_cmdFs, RF_pCmdFs_preDef, sizeof(rfc_CMD_FS_t));
        memcpy(&EasyLink_RF_prop, RF_pProp_2_4G_fsk, sizeof(RF_Mode));
        memcpy(&EasyLink_cmdPropRxAdv, RF_pCmdPropRxAdv_preDef, sizeof(rfc_CMD_PROP_RX_ADV_t));
        memcpy(&EasyLink_cmdPropTx, RF_pCmdPropTx_preDef, sizeof(rfc_CMD_PROP_TX_t));
    }
#endif
    else
    {
        if (busyMutex != NULL)
        {
            Semaphore_post(busyMutex);
        }
        return EasyLink_Status_Param_Error;
    }

    if (rfModeMultiClient)
    {
        EasyLink_RF_prop.rfMode = RF_MODE_MULTIPLE;
    }

    /* Request access to the radio */
    rfHandle = RF_open(&rfObject, &EasyLink_RF_prop,
            (RF_RadioSetup*)&EasyLink_cmdPropRadioSetup.setup, &rfParams);

    //Set Rx packet size, taking into account addr which is not in the hdr
    //(only length can be)

    EasyLink_cmdPropRxAdv.maxPktLen = LONG_FRAME_BYTES;
    EasyLink_cmdPropRxAdv.pAddr = addrFilterTable;
    addrSize = 1;
#if 0
    EasyLink_cmdPropRxAdv.addrConf.addrSize = addrSize; //Set addr size to the
                                                        //default
    EasyLink_cmdPropRxAdv.pktConf.filterOp = 1;  // Disable Addr filter by
                                                 //default
#endif
    EasyLink_cmdPropRxAdv.pQueue = &dataQueue;   // Set the Data Entity queue
                                                 // for received data
    EasyLink_cmdPropRxAdv.pOutput = (uint8_t*)&rxStatistics;

    //Set the frequency
    RF_runCmd(rfHandle, (RF_Op*)&EasyLink_cmdFs, RF_PriorityNormal, 0, //asyncCmdCallback,
            EASYLINK_RF_EVENT_MASK);

    //set default asyncRxTimeOut to 0
    asyncRxTimeOut = 0;

    //Create a semaphore for blocking commands
    Semaphore_Params params;
    Error_Block eb;

    // init params
    Semaphore_Params_init(&params);
    Error_init(&eb);

    // create semaphore instance if not already created
    if (busyMutex == NULL)
    {
        busyMutex = Semaphore_create(0, &params, &eb);
        if (busyMutex == NULL)
        {
            return EasyLink_Status_Mem_Error;
        }

        Semaphore_post(busyMutex);
    }
    else
    {
        //already configured and taken busyMutex, so release it
        Semaphore_post(busyMutex);
    }

    configured = 1;

    return EasyLink_Status_Success;
}

EasyLink_Status EasyLink_setFrequency(uint32_t ui32Freq)
{
    EasyLink_Status status = EasyLink_Status_Cmd_Error;
    //uint64_t ui64FractFreq;

    if ( (!configured) || suspended)
    {
        return EasyLink_Status_Config_Error;
    }
    //Check and take the busyMutex
    if (Semaphore_pend(busyMutex, 0) == FALSE)
    {
        return EasyLink_Status_Busy_Error;
    }

    /* Set the frequency */
    EasyLink_cmdFs.frequency = (uint16_t)(ui32Freq / 1000000);
    EasyLink_cmdFs.fractFreq = (uint16_t) (((uint64_t)ui32Freq -
            ((uint64_t)EasyLink_cmdFs.frequency * 1000000)) * 65536 / 1000000);

    /* Run command */
    RF_EventMask result = RF_runCmd(rfHandle, (RF_Op*)&EasyLink_cmdFs,
            RF_PriorityNormal, 0, EASYLINK_RF_EVENT_MASK);

    if (result & RF_EventLastCmdDone)
    {
        status = EasyLink_Status_Success;
    }

    Semaphore_post(busyMutex);

    return status;
}

uint32_t EasyLink_getFrequency(void)
{
    uint32_t freq_khz;

    if ( (!configured) || suspended)
    {
        return EasyLink_Status_Config_Error;
    }

    freq_khz = EasyLink_cmdFs.frequency * 1000000;
    freq_khz += ((((uint64_t)EasyLink_cmdFs.fractFreq * 1000000)) / 65536);

    return freq_khz;
}

EasyLink_Status EasyLink_setRfPwr(int8_t i8txPowerdBm)
{
    EasyLink_Status status = EasyLink_Status_Cmd_Error;
    rfc_CMD_SCH_IMM_t immOpCmd = {0};
    rfc_CMD_SET_TX_POWER_t cmdSetPower = {0};
    uint8_t txPowerIdx;

    if ( (!configured) || suspended)
    {
        return EasyLink_Status_Config_Error;
    }
    //Check and take the busyMutex
    if (Semaphore_pend(busyMutex, 0) == FALSE)
    {
        return EasyLink_Status_Busy_Error;
    }

    immOpCmd.commandNo = CMD_SCH_IMM;
    immOpCmd.startTrigger.triggerType = TRIG_NOW;
    immOpCmd.startTrigger.pastTrig = 1;
    immOpCmd.startTime = 0;

    cmdSetPower.commandNo = CMD_SET_TX_POWER;

    if (i8txPowerdBm < rfPowerTable[0].dbm)
    {
        i8txPowerdBm = rfPowerTable[0].dbm;
    }
    else if (i8txPowerdBm > rfPowerTable[rfPowerTableSize-1].dbm )
    {
        i8txPowerdBm = rfPowerTable[rfPowerTableSize-1].dbm;
    }

    //if max power is requested then the CCFG_FORCE_VDDR_HH must be set in
    //the ccfg
#if (CCFG_FORCE_VDDR_HH != 0x1)
    if (i8txPowerdBm == rfPowerTable[rfPowerTableSize-1].dbm)
    {
        //Release the busyMutex
        Semaphore_post(busyMutex);
        return EasyLink_Status_Config_Error;
    }
#endif

    for (txPowerIdx = 0;
            txPowerIdx < rfPowerTableSize;
            txPowerIdx++)
    {
        if (i8txPowerdBm >= rfPowerTable[txPowerIdx].dbm)
        {
            cmdSetPower.txPower = rfPowerTable[txPowerIdx].txPower;
            EasyLink_cmdPropRadioSetup.setup.txPower = rfPowerTable[txPowerIdx].txPower;
        }
    }

    //point the Operational Command to the immediate set power command
    immOpCmd.cmdrVal = (uint32_t) &cmdSetPower;

    // Send command
    RF_CmdHandle cmd = RF_postCmd(rfHandle, (RF_Op*)&immOpCmd,
            RF_PriorityNormal, 0, EASYLINK_RF_EVENT_MASK);

    RF_EventMask result = RF_pendCmd(rfHandle, cmd,  (RF_EventLastCmdDone |
            RF_EventCmdError));

    if (result & RF_EventLastCmdDone)
    {
        status = EasyLink_Status_Success;
    }

    //Release the busyMutex
    Semaphore_post(busyMutex);

    return status;
}

int8_t EasyLink_getRfPwr(void)
{
    uint8_t txPowerIdx;
    int8_t txPowerdBm = 0xff;

    if ( (!configured) || suspended)
    {
        return EasyLink_Status_Config_Error;
    }

    for (txPowerIdx = 0;
            txPowerIdx < rfPowerTableSize;
            txPowerIdx++)
    {
        if (rfPowerTable[txPowerIdx].txPower == EasyLink_cmdPropRadioSetup.setup.txPower)
        {
            txPowerdBm = rfPowerTable[txPowerIdx].dbm;
            continue;
        }
    }

    //if CCFG_FORCE_VDDR_HH is not set max power cannot be achieved
#if (CCFG_FORCE_VDDR_HH != 0x1)
    if (txPowerdBm == rfPowerTable[rfPowerTableSize-1].dbm)
    {
        txPowerdBm = rfPowerTable[rfPowerTableSize-2].dbm;
    }
#endif

    return txPowerdBm;
}

uint32_t EasyLink_getAbsTime(void)
{
    if ( (!configured) || suspended)
    {
        return EasyLink_Status_Config_Error;
    }

    return RF_getCurrentTime();
}

EasyLink_Status EasyLink_transmit(EasyLink_TxPacket *txPacket)
{
    EasyLink_Status status = EasyLink_Status_Tx_Error;

    if ( (!configured) || suspended)
    {
        return EasyLink_Status_Config_Error;
    }
    //Check and take the busyMutex
    if (Semaphore_pend(busyMutex, 0) == FALSE)
    {
        return EasyLink_Status_Busy_Error;
    }
    if (txPacket->len > EASYLINK_MAX_DATA_LENGTH)
    {
        return EasyLink_Status_Param_Error;
    }

    memcpy(txBuffer, txPacket->dstAddr, addrSize);
    memcpy(txBuffer + addrSize, txPacket->payload, txPacket->len);

    //packet length to Tx includes address
    EasyLink_cmdPropTx.pktLen = txPacket->len + addrSize;
    EasyLink_cmdPropTx.pPkt = txBuffer;

    if (txPacket->absTime != 0)
    {
        EasyLink_cmdPropTx.startTrigger.triggerType = TRIG_ABSTIME;
        EasyLink_cmdPropTx.startTrigger.pastTrig = 1;
        EasyLink_cmdPropTx.startTime = txPacket->absTime;
    }
    else
    {
        EasyLink_cmdPropTx.startTrigger.triggerType = TRIG_NOW;
        EasyLink_cmdPropTx.startTrigger.pastTrig = 1;
        EasyLink_cmdPropTx.startTime = 0;
    }

    // Send packet
    RF_CmdHandle cmdHdl = RF_postCmd(rfHandle, (RF_Op*)&EasyLink_cmdPropTx,
            RF_PriorityNormal, 0, EASYLINK_RF_EVENT_MASK);

    // Wait for Command to complete
    RF_EventMask result = RF_pendCmd(rfHandle, cmdHdl,  (RF_EventLastCmdDone |
            RF_EventCmdError));


    if (result & RF_EventLastCmdDone)
    {
        status = EasyLink_Status_Success;
    }

    //Release the busyMutex
    Semaphore_post(busyMutex);


    return status;
}

EasyLink_Status EasyLink_transmitAsync(EasyLink_TxPacket *txPacket, EasyLink_TxDoneCb cb)
{
    EasyLink_Status status = EasyLink_Status_Tx_Error;

    //Check if not configure or already an Async command being performed
    if ( (!configured) || suspended)
    {
        return EasyLink_Status_Config_Error;
    }
    //Check and take the busyMutex
    if ( (Semaphore_pend(busyMutex, 0) == FALSE) || (EasyLink_CmdHandle_isValid(asyncCmdHndl)) )
    {
        return EasyLink_Status_Busy_Error;
    }
    if (txPacket->len > EASYLINK_MAX_DATA_LENGTH)
    {
        return EasyLink_Status_Param_Error;
    }

    //store application callback
    txCb = cb;

    memcpy(txBuffer, txPacket->dstAddr, addrSize);
    memcpy(txBuffer + addrSize, txPacket->payload, txPacket->len);

    //packet length to Tx includes address
    EasyLink_cmdPropTx.pktLen = txPacket->len + addrSize;
    EasyLink_cmdPropTx.pPkt = txBuffer;

    if (txPacket->absTime != 0)
    {
        EasyLink_cmdPropTx.startTrigger.triggerType = TRIG_ABSTIME;
        EasyLink_cmdPropTx.startTrigger.pastTrig = 1;
        EasyLink_cmdPropTx.startTime = txPacket->absTime;
    }
    else
    {
        EasyLink_cmdPropTx.startTrigger.triggerType = TRIG_NOW;
        EasyLink_cmdPropTx.startTrigger.pastTrig = 1;
        EasyLink_cmdPropTx.startTime = 0;
    }

    /* Send packet */
    asyncCmdHndl = RF_postCmd(rfHandle, (RF_Op*)&EasyLink_cmdPropTx,
            RF_PriorityNormal, txDoneCallback, EASYLINK_RF_EVENT_MASK);

    if (EasyLink_CmdHandle_isValid(asyncCmdHndl))
    {
        status = EasyLink_Status_Success;
    }

    //busyMutex will be released by the callback

    return status;
}

EasyLink_Status EasyLink_receive(EasyLink_RxPacket *rxPacket)
{
    EasyLink_Status status = EasyLink_Status_Rx_Error;
    RF_EventMask result;
    rfc_dataEntryGeneral_t *pDataEntry;

    if ( (!configured) || suspended)
    {
        return EasyLink_Status_Config_Error;
    }
    //Check and take the busyMutex
    if (Semaphore_pend(busyMutex, 0) == FALSE)
    {
        return EasyLink_Status_Busy_Error;
    }

    pDataEntry = (rfc_dataEntryGeneral_t*) rxBuffer;
    //data entry rx buffer includes hdr (len-1Byte), addr (max 8Bytes) and data
    pDataEntry->length = 1 + EASYLINK_MAX_ADDR_SIZE + EASYLINK_MAX_DATA_LENGTH;
    pDataEntry->status = 0;
    dataQueue.pCurrEntry = (uint8_t*) pDataEntry;
    dataQueue.pLastEntry = NULL;
    EasyLink_cmdPropRxAdv.pQueue = &dataQueue;               /* Set the Data Entity queue for received data */
    EasyLink_cmdPropRxAdv.pOutput = (uint8_t*)&rxStatistics;

    if (rxPacket->absTime != 0)
    {
        EasyLink_cmdPropRxAdv.startTrigger.triggerType = TRIG_ABSTIME;
        EasyLink_cmdPropRxAdv.startTrigger.pastTrig = 1;
        EasyLink_cmdPropRxAdv.startTime = rxPacket->absTime;
    }
    else
    {
        EasyLink_cmdPropRxAdv.startTrigger.triggerType = TRIG_NOW;
        EasyLink_cmdPropRxAdv.startTrigger.pastTrig = 1;
        EasyLink_cmdPropRxAdv.startTime = 0;
    }

    if (rxPacket->rxTimeout != 0)
    {
        EasyLink_cmdPropRxAdv.endTrigger.triggerType = TRIG_ABSTIME;
        EasyLink_cmdPropRxAdv.endTime = RF_getCurrentTime() + rxPacket->rxTimeout;
    }
    else
    {
        EasyLink_cmdPropRxAdv.endTrigger.triggerType = TRIG_NEVER;
        EasyLink_cmdPropRxAdv.endTime = 0;
    }

    //Clear the Rx statistics structure
    memset(&rxStatistics, 0, sizeof(rfc_propRxOutput_t));

    RF_CmdHandle rx_cmd = RF_postCmd(rfHandle, (RF_Op*)&EasyLink_cmdPropRxAdv,
            RF_PriorityNormal, 0, EASYLINK_RF_EVENT_MASK);

    /* Wait for Command to complete */
    result = RF_pendCmd(rfHandle, rx_cmd, (RF_EventLastCmdDone | RF_EventCmdError));

    if (result & RF_EventLastCmdDone)
    {
        //Check command status
        if (EasyLink_cmdPropRxAdv.status == PROP_DONE_OK)
        {
            //Check that data entry status indicates it is finished with
            if (pDataEntry->status != DATA_ENTRY_FINISHED)
            {
                status = EasyLink_Status_Rx_Error;
            }
            //check Rx Statistics
            else if ( (rxStatistics.nRxOk == 1) ||
                     //or  filer disabled and ignore due to addr mistmatch
                     ((EasyLink_cmdPropRxAdv.pktConf.filterOp == 1) &&
                      (rxStatistics.nRxIgnored == 1)) )
            {
#if 0
                //copy length from pDataEntry (- addrSize)
                rxPacket->len = *(uint8_t*)(&pDataEntry->data) - addrSize;
                //copy address
                memcpy(rxPacket->dstAddr, (&pDataEntry->data + 1), addrSize);
                //copy payload
                memcpy(&rxPacket->payload, (&pDataEntry->data + 1 + addrSize), (rxPacket->len));
#else
                rxPacket->len = LONG_FRAME_BYTES;
                memcpy(&rxPacket->payload, (&pDataEntry->data), (rxPacket->len));
#endif
                rxPacket->rssi = rxStatistics.lastRssi;

                status = EasyLink_Status_Success;
                rxPacket->absTime = rxStatistics.timeStamp;
            }
            else if ( rxStatistics.nRxBufFull == 1)
            {
                status = EasyLink_Status_Rx_Buffer_Error;
            }
            else if ( rxStatistics.nRxStopped == 1)
            {
                status = EasyLink_Status_Aborted;
            }
            else
            {
                status = EasyLink_Status_Rx_Error;
            }
        }
        else if ( EasyLink_cmdPropRxAdv.status == PROP_DONE_RXTIMEOUT)
        {
            status = EasyLink_Status_Rx_Timeout;
        }
        else
        {
            status = EasyLink_Status_Rx_Error;
        }
    }

    //Release the busyMutex
    Semaphore_post(busyMutex);

    return status;
}

EasyLink_Status EasyLink_receiveAsync(EasyLink_ReceiveCb cb, uint32_t absTime)
{
    EasyLink_Status status = EasyLink_Status_Rx_Error;
    rfc_dataEntryGeneral_t *pDataEntry;

    //Check if not configure of already an Async command being performed
    if ( (!configured) || suspended)
    {
        return EasyLink_Status_Config_Error;
    }
    //Check and take the busyMutex
    if ( (Semaphore_pend(busyMutex, 0) == FALSE) || (EasyLink_CmdHandle_isValid(asyncCmdHndl)) )
    {
        return EasyLink_Status_Busy_Error;
    }

    rxCb = cb;

    pDataEntry = (rfc_dataEntryGeneral_t*) rxBuffer;
    //data entry rx buffer includes hdr (len-1Byte), addr (max 8Bytes) and data
    pDataEntry->length = 1 + EASYLINK_MAX_ADDR_SIZE + EASYLINK_MAX_DATA_LENGTH;
    pDataEntry->status = 0;
    dataQueue.pCurrEntry = (uint8_t*) pDataEntry;
    dataQueue.pLastEntry = NULL;
    EasyLink_cmdPropRxAdv.pQueue = &dataQueue;               /* Set the Data Entity queue for received data */
    EasyLink_cmdPropRxAdv.pOutput = (uint8_t*)&rxStatistics;

    if (absTime != 0)
    {
        EasyLink_cmdPropRxAdv.startTrigger.triggerType = TRIG_ABSTIME;
        EasyLink_cmdPropRxAdv.startTrigger.pastTrig = 1;
        EasyLink_cmdPropRxAdv.startTime = absTime;
    }
    else
    {
        EasyLink_cmdPropRxAdv.startTrigger.triggerType = TRIG_NOW;
        EasyLink_cmdPropRxAdv.startTrigger.pastTrig = 1;
        EasyLink_cmdPropRxAdv.startTime = 0;
    }

    if (asyncRxTimeOut != 0)
    {
        EasyLink_cmdPropRxAdv.endTrigger.triggerType = TRIG_ABSTIME;
        EasyLink_cmdPropRxAdv.endTime = RF_getCurrentTime() + asyncRxTimeOut;
    }
    else
    {
        EasyLink_cmdPropRxAdv.endTrigger.triggerType = TRIG_NEVER;
        EasyLink_cmdPropRxAdv.endTime = 0;
    }

    //Clear the Rx statistics structure
    memset(&rxStatistics, 0, sizeof(rfc_propRxOutput_t));

    asyncCmdHndl = RF_postCmd(rfHandle, (RF_Op*)&EasyLink_cmdPropRxAdv,
            RF_PriorityNormal, rxDoneCallback, EASYLINK_RF_EVENT_MASK);

    if (EasyLink_CmdHandle_isValid(asyncCmdHndl))
    {
        status = EasyLink_Status_Success;
    }

    //busyMutex will be released in callback

    return status;
}

EasyLink_Status EasyLink_abort(void)
{
    EasyLink_Status status = EasyLink_Status_Cmd_Error;

    if ( (!configured) || suspended)
    {
        return EasyLink_Status_Config_Error;
    }
    //check an Async command is running, if not return success
    if (!EasyLink_CmdHandle_isValid(asyncCmdHndl))
    {
        return EasyLink_Status_Aborted;
    }

    //force abort (gracefull param set to 0)
    if (RF_cancelCmd(rfHandle, asyncCmdHndl, 0) == RF_StatSuccess)
    {
       /* Wait for Command to complete */
       RF_EventMask result = RF_pendCmd(rfHandle, asyncCmdHndl, (RF_EventLastCmdDone | RF_EventCmdError |
               RF_EventCmdAborted | RF_EventCmdCancelled | RF_EventCmdStopped));

       if (result & RF_EventLastCmdDone)
       {
           status = EasyLink_Status_Success;
       }
    }
    else
    {
       status = EasyLink_Status_Cmd_Error;
    }

    return status;
}

EasyLink_Status EasyLink_enableRxAddrFilter(uint8_t* pui8AddrFilterTable, uint8_t ui8AddrSize, uint8_t ui8NumAddrs)
{
    EasyLink_Status status = EasyLink_Status_Param_Error;

    if ( (!configured) || suspended)
    {
        return EasyLink_Status_Config_Error;
    }
    if ( Semaphore_pend(busyMutex, 0) == FALSE )
    {
        return EasyLink_Status_Busy_Error;
    }

    if ( (pui8AddrFilterTable != NULL) &&
            (ui8AddrSize != 0) && (ui8NumAddrs != 0) &&
            (ui8AddrSize == addrSize) &&
            (ui8NumAddrs <= EASYLINK_MAX_ADDR_FILTERS) )
    {
        memcpy(addrFilterTable, pui8AddrFilterTable, EASYLINK_MAX_ADDR_SIZE * EASYLINK_MAX_ADDR_FILTERS);
        EasyLink_cmdPropRxAdv.addrConf.addrSize = ui8AddrSize;
        EasyLink_cmdPropRxAdv.addrConf.numAddr = ui8NumAddrs;
        EasyLink_cmdPropRxAdv.pktConf.filterOp = 0;

        status = EasyLink_Status_Success;
    }
    else if (pui8AddrFilterTable == NULL)
    {
        //disable filter
        EasyLink_cmdPropRxAdv.pktConf.filterOp = 1;

        status = EasyLink_Status_Success;
    }

    //Release the busyMutex
    Semaphore_post(busyMutex);

    return status;
}

EasyLink_Status EasyLink_setCtrl(EasyLink_CtrlOption Ctrl, uint32_t ui32Value)
{
    EasyLink_Status status = EasyLink_Status_Param_Error;
    switch(Ctrl)
    {
        case EasyLink_Ctrl_AddSize:
            if (ui32Value <= EASYLINK_MAX_ADDR_SIZE)
            {
                addrSize = (uint8_t) ui32Value;
                EasyLink_cmdPropRxAdv.addrConf.addrSize = addrSize;
                status = EasyLink_Status_Success;
            }
            break;
        case EasyLink_Ctrl_Idle_TimeOut:
            rfParams.nInactivityTimeout = ui32Value;
            rfParamsConfigured = 1;
            status = EasyLink_Status_Success;
            break;
        case EasyLink_Ctrl_MultiClient_Mode:
            rfModeMultiClient = (bool) ui32Value;
            status = EasyLink_Status_Success;
            break;
        case EasyLink_Ctrl_AsyncRx_TimeOut:
            asyncRxTimeOut = ui32Value;
            status = EasyLink_Status_Success;
            break;
        case EasyLink_Ctrl_Test_Tone:
            status = enableTestMode(EasyLink_Ctrl_Test_Tone);
            break;
        case EasyLink_Ctrl_Test_Signal:
            status = enableTestMode(EasyLink_Ctrl_Test_Signal);
            break;
    }

    return status;
}

EasyLink_Status EasyLink_getCtrl(EasyLink_CtrlOption Ctrl, uint32_t* pui32Value)
{
    EasyLink_Status status = EasyLink_Status_Cmd_Error;

    switch(Ctrl)
    {
        case EasyLink_Ctrl_AddSize:
            *pui32Value = addrSize;
            status = EasyLink_Status_Success;
            break;
        case EasyLink_Ctrl_Idle_TimeOut:
            *pui32Value = rfParams.nInactivityTimeout;
            status = EasyLink_Status_Success;
            break;
        case EasyLink_Ctrl_MultiClient_Mode:
            *pui32Value = (uint32_t) rfModeMultiClient;
            status = EasyLink_Status_Success;
            break;
        case EasyLink_Ctrl_AsyncRx_TimeOut:
            *pui32Value = asyncRxTimeOut;
            status = EasyLink_Status_Success;
            break;
        case EasyLink_Ctrl_Test_Tone:
        case EasyLink_Ctrl_Test_Signal:
            *pui32Value = 0;
            status = EasyLink_Status_Success;
            break;
    }

    return status;
}

EasyLink_Status EasyLink_getIeeeAddr(uint8_t *ieeeAddr)
{
    EasyLink_Status status = EasyLink_Status_Param_Error;

    if (ieeeAddr != NULL)
    {
        int i;

        //Reading from primary IEEE location...
        uint8_t *location = (uint8_t *)EASYLINK_PRIMARY_IEEE_ADDR_LOCATION;

        /*
         * ...unless we can find a byte != 0xFF in secondary
         *
         * Intentionally checking all 8 bytes here instead of len, because we
         * are checking validity of the entire IEEE address irrespective of the
         * actual number of bytes the caller wants to copy over.
         */
        for (i = 0; i < 8; i++) {
            if (((uint8_t *)EASYLINK_SECONDARY_IEEE_ADDR_LOCATION)[i] != 0xFF) {
                //A byte in the secondary location is not 0xFF. Use the
                //secondary
                location = (uint8_t *)EASYLINK_SECONDARY_IEEE_ADDR_LOCATION;
                break;
            }
        }

        //inverting byte order
       for (i = 0; i < 8; i++) {
           ieeeAddr[i] = location[8 - 1 - i];
       }


        status = EasyLink_Status_Success;
    }

    return status;
}
#endif //DEVICE_FAMILY == cc13x0
