#ifdef ENERGIA_ARCH_CC13X2

/*
 *  ======== ti_easylink_config.c ========
 *  Configured EasyLink module definitions
 *
 *  DO NOT EDIT - This file is generated for the CC1312R1_LAUNCHXL
 *  by the SysConfig tool.
 */

/***** Includes *****/
#include "easylink/EasyLinkv2.h"
#include <stdint.h>

/* TI Drivers */
#include <ti/drivers/rf/RF.h>

/* RF Studio */
#include "ti_radio_config.h"

EasyLink_RfSetting EasyLink_supportedPhys[] = {
    {
        .EasyLink_phyType = EasyLink_Phy_Custom,
        .RF_pProp = &RF_prop,
        .RF_uCmdPropRadio.RF_pCmdPropRadioDivSetup = &RF_cmdPropRadioDivSetup,
        .RF_pCmdFs = &RF_cmdFs,
        .RF_pCmdPropTx = &RF_cmdPropTx,
        .RF_pCmdPropTxAdv = NULL,
        .RF_pCmdPropRxAdv = &RF_cmdPropRxAdv,
        .RF_pTxPowerTable = txPowerTable,
        .RF_txPowerTableSize = TX_POWER_TABLE_SIZE,
    },
#if 0
    {
        .EasyLink_phyType = EasyLink_Phy_50kbps2gfsk,
        .RF_pProp = &RF_prop,
        .RF_uCmdPropRadio.RF_pCmdPropRadioDivSetup = &RF_cmdPropRadioDivSetup_fsk_50kbps,
        .RF_pCmdFs = &RF_cmdFs,
        .RF_pCmdPropTx = &RF_cmdPropTx,
        .RF_pCmdPropTxAdv = NULL,
        .RF_pCmdPropRxAdv = &RF_cmdPropRxAdv,
        .RF_pTxPowerTable = txPowerTable,
        .RF_txPowerTableSize = TX_POWER_TABLE_SIZE,
    },
    {
        .EasyLink_phyType = EasyLink_Phy_5kbpsSlLr,
        .RF_pProp = &RF_prop,
        .RF_uCmdPropRadio.RF_pCmdPropRadioDivSetup = &RF_cmdPropRadioDivSetup_sl_lr,
        .RF_pCmdFs = &RF_cmdFs,
        .RF_pCmdPropTx = &RF_cmdPropTx_sl_lr,
        .RF_pCmdPropTxAdv = NULL,
        .RF_pCmdPropRxAdv = &RF_cmdPropRxAdv_sl_lr,
        .RF_pTxPowerTable = txPowerTable,
        .RF_txPowerTableSize = TX_POWER_TABLE_SIZE,
    },
    {
        .EasyLink_phyType = EasyLink_Phy_200kbps2gfsk,
        .RF_pProp = &RF_prop,
        .RF_uCmdPropRadio.RF_pCmdPropRadioDivSetup = &RF_cmdPropRadioDivSetup_fsk_200kbps,
        .RF_pCmdFs = &RF_cmdFs_fsk_200kbps,
        .RF_pCmdPropTx = NULL,
        .RF_pCmdPropTxAdv = &RF_cmdPropTxAdv_fsk_200kbps,
        .RF_pCmdPropRxAdv = &RF_cmdPropRxAdv_fsk_200kbps,
        .RF_pTxPowerTable = txPowerTable,
        .RF_txPowerTableSize = TX_POWER_TABLE_SIZE,
    },
#else
    {
        .EasyLink_phyType = EasyLink_Phy_100kbps2gfsk_ogntp,
        .RF_pProp = &RF_prop,
        .RF_uCmdPropRadio.RF_pCmdPropRadioDivSetup = &RF_cmdPropRadioDivSetup_fsk_100kbps_ogntp,
        .RF_pCmdFs = &RF_cmdFs_fsk_100kbps_ogntp,
        .RF_pCmdPropTx = &RF_cmdPropTx_fsk_100kbps_ogntp,
        .RF_pCmdPropTxAdv = NULL,
        .RF_pCmdPropRxAdv = &RF_cmdPropRxAdv_fsk_100kbps_ogntp,
        .RF_pTxPowerTable = txPowerTable,
        .RF_txPowerTableSize = TX_POWER_TABLE_SIZE,
    },
    {
        .EasyLink_phyType = EasyLink_Phy_100kbps2gfsk_legacy,
        .RF_pProp = &RF_prop,
        .RF_uCmdPropRadio.RF_pCmdPropRadioDivSetup = &RF_cmdPropRadioDivSetup_fsk_100kbps_legacy,
        .RF_pCmdFs = &RF_cmdFs_fsk_100kbps_legacy,
        .RF_pCmdPropTx = &RF_cmdPropTx_fsk_100kbps_legacy,
        .RF_pCmdPropTxAdv = NULL,
        .RF_pCmdPropRxAdv = &RF_cmdPropRxAdv_fsk_100kbps_legacy,
        .RF_pTxPowerTable = txPowerTable,
        .RF_txPowerTableSize = TX_POWER_TABLE_SIZE,
    },
    {
        .EasyLink_phyType = EasyLink_Phy_38400bps2gfsk_p3i,
        .RF_pProp = &RF_prop,
        .RF_uCmdPropRadio.RF_pCmdPropRadioDivSetup = &RF_cmdPropRadioDivSetup_fsk_38400bps_p3i,
        .RF_pCmdFs = &RF_cmdFs_fsk_38400bps_p3i,
        .RF_pCmdPropTx = &RF_cmdPropTx_fsk_38400bps_p3i,
        .RF_pCmdPropTxAdv = NULL,
        .RF_pCmdPropRxAdv = &RF_cmdPropRxAdv_fsk_38400bps_p3i,
        .RF_pTxPowerTable = txPowerTable,
        .RF_txPowerTableSize = TX_POWER_TABLE_SIZE,
    },
#endif
};

const uint8_t EasyLink_numSupportedPhys = sizeof(EasyLink_supportedPhys)/sizeof(EasyLink_RfSetting);
#endif // ENERGIA_ARCH_CC13X2
