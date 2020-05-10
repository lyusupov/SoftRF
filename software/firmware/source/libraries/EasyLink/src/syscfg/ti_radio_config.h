#ifdef ENERGIA_ARCH_CC13X2
/*
 *  ======== ti_radio_config.h ========
 *  Configured RadioConfig module definitions
 *
 *  DO NOT EDIT - This file is generated for the CC1312R1F3RGZ
 *  by the SysConfig tool.
 *  
 *  Radio Config module version : 1.4
 *  SmartRF Studio data version : 2.16.0
 */
#ifndef _TI_RADIO_CONFIG_H_
#define _TI_RADIO_CONFIG_H_

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/rf_mailbox.h)
#include DeviceFamily_constructPath(driverlib/rf_common_cmd.h)
#include DeviceFamily_constructPath(driverlib/rf_prop_cmd.h)
#include <ti/drivers/rf/RF.h>


//*********************************************************************************
//  RF Setting:   50 kbps, 2-GFSK, 25 kHz deviation
//
//  PHY:          2gfsk50kbps     
//  Setting file: setting_tc106.json
//*********************************************************************************

// TX Power table size definition
#define TX_POWER_TABLE_SIZE 20

// TX Power Table Object
extern const RF_TxPowerTable_Entry txPowerTable[];

// TI-RTOS RF Mode Object
extern const RF_Mode RF_prop;

// RF Core API commands
extern const rfc_CMD_PROP_RADIO_DIV_SETUP_t RF_cmdPropRadioDivSetup;
extern const rfc_CMD_FS_t RF_cmdFs;
extern const rfc_CMD_PROP_TX_t RF_cmdPropTx;
extern const rfc_CMD_PROP_RX_ADV_t RF_cmdPropRxAdv;

// RF Core API Overrides
extern uint32_t pOverrides[];

#define OGNTP_SYNCWORD_SIZE   8
#define OGNTP_PAYLOAD_SIZE    20
#define OGNTP_CRC_SIZE        6

// RF Core API commands
extern const rfc_CMD_PROP_RADIO_DIV_SETUP_t RF_cmdPropRadioDivSetup_fsk_100kbps_ogntp;
extern const rfc_CMD_FS_t RF_cmdFs_fsk_100kbps_ogntp;
extern const rfc_CMD_PROP_TX_t RF_cmdPropTx_fsk_100kbps_ogntp;
extern const rfc_CMD_PROP_RX_ADV_t RF_cmdPropRxAdv_fsk_100kbps_ogntp;

#define LEGACY_SYNCWORD_SIZE  7
#define LEGACY_PAYLOAD_SIZE   24
#define LEGACY_CRC_SIZE       2

// RF Core API commands
extern const rfc_CMD_PROP_RADIO_DIV_SETUP_t RF_cmdPropRadioDivSetup_fsk_100kbps_legacy;
extern const rfc_CMD_FS_t RF_cmdFs_fsk_100kbps_legacy;
extern const rfc_CMD_PROP_TX_t RF_cmdPropTx_fsk_100kbps_legacy;
extern const rfc_CMD_PROP_RX_ADV_t RF_cmdPropRxAdv_fsk_100kbps_legacy;

#define P3I_SYNCWORD_SIZE     2
#define P3I_PAYLOAD_SIZE      24
#define P3I_PAYLOAD_OFFSET    6
#define P3I_CRC_SIZE          1

// RF Core API commands
extern const rfc_CMD_PROP_RADIO_DIV_SETUP_t RF_cmdPropRadioDivSetup_fsk_38400bps_p3i;
extern const rfc_CMD_FS_t RF_cmdFs_fsk_38400bps_p3i;
extern const rfc_CMD_PROP_TX_t RF_cmdPropTx_fsk_38400bps_p3i;
extern const rfc_CMD_PROP_RX_ADV_t RF_cmdPropRxAdv_fsk_38400bps_p3i;

//*********************************************************************************
//  RF Setting:   50 kbps, 2-GFSK, 25 kHz deviation
//
//  PHY:          2gfsk50kbps     
//  Setting file: setting_tc106.json
//*********************************************************************************

// TX Power table size definition
#define RF_PROP_TX_POWER_TABLE_SIZE_fsk_50kbps 20

// TX Power Table Object
extern RF_TxPowerTable_Entry RF_PROP_txPowerTable_fsk_50kbps[];

// TI-RTOS RF Mode Object
extern RF_Mode RF_prop_fsk_50kbps;

// RF Core API commands
extern rfc_CMD_PROP_RADIO_DIV_SETUP_t RF_cmdPropRadioDivSetup_fsk_50kbps;
extern rfc_CMD_FS_t RF_cmdFs_fsk_50kbps;
extern rfc_CMD_PROP_TX_t RF_cmdPropTx_fsk_50kbps;
extern rfc_CMD_PROP_RX_ADV_t RF_cmdPropRxAdv_fsk_50kbps;

// RF Core API Overrides
extern uint32_t pOverrides_fsk_50kbps[];

//*********************************************************************************
//  RF Setting:   5 kbps, SimpleLink Long Range (20 kchip/s, 2-GFSK, conv. FEC r=1/2 K=7, DSSS SF=2, Tx dev.: 5 kHz, Rx BW: 34 kHz)
//
//  PHY:          slr5kbps2gfsk     
//  Setting file: setting_tc480.json
//*********************************************************************************

// TX Power table size definition
#define RF_PROP_TX_POWER_TABLE_SIZE_SL_LR 20

// TX Power Table Object
extern RF_TxPowerTable_Entry PROP_RF_txPowerTable_sl_lr[];

// TI-RTOS RF Mode Object
extern RF_Mode RF_prop_sl_lr;

// RF Core API commands
extern rfc_CMD_PROP_RADIO_DIV_SETUP_t RF_cmdPropRadioDivSetup_sl_lr;
extern rfc_CMD_FS_t RF_cmdFs_sl_lr;
extern rfc_CMD_PROP_TX_t RF_cmdPropTx_sl_lr;
extern rfc_CMD_PROP_RX_ADV_t RF_cmdPropRxAdv_sl_lr;

// RF Core API Overrides
extern uint32_t pOverrides_sl_lr[];

//*********************************************************************************
//  RF Setting:   200 kbps, 2-GFSK, 50 kHz deviation, IEEE 802.15.4g FSK PHY mode, 915 MHz
//
//  PHY:          2gfsk200kbps154g     
//  Setting file: setting_tc146_154g.json
//*********************************************************************************

// TX Power table size definition
#define RF_PROP_TX_POWER_TABLE_SIZE_fsk_200kbps 20

// TX Power Table Object
extern RF_TxPowerTable_Entry RF_PROP_txPowerTable_fsk_200kbps[];

// TI-RTOS RF Mode Object
extern RF_Mode RF_prop_fsk_200kbps;

// RF Core API commands
extern rfc_CMD_PROP_RADIO_DIV_SETUP_t RF_cmdPropRadioDivSetup_fsk_200kbps;
extern rfc_CMD_FS_t RF_cmdFs_fsk_200kbps;
extern rfc_CMD_PROP_TX_ADV_t RF_cmdPropTxAdv_fsk_200kbps;
extern rfc_CMD_PROP_RX_ADV_t RF_cmdPropRxAdv_fsk_200kbps;

// RF Core API Overrides
extern uint32_t pOverrides_fsk_200kbps[];

#endif // _TI_RADIO_CONFIG_H_
#endif // ENERGIA_ARCH_CC13X2
