/*
 * bm78.h
 *
 *  Created on: 26 Dec 2016
 *      Author: sid
 */

#ifndef COM_BM78_H_
#define COM_BM78_H_

#include "../fanet.h"

//todo change
#define BM78_NRESET					6
#define BM78_NEEPROM					7

/* Debug */
#define BM78_debug_mode					0

/* Commands */
/* Common 1 */
#define	BM78_Read_Local_Information			0x01
#define	BM78_Reset 					0x02
#define	BM78_Read_BM77_Status				0x03
#define	BM78_Into_Power_Down_Mode			0x05
#define	BM78_Read_Device_Name				0x07
#define	BM78_Write_Device_Name 				0x08
#define	BM78_Erase_All_Paired_Device_Information	0x09
#define	BM78_Read_Pairing_Mode_Setting			0x0A
#define	BM78_Write_Pairing_Mode_Setting			0x0B
#define	BM78_Read_All_Paired_Device_Information		0x0C
#define	BM78_Delete_Paired_Device 			0x0D
/* GAP Comamnds */
#define	BM78_Read_RSSI_Value				0x10
#define	BM78_Write_Adv_Data				0x11
#define	BM78_Write_Scan_Res_Data			0x12
#define	BM78_Set Advertising Parameter			0x13
#define	BM78_Disconnect					0x1B
#define	BM78_Invisible_Setting				0x1C
#define	BM78_SPP_Create_Link				0x1D
#define	BM78_SPP_Create_Link_Cancel			0x1E
#define	BM78_Read_Remote_Device_Name			0x1F
/* SPP/GATT Transparent */
#define	BM78_Send_Transparent_Data			0x3a
/* Pairing commands */
#define	BM78_Passkey_Entry_Res				0x40
#define	BM78_User_Confirm_Res				0x41
/* Common 2*/
#define	BM78_Read_PIN_Code				0x50
#define	BM78_Write_PIN_Code				0x51
#define	BM78_Leave_Configure_Mode			0x52

/* Events */
/* Pairing */
#define	BM78_Passkey_Entry_Req 				0x60
#define	BM78_Pairing_Complete				0x61
#define	BM78_Passkey_DisplayYesNo_Req			0x62
/* GAP Event */
#define	BM78_LE_Connection_Complete 			0x71
#define	BM78_Disconnection_Complete 			0x72
#define	BM78_SPP_Connection_Complete			0x74
/* Common Event */
#define	BM78_Command_Complete				0x80
#define	BM78_BM77_Status_Report				0x81
#define	BM78_Configure_Mode_Status 			0x8f
/* SPP/GATT Transparent */
#define	BM78_Recieved _Transparent_Data			0x9a

#define BM78_SYNC					0xAA

/* Status Error Code */
#define BM78_Command_succeeded				0x00
//... todo

class BM78
{
private:
	Stream *mySerial = NULL;

	/* internal helper */
	uint8_t crc(uint8_t *data, unsigned int length);
	int rx(uint8_t *data, unsigned int length, unsigned int *timeout_ms);

	/* communication */
	boolean transmitt(uint8_t *data, unsigned int length);
	unsigned int receive(uint8_t *data, unsigned int buf_length, unsigned int timeout_ms);

	/* initial config*/
	boolean write_eeprom();

	/* config */
	boolean config_mode(boolean enter);
	int read_device_name(char *name, int length);
	boolean write_device_name(char *name, boolean permanent);
	boolean write_name_to_adv_data(char *name, boolean permanent);

public:
	BM78();
	boolean begin(Stream &port);

	boolean write_id(int id);
	boolean write_name(char *name);

	void end();
};

#ifdef FANET_BLUETOOTH
	extern BM78 bm78;
#endif

#endif /* COM_BM78_H_ */
