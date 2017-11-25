/*
 * serial.h
 *
 *  Created on: 1 Oct 2016
 *      Author: sid
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#include "../fanet.h"
#include "../fanet_stack/fmac.h"

#define SerialFANET			Serial1
#ifdef FANET_USB
	#define SerialFANET_USB		Serial
#endif

#if defined(FANET_DEBUG_USB) && defined(FANET_USB)
	#define SerialDEBUG		Serial
#elif defined(FANET_DEBUG_SERIAL_BLUETOOTH)
	#define SerialDEBUG		Serial1
#endif

#define FANET_CMD_START			"#FN"
#define BT_CMD_START			"#BT"
#define DONGLE_CMD_START		"#DG"
#define SEPARATOR			','


/* Fanet Replies */
#define FANET_CMD_OK			F(FANET_CMD_START "R OK")
#define FANET_CMD_ERROR			F(FANET_CMD_START "R ERR")
#define FANET_CMD_MSG			F(FANET_CMD_START "R MSG")
#define FANET_CMD_ACK			F(FANET_CMD_START "R ACK")
#define FANET_CMD_NACK			F(FANET_CMD_START "R NACK")

/* Bluetooth Replies */
#define BT_CMD_OK			F(BT_CMD_START "R OK")
#define BT_CMD_WARN			F(BT_CMD_START "R WRN")
#define BT_CMD_ERROR			F(BT_CMD_START "R ERR")

/* Dongle Replies */
#define DONGLE_CMD_OK			F(DONGLE_CMD_START "R OK")
#define DONGLE_CMD_MSG			F(DONGLE_CMD_START "R MSG")
#define DONGLE_CMD_ERROR		F(DONGLE_CMD_START "R ERR")

/* Commands */
/* FANET */
#define CMD_STATE			'S'
#define CMD_TRANSMIT			'T'
#define CMD_ADDR			'A'
#define CMD_CONFIG			'C'

#define CMD_NAME			'N'		//bluetooth only

/* Dongle */
#define CMD_VERSION			'V'
#define CMD_POWER			'P'
#define CMD_REGION			'L'
#define CMD_BOOTLOADER			'J'

#define CMD_RX_FRAME			"F"

#define SERIAL_debug_mode		0

/* values <= 0 removes code and message */
#define FN_REPLY_OK			FANET_CMD_OK, 	 0,  NULL
#define FN_REPLYM_INITIALIZED		FANET_CMD_MSG,   1,  F("initialized")
#define FN_REPLYE_RADIO_FAILED		FANET_CMD_ERROR, 2,  F("radio failed")
#define FN_REPLYE_UNKNOWN_CMD		FANET_CMD_ERROR, 5,  F("unknown command")
#define FN_REPLYE_FN_UNKNOWN_CMD	FANET_CMD_ERROR, 6,  F("unknown FN command")
#define FN_REPLYE_NO_SRC_ADDR		FANET_CMD_ERROR, 10, F("no source address")
#define FN_REPLYE_INVALID_ADDR		FANET_CMD_ERROR, 11, F("invalid address")
#define FN_REPLYE_INCOMPATIBLE_TYPE	FANET_CMD_ERROR, 12, F("incompatible type")
#define FN_REPLYE_TX_BUFF_FULL		FANET_CMD_ERROR, 14, F("tx buffer full")
#define FN_REPLYE_CMD_TOO_SHORT		FANET_CMD_ERROR, 30, F("too short")
#define FN_REPLYE_BT_FAILED		BT_CMD_ERROR,    51, F("bt failed")
#define FN_REPLYE_BT_UNKNOWN_CMD	BT_CMD_ERROR,    52, F("unknown BT command")
#define FN_REPLYE_BT_NAME_TOO_SHORT	BT_CMD_ERROR,    53, F("name too short")
#define FN_REPLYW_BT_RECONF_ID		BT_CMD_WARN,     55, F("reconfiguring id")
#define DN_REPLY_OK			DONGLE_CMD_OK, 	 0,  NULL
#define DN_REPLYE_DONGLE_UNKNOWN_CMD	DONGLE_CMD_ERROR,60, F("unknown DG command")
#define DN_REPLYE_JUMP			DONGLE_CMD_ERROR,61, F("unknown jump point")
#define DN_REPLYE_POWER			DONGLE_CMD_ERROR,70, F("power switch failed")
#define DN_REPLYE_TOOLESSPARAMETER	DONGLE_CMD_ERROR,80, F("too less parameter")
#define DN_REPLYE_UNKNOWNPARAMETER	DONGLE_CMD_ERROR,81, F("unknown parameter")

/*
 * Normal Commands
 * State: 		#FNS lat(deg),lon(deg),alt(m),speed(km/h),climb(m/s),heading(deg)[,turn(deg/s)]		note: all values in float (NOT hex)
 * Address: 		#FNA manufacturer(hex),id(hex)
 * Config: 		#FNC type(0..7),onlinelogging(0..1)							note: type see protocol.txt
 * Transmit: 		#FNT type,dest_manufacturer,dest_id,forward,ack_required,length,length*2hex		note: all values in hex
 *
 * Receive a Frame:	#FNF src_manufacturer,src_id,broadcast,signature,type,payloadlength,payload
 *
 * Maintenance
 * Version:		#DGV
 * Standby:		#DGP powermode(0..1)									note: w/o status is returned
 * Region:		#DGL freq(868,915),power(2..20 (dBm))
//todo: txpower, channel
//todo
 * Jump to DFU:		#DGJ BLsw										note: not working probably for USB
 * 			#DGJ BLhw										note: requires external RC network
 * 														recommended: use 2 GPIOs connected
 //
 * 														to RESEST and BOOT0
 *
 * Bluetooth Commands
 * Name:		#BTN name
 * Address:		#BTA addr										note: cmd may be disabled
 */

class Serial_Interface
{
private:
	Stream *mySerial = NULL;
	uint32_t last_activity = 0;

	/* Normal Commands */
	void fanet_eval(String &str);
	void fanet_cmd_state(char *ch_str);
	void fanet_cmd_addr(char *ch_str);
	void fanet_cmd_config(char *ch_str);
	void fanet_cmd_transmit(char *ch_str);

	/* Dongle Commands */
	void dongle_eval(String &str);
	void dongle_cmd_version(char *ch_str);
	void dongle_cmd_power(char *ch_str);
	void dongle_cmd_region(char *ch_str);
	void dongle_cmd_jump(char *ch_str);

	/* Bluetooth Commands */
#ifdef FANET_BLUETOOTH
	void bt_eval(String &str);
	void bt_cmd_addr(char *ch_str);
	void bt_cmd_name(char *ch_str);
#endif

#ifdef FANET_NMEA_EXTENTION
	//note: that could be used to switch to a different output format
	boolean has_nmea = false;
#endif

public:
	Serial_Interface() {};
	void begin(Stream &port);

	/* redirected from app */
	void handle_frame(Frame *frm);
	void handle_acked(boolean ack, MacAddr &addr);

	void handle_rx(void);
	void ack(bool value, int manufacturer, int id);

	uint32_t get_lastactivity(void) { return last_activity; };
	boolean any_actitity(void) { return (last_activity!=0); };

	void println(const __FlashStringHelper *type, int key, const __FlashStringHelper *msg);

};

extern Serial_Interface serial_int;

#endif /* SERIAL_H_ */
