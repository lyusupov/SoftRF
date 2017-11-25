#ifndef _fanet_H_
#define _fanet_H_

/* optional communication interfaces */
#define FANET_USB
//#define FANET_BLUETOOTH				//also uses SerialFANET but extends features

/* only select one or none */
//#define FANET_DEBUG_USB
#define FANET_DEBUG_SERIAL_BLUETOOTH

/* Automatically broadcast the name */
//currently the only name source is the bluetooth device name.
//#define FANET_NAME_AUTOBRDCAST

#ifdef FANET_BLUETOOTH
/* this define is used to remove the ability to upload eeprom data to the BM78 and disables FANET id setting */
//#define FANET_BLUETOOTH_ENDUSER_SAVE
#define FANET_BT_AUTOFF_MS			(5*60*1000)		//5min
#define FANET_BT_OFF_MS				2000
#endif

//note: doe not change 'firmware-' it is required for bin version identification. only use a float number afterwards 'x.yy'
#define FANET_VERSION		F("firmware-0.02")

/*
 * ***CUSTOMIZE***
 * Must be changed if its not a Skytraxx device
 * For a manufacturer ID contact me: juergen [at] skytraxx [dot] eu
 */
#define FANET_MANUFACTURER		FANET_MANU_SKYTRAXX

/*
 * Do not change anything from here on
 */
#define FANET_MANU_SKYTRAXX		1


/*
 * Pins
 */
/* required for bluetooth */
#define FANET_BTN			2
#define FANET_PWR			6
#define FANET_CHG			7

#endif /* _fanet_H_ */
