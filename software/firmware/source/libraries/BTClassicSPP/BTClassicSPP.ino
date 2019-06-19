//This example code is in the Public Domain (or CC0 licensed, at your option.)
//By Evandro Copercini - 2018
//
//This example creates a bridge between Serial and Classical Bluetooth (SPP)
//and also demonstrate that SerialBT have the same functionalities of a normal Serial

#include <MicroNMEA.h>
#include "BTSPP.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
esp_bd_addr_t BT_SPP_address = { 0x00, 0x0d , 0xb5 , 0x32 , 0x58 , 0x81 };
String SB_SPP_name = "BT-GPS-325881";
char nmeaBuffer[200];
MicroNMEA nmea (nmeaBuffer, sizeof (nmeaBuffer));

void show_gps_data () {
	//ledState = !ledState;
	//digitalWrite (LED_BUILTIN, ledState);

	// Output GPS information from previous second
	Serial.print ("Valid fix: ");
	Serial.print (nmea.isValid () ? "yes " : "no ");
	Serial.print (nmea.getFix () == 1 ? "No Fix" : nmea.getFix () == 2 ? "2D" : "3D");
	Serial.println (nmea.getAutofix () == 'A' ? " Auto" : " Manual");

	Serial.print ("Nav. system: ");
	if (nmea.getNavSystem ())
		Serial.println (nmea.getNavSystem ());
	else
		Serial.println ("none");

	Serial.print ("Num. satellites: ");
	Serial.println (nmea.getNumSatellites ());

	Serial.print ("Dispersion: P ");
	Serial.print (nmea.getPDOP () / 10., 1);
	Serial.print (" V ");
	Serial.print (nmea.getVDOP () / 10., 1);
	Serial.print (" H "); 
	Serial.println (nmea.getHDOP () / 10., 1);

	Serial.print ("Date/time: ");
	Serial.print (nmea.getYear ());
	Serial.print ('-');
	Serial.print (int (nmea.getMonth ()));
	Serial.print ('-');
	Serial.print (int (nmea.getDay ()));
	Serial.print ('T');
	Serial.print (int (nmea.getHour ()));
	Serial.print (':');
	Serial.print (int (nmea.getMinute ()));
	Serial.print (':');
	Serial.println (int (nmea.getSecond ()));

	long latitude_mdeg = nmea.getLatitude ();
	long longitude_mdeg = nmea.getLongitude ();
	Serial.print ("Latitude (deg): ");
	Serial.println (latitude_mdeg / 1000000., 6);

	Serial.print ("Longitude (deg): ");
	Serial.println (longitude_mdeg / 1000000., 6);

	long alt;
	Serial.print ("Altitude (m): ");
	if (nmea.getAltitude (alt))
		Serial.println (alt / 1000., 3);
	else
		Serial.println ("not available");

	Serial.print ("Speed: ");
	Serial.println (nmea.getSpeed () / 1000., 3);
	Serial.print ("Course: ");
	Serial.println (nmea.getCourse () / 1000., 3);

	Serial.println ("-----------------------");
	nmea.clear();
}

void setup () {
	Serial.begin (115200);
	if (SerialBT.begin ("ESP32test", ESP_SPP_ROLE_MASTER, SB_SPP_name, BT_SPP_address)) { //Bluetooth device name
		Serial.println ("The device started, now you can pair it with bluetooth!");
	}
	while (SerialBT.available ())
		SerialBT.read ();
}

void loop () {
	/*if (Serial.available ()) {
		SerialBT.write (Serial.read ());
	}
	if (SerialBT.available ()) {
		String line = SerialBT.readStringUntil ('\n');
		Serial.println (line);
	}*/
	while (SerialBT.available ())
	{
		//Fetch the character one by one
		char c = SerialBT.read ();
		Serial.print (c);
		//Pass the character to the library
		nmea.process (c);
	}
	static time_t last_print = 0;
	uint16_t period = 1000;

	if (millis () - last_print > period) {
		last_print = millis ();
		show_gps_data ();
	}
}