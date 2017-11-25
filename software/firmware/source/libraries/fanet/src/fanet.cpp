/*
Copyright (C) 2016-2017 Juergen Eckert
All rights reserved.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <Arduino.h>

#include "fanet.h"

#include "com/bm78.h"
#include "com/serial.h"
#include "app.h"
#include "fanet_stack/fmac.h"

//test
#include "fanet_stack/sx1272.h"

/* fanet */

//TODO: neighbor list of forwarded packets? use 1bit?
//TODO: USB CRS for HSI48 on USB SOF


#ifdef FANET_USB
/* function for switching back and forth between serial and usb */
void switch_serial()
{
	static uint32_t ss_nextcheck = 0;
	static boolean second_serial = false;

	if(millis() > ss_nextcheck)
	{
		if(!second_serial && SerialFANET_USB)
		{
			/* USB connected, switching */
			serial_int.begin(SerialFANET_USB);
#ifdef FANET_BLUETOOTH
			bm78.end();
#endif
			second_serial = true;
			//SerialDEBUG.println("serial->usb");
		}
		else if(second_serial && !SerialFANET_USB &&
#ifdef FANET_BLUETOOTH
				bm78.begin(SerialFANET)
#else
				true
#endif
				)
		{
			serial_int.begin(SerialFANET);
			second_serial = false;
			//SerialDEBUG.println("usb->serial");
		}

		/* only check every 2 sec as operator bool() as a 10ms "bugfix" delay, see Arduino Zero CDC.cpp */
		ss_nextcheck = millis() + 2000;
	}
}
#endif

#if 0
void switch_to_usb()
{
	static bool isUSB = false;

	/* return if already switched or usb not configured or no data present */
	if(isUSB || SerialFANET_USB.isConfigured() == false || !SerialFANET_USB.available())
		return;

#ifdef SerialDEBUG
	SerialDEBUG.println("### Switching to USB");
#endif

	/* switching to usb */
	serial_int.begin(SerialFANET_USB);

#ifdef FANET_BLUETOOTH
	/* deactivate blueooth module and remove self-power */
	bm78.end();
	digitalWrite(FANET_PWR, LOW);
#endif

	/* we can never go back as lib USB STM32 always returns true, once configured... todo */
	isUSB = true;
}
#endif

#if !defined(ESP8266)

void setup()
{
#ifdef FANET_BLUETOOTH
	/* self powered */
	pinMode(FANET_BTN, INPUT_PULLDOWN);
	pinMode(FANET_CHG, INPUT_PULLUP);
	pinMode(FANET_PWR, OUTPUT);
	digitalWrite(FANET_PWR, HIGH);
#endif

	/* LED */
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);

	/* serial */
	SerialFANET.begin(115200);
	serial_int.begin(SerialFANET);
#ifdef FANET_USB
	SerialFANET_USB.begin(115200);
#endif

	/* Bluetooth */
#ifdef FANET_BLUETOOTH
	while(bm78.begin(SerialFANET) == false)
	{
		serial_int.println(FN_REPLY_BT_FAILED);
#if FANET_USB
		if(SerialFANET_USB.isConfigured() && SerialFANET_USB.available())
		{
			bm78.end();
			digitalWrite(FANET_PWR, LOW);
			break;
		}
#endif
		delay(1000);
	}
#endif

	/* Radio */
	app.begin(serial_int);
	while(fmac.begin(app) == false)
	{
#ifdef FANET_USB
		switch_serial();
#endif
		serial_int.println(FN_REPLYE_RADIO_FAILED);
		delay(1000);
	}

	digitalWrite(LED_BUILTIN, LOW);
	serial_int.println(FN_REPLYM_INITIALIZED);

	/* Button pressed */
#ifdef FANET_BLUETOOTH
	digitalWrite(FANET_PWR, digitalRead(FANET_BTN));
#endif
}

#ifdef FANET_BLUETOOTH
void led_handle()
{
	static uint32_t prev = 0;
	uint32_t bright = 20;
	uint32_t dark = serial_int.any_actitity()?1980:480;

	if(digitalRead(FANET_CHG) == LOW)
	{
		bright += 200;
		dark -= 200;
	}

	if(digitalRead(LED_BUILTIN))
	{
		/* currently bright */
		if(prev + bright < millis())
		{
			digitalWrite(BOARD_LED_PIN, LOW);
			prev = millis();
		}
	}
	else
	{
		/* currently dark */
		if(prev + dark < millis())
		{
			digitalWrite(BOARD_LED_PIN, HIGH);
			prev = millis();
		}
	}


}

void pwr_handle()
{
	/* button */
	static uint32_t last_btn_act = 0;
	static uint32_t off_time = 0;
	static boolean btn_last = false;
	boolean btn_current = digitalRead(FANET_BTN);
	if(btn_current != btn_last)
		last_btn_act = millis();
	btn_last = btn_current;

	if(btn_current && millis() > last_btn_act+FANET_BT_OFF_MS)
	{
		/* off */
		digitalWrite(FANET_PWR, LOW);
		off_time = millis();

		digitalWrite(LED_BUILTIN, HIGH);
		delay(50);
		digitalWrite(LED_BUILTIN, LOW);
		delay(100);
	}

	/* give the CPU 500ms to loose power */
	if(off_time+500 > millis())
		return;

	/* serial */
	uint32_t last_serial = serial_int.get_lastactivity();

	/* change power state */
	if(millis() > max(last_serial, last_btn_act) + FANET_BT_AUTOFF_MS)
	{
		/* release power pin */
		digitalWrite(FANET_PWR, LOW);
	}
	else
	{
		/* enforce power */
		digitalWrite(FANET_PWR, HIGH);
	}
}
#endif

void loop()
{
#ifdef FANET_USB
	/* use alternative serial channel? */
	switch_serial();
#endif

	/* get commands from serial */
	serial_int.handle_rx();

	/* update MAC state machine */
	fmac.handle();

#ifdef FANET_BLUETOOTH
	led_handle();
	pwr_handle();
#endif

}
#endif /* ESP8266 */
