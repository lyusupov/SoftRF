#include "le3dp_rptparser.h"

// On SAMD boards where the native USB port is also the serial console, use
// Serial1 for the serial console. This applies to all SAMD boards except for
// Arduino Zero and M0 boards.
#if (USB_VID==0x2341 && defined(ARDUINO_SAMD_ZERO)) || (USB_VID==0x2a03 && defined(ARDUINO_SAM_ZERO))
#define SerialDebug SERIAL_PORT_MONITOR
#else
#define SerialDebug Serial1
#endif

JoystickReportParser::JoystickReportParser(JoystickEvents *evt) :
	joyEvents(evt)
{}

void JoystickReportParser::Parse(HID *hid, uint32_t is_rpt_id, uint32_t len, uint8_t *buf)
{
	bool match = true;

	// Checking if there are changes in report since the method was last called
	for (uint8_t i=0; i<RPT_GAMEPAD_LEN; i++) {
		if( buf[i] != oldPad[i] ) {
			match = false;
			break;
		}
  }
  	// Calling Game Pad event handler
	if (!match && joyEvents) {
		joyEvents->OnGamePadChanged((const GamePadEventData*)buf);

		for (uint8_t i=0; i<RPT_GAMEPAD_LEN; i++) oldPad[i] = buf[i];
	}
}

void JoystickEvents::OnGamePadChanged(const GamePadEventData *evt)
{
	SerialDebug.print("X: ");
	PrintHex<uint16_t>(evt->x, 0x80);
	SerialDebug.print(" Y: ");
	PrintHex<uint16_t>(evt->y, 0x80);
	SerialDebug.print(" Hat Switch: ");
	PrintHex<uint8_t>(evt->hat, 0x80);
	SerialDebug.print(" Twist: ");
	PrintHex<uint8_t>(evt->twist, 0x80);
	SerialDebug.print(" Slider: ");
	PrintHex<uint8_t>(evt->slider, 0x80);
  SerialDebug.print(" Buttons A: ");
	PrintHex<uint8_t>(evt->buttons_a, 0x80);
	SerialDebug.print(" Buttons B: ");
	PrintHex<uint8_t>(evt->buttons_b, 0x80);
	SerialDebug.println("");
}
