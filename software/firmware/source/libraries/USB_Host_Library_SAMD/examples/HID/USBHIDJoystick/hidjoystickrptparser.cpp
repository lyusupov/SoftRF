#include "hidjoystickrptparser.h"

// On SAMD boards where the native USB port is also the serial console, use
// Serial1 for the serial console. This applies to all SAMD boards except for
// Arduino Zero and M0 boards.
#if (USB_VID==0x2341 && defined(ARDUINO_SAMD_ZERO)) || (USB_VID==0x2a03 && defined(ARDUINO_SAM_ZERO))
#define SerialDebug SERIAL_PORT_MONITOR
#else
#define SerialDebug Serial1
#endif

JoystickReportParser::JoystickReportParser(JoystickEvents *evt) :
joyEvents(evt),
oldHat(0xDE),
oldButtons(0) {
        for (uint8_t i = 0; i < RPT_GEMEPAD_LEN; i++)
                oldPad[i] = 0xD;
}

void JoystickReportParser::Parse(HID *hid, uint32_t is_rpt_id, uint32_t len, uint8_t *buf) {
        bool match = true;

        // Checking if there are changes in report since the method was last called
        for (uint8_t i = 0; i < RPT_GEMEPAD_LEN; i++)
                if (buf[i] != oldPad[i]) {
                        match = false;
                        break;
                }

        // Calling Game Pad event handler
        if (!match && joyEvents) {
                joyEvents->OnGamePadChanged((const GamePadEventData*)buf);

                for (uint8_t i = 0; i < RPT_GEMEPAD_LEN; i++) oldPad[i] = buf[i];
        }

        uint8_t hat = (buf[5] & 0xF);

        // Calling Hat Switch event handler
        if (hat != oldHat && joyEvents) {
                joyEvents->OnHatSwitch(hat);
                oldHat = hat;
        }

        uint16_t buttons = (0x0000 | buf[6]);
        buttons <<= 4;
        buttons |= (buf[5] >> 4);
        uint16_t changes = (buttons ^ oldButtons);

        // Calling Button Event Handler for every button changed
        if (changes) {
                for (uint8_t i = 0; i < 0x0C; i++) {
                        uint16_t mask = (0x0001 << i);

                        if (((mask & changes) > 0) && joyEvents) {
                                if ((buttons & mask) > 0)
                                        joyEvents->OnButtonDn(i + 1);
                                else
                                        joyEvents->OnButtonUp(i + 1);
                        }
                }
                oldButtons = buttons;
        }
}

void JoystickEvents::OnGamePadChanged(const GamePadEventData *evt) {
        SerialDebug.print("X1: ");
        PrintHex<uint8_t > (evt->X, 0x80);
        SerialDebug.print("\tY1: ");
        PrintHex<uint8_t > (evt->Y, 0x80);
        SerialDebug.print("\tX2: ");
        PrintHex<uint8_t > (evt->Z1, 0x80);
        SerialDebug.print("\tY2: ");
        PrintHex<uint8_t > (evt->Z2, 0x80);
        SerialDebug.print("\tRz: ");
        PrintHex<uint8_t > (evt->Rz, 0x80);
        SerialDebug.println("");
}

void JoystickEvents::OnHatSwitch(uint8_t hat) {
        SerialDebug.print("Hat Switch: ");
        PrintHex<uint8_t > (hat, 0x80);
        SerialDebug.println("");
}

void JoystickEvents::OnButtonUp(uint8_t but_id) {
        SerialDebug.print("Up: ");
        SerialDebug.println(but_id, DEC);
}

void JoystickEvents::OnButtonDn(uint8_t but_id) {
        SerialDebug.print("Dn: ");
        SerialDebug.println(but_id, DEC);
}
