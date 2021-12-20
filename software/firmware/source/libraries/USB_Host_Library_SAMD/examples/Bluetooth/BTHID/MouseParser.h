#ifndef __mouserptparser_h__
#define __mouserptparser_h__

// On SAMD boards where the native USB port is also the serial console, use
// Serial1 for the serial console. This applies to all SAMD boards except for
// Arduino Zero and M0 boards.
#if (USB_VID==0x2341 && defined(ARDUINO_SAMD_ZERO)) || (USB_VID==0x2a03 && defined(ARDUINO_SAM_ZERO))
#define SerialDebug SERIAL_PORT_MONITOR
#else
#define SerialDebug Serial1
#endif

class MouseRptParser : public MouseReportParser {
  protected:
    virtual void OnMouseMove(MOUSEINFO *mi);
    virtual void OnLeftButtonUp(MOUSEINFO *mi);
    virtual void OnLeftButtonDown(MOUSEINFO *mi);
    virtual void OnRightButtonUp(MOUSEINFO *mi);
    virtual void OnRightButtonDown(MOUSEINFO *mi);
    virtual void OnMiddleButtonUp(MOUSEINFO *mi);
    virtual void OnMiddleButtonDown(MOUSEINFO *mi);
};

void MouseRptParser::OnMouseMove(MOUSEINFO *mi) {
  SerialDebug.print(F("dx="));
  SerialDebug.print(mi->dX, DEC);
  SerialDebug.print(F(" dy="));
  SerialDebug.println(mi->dY, DEC);
};

void MouseRptParser::OnLeftButtonUp(MOUSEINFO *mi) {
  SerialDebug.println(F("L Butt Up"));
};

void MouseRptParser::OnLeftButtonDown(MOUSEINFO *mi) {
  SerialDebug.println(F("L Butt Dn"));
};

void MouseRptParser::OnRightButtonUp(MOUSEINFO *mi) {
  SerialDebug.println(F("R Butt Up"));
};

void MouseRptParser::OnRightButtonDown(MOUSEINFO *mi) {
  SerialDebug.println(F("R Butt Dn"));
};

void MouseRptParser::OnMiddleButtonUp(MOUSEINFO *mi) {
  SerialDebug.println(F("M Butt Up"));
};

void MouseRptParser::OnMiddleButtonDown(MOUSEINFO *mi) {
  SerialDebug.println(F("M Butt Dn"));
};

#endif
