/* Parser for standard HID scale (usage page 0x8d) data input report (ID 3) */
#if defined(ARDUINO_SAM_DUE) || defined(ARDUINO_ARCH_SAMD)
#include <avr/dtostrf.h>
#endif
#include "scale_rptparser.h"

// On SAMD boards where the native USB port is also the serial console, use
// Serial1 for the serial console. This applies to all SAMD boards except for
// Arduino Zero and M0 boards.
#if (USB_VID==0x2341 && defined(ARDUINO_SAMD_ZERO)) || (USB_VID==0x2a03 && defined(ARDUINO_SAM_ZERO))
#define SerialDebug SERIAL_PORT_MONITOR
#else
#define SerialDebug Serial1
#endif

const char* UNITS[13] = {
    "units",        // unknown unit
    "mg",           // milligram
    "g",            // gram
    "kg",           // kilogram
    "cd",           // carat
    "taels",        // lian
    "gr",           // grain
    "dwt",          // pennyweight
    "tonnes",       // metric tons
    "tons",         // avoir ton
    "ozt",          // troy ounce
    "oz",           // ounce
    "lbs"           // pound
};

ScaleReportParser::ScaleReportParser(ScaleEvents *evt) :
	scaleEvents(evt)
{}

void ScaleReportParser::Parse(HID *hid, uint32_t is_rpt_id, uint32_t len, uint8_t *buf)
{
	bool match = true;

	// Checking if there are changes in report since the method was last called
	for (uint8_t i=0; i<RPT_SCALE_LEN; i++) {
		if( buf[i] != oldScale[i] ) {
			match = false;
			break;
		}
  }
  	// Calling Game Pad event handler
	if (!match && scaleEvents) {
		scaleEvents->OnScaleChanged((const ScaleEventData*)buf);

		for (uint8_t i=0; i<RPT_SCALE_LEN; i++) oldScale[i] = buf[i];
	}
}

ScaleEvents::ScaleEvents( Max_LCD* pLCD ) :
	
	pLcd( pLCD )

{}

void ScaleEvents::LcdPrint( const char* str )
{
	
	while( *str ) {
		
		pLcd->write(	*str++ );
		
	}
}

void ScaleEvents::OnScaleChanged(const ScaleEventData *evt)
{
	
	pLcd->clear();
  pLcd->home();
  pLcd->setCursor(0,0);
	
	if( evt->reportID != 3 ) {
		
		const char inv_report[]="Invalid report!";
		
		SerialDebug.println(inv_report);
		LcdPrint(inv_report);
		
		return;
		
	}//if( evt->reportID != 3...
	
	switch( evt->status ) {
		
		case REPORT_FAULT:
			SerialDebug.println(F("Report fault"));
			break;
			
		case ZEROED:
			SerialDebug.println(F("Scale zero set"));
			break;
			
		case WEIGHING: {
			
			const char progress[] = "Weighing...";
			SerialDebug.println(progress);
			LcdPrint(progress);
			break;
		}
		
		case WEIGHT_VALID: {
			
			char buf[10];
      double weight = evt->weight * pow( 10, evt->exp );

      	

      	SerialDebug.print(F("Weight: "));
				SerialDebug.print( weight );
				SerialDebug.print(F(" "));
				SerialDebug.println( UNITS[ evt->unit ]);
				
				LcdPrint("Weight: ");
				dtostrf( weight, 4, 2, buf );
				LcdPrint( buf );
				LcdPrint( UNITS[ evt->unit ]);
			
			break;
			
		}//case WEIGHT_VALID...
			
		case WEIGHT_NEGATIVE: {
			
			const char negweight[] = "Negative weight";
			SerialDebug.println(negweight);
			LcdPrint(negweight);
			break;
		}
			
		case OVERWEIGHT: {
		
			const char overweight[] = "Max.weight reached";
			SerialDebug.println(overweight);
			LcdPrint( overweight );
			break;
		}
		
		case CALIBRATE_ME:
			
			SerialDebug.println(F("Scale calibration required"));
			break;
			
		case ZERO_ME:
			
			SerialDebug.println(F("Scale zeroing required"));
			break;
			
		default:
			
			SerialDebug.print(F("Undefined status code: "));
			SerialDebug.println( evt->status );
			break;	
			
	}//switch( evt->status...

}
