#include <usbhub.h>

#include "pgmstrings.h"

// On SAMD boards where the native USB port is also the serial console, use
// Serial1 for the serial console. This applies to all SAMD boards except for
// Arduino Zero and M0 boards.
#if (USB_VID==0x2341 && defined(ARDUINO_SAMD_ZERO)) || (USB_VID==0x2a03 && defined(ARDUINO_SAM_ZERO))
#define SerialDebug SERIAL_PORT_MONITOR
#else
#define SerialDebug Serial1
#endif

USBHost     UsbH;
USBHub  Hub1(&UsbH);
//USBHub  Hub2(&UsbH);
//USBHub  Hub3(&UsbH);
//USBHub  Hub4(&UsbH);
//USBHub  Hub5(&UsbH);
//USBHub  Hub6(&UsbH);
//USBHub  Hub7(&UsbH);

void print_hex(int v, int num_places);
void printintfdescr( uint8_t* descr_ptr );
uint8_t getconfdescr( uint8_t addr, uint8_t conf );
void printconfdescr( uint8_t* descr_ptr );
void printunkdescr( uint8_t* descr_ptr );
void printepdescr( uint8_t* descr_ptr );
void printProgStr(const prog_char str[]);
void printHIDdescr( uint8_t* descr_ptr );

void PrintAllAddresses(UsbDeviceDefinition *pdev)
{
  UsbDeviceAddress adr;
  adr.devAddress = pdev->address.devAddress;
  SerialDebug.print("\r\nAddr:");
  SerialDebug.print(adr.devAddress, HEX);
  SerialDebug.print("(");
  SerialDebug.print(adr.bmHub, HEX);
  SerialDebug.print(".");
  SerialDebug.print(adr.bmParent, HEX);
  SerialDebug.print(".");
  SerialDebug.print(adr.bmAddress, HEX);
  SerialDebug.println(")");
}

void PrintAddress(uint8_t addr)
{
  UsbDeviceAddress adr;
  adr.devAddress = addr;
  SerialDebug.print("\r\nADDR:\t");
  SerialDebug.println(adr.devAddress, HEX);
  SerialDebug.print("DEV:\t");
  SerialDebug.println(adr.bmAddress, HEX);
  SerialDebug.print("PRNT:\t");
  SerialDebug.println(adr.bmParent, HEX);
  SerialDebug.print("HUB:\t");
  SerialDebug.println(adr.bmHub, HEX);
}

void setup()
{
  SerialDebug.begin( 115200 );
  SerialDebug.println("Start");

  if (UsbH.Init())
    SerialDebug.println("USB host did not start.");

  delay( 200 );
}

uint8_t getdevdescr( uint8_t addr, uint8_t &num_conf );

void PrintDescriptors(uint8_t addr)
{
  uint8_t rcode = 0;
  uint8_t num_conf = 0;

  rcode = getdevdescr( (uint8_t)addr, num_conf );
  if ( rcode )
  {
    printProgStr(Gen_Error_str);
    print_hex( rcode, 8 );
  }
  SerialDebug.print("\r\n");

  for (int i = 0; i < num_conf; i++)
  {
    rcode = getconfdescr( addr, i );                 // get configuration descriptor
    if ( rcode )
    {
      printProgStr(Gen_Error_str);
      print_hex(rcode, 8);
    }
    SerialDebug.println("\r\n");
  }
}

void PrintAllDescriptors(UsbDeviceDefinition *pdev)
{
  SerialDebug.println("\r\n");
  print_hex(pdev->address.devAddress, 8);
  SerialDebug.println("\r\n--");
  PrintDescriptors( pdev->address.devAddress );
}

void loop()
{
  UsbH.Task();

  if ( UsbH.getUsbTaskState() == USB_STATE_RUNNING )
  {
    UsbH.ForEachUsbDevice(&PrintAllDescriptors);
    UsbH.ForEachUsbDevice(&PrintAllAddresses);

    while ( 1 ) { // stop
#ifdef ESP8266
        yield(); // needed in order to reset the watchdog timer on the ESP8266
#endif
    }
  }
}

uint8_t getdevdescr( uint8_t addr, uint8_t &num_conf )
{
  USB_DEVICE_DESCRIPTOR buf;
  uint8_t rcode;
  rcode = UsbH.getDevDescr( addr, 0, 0x12, ( uint8_t *)&buf );
  if ( rcode ) {
    return ( rcode );
  }
  printProgStr(Dev_Header_str);
  printProgStr(Dev_Length_str);
  print_hex( buf.bLength, 8 );
  printProgStr(Dev_Type_str);
  print_hex( buf.bDescriptorType, 8 );
  printProgStr(Dev_Version_str);
  print_hex( buf.bcdUSB, 16 );
  printProgStr(Dev_Class_str);
  print_hex( buf.bDeviceClass, 8 );
  printProgStr(Dev_Subclass_str);
  print_hex( buf.bDeviceSubClass, 8 );
  printProgStr(Dev_Protocol_str);
  print_hex( buf.bDeviceProtocol, 8 );
  printProgStr(Dev_Pktsize_str);
  print_hex( buf.bMaxPacketSize0, 8 );
  printProgStr(Dev_Vendor_str);
  print_hex( buf.idVendor, 16 );
  printProgStr(Dev_Product_str);
  print_hex( buf.idProduct, 16 );
  printProgStr(Dev_Revision_str);
  print_hex( buf.bcdDevice, 16 );
  printProgStr(Dev_Mfg_str);
  print_hex( buf.iManufacturer, 8 );
  printProgStr(Dev_Prod_str);
  print_hex( buf.iProduct, 8 );
  printProgStr(Dev_Serial_str);
  print_hex( buf.iSerialNumber, 8 );
  printProgStr(Dev_Nconf_str);
  print_hex( buf.bNumConfigurations, 8 );
  num_conf = buf.bNumConfigurations;
  return ( 0 );
}

void printhubdescr(uint8_t *descrptr, uint8_t addr)
{
  HubDescriptor  *pHub = (HubDescriptor*) descrptr;
  uint8_t        len = *((uint8_t*)descrptr);

  printProgStr(PSTR("\r\n\r\nHub Descriptor:\r\n"));
  printProgStr(PSTR("bDescLength:\t\t"));
  SerialDebug.println(pHub->bDescLength, HEX);

  printProgStr(PSTR("bDescriptorType:\t"));
  SerialDebug.println(pHub->bDescriptorType, HEX);

  printProgStr(PSTR("bNbrPorts:\t\t"));
  SerialDebug.println(pHub->bNbrPorts, HEX);

  printProgStr(PSTR("LogPwrSwitchMode:\t"));
  SerialDebug.println(pHub->LogPwrSwitchMode, BIN);

  printProgStr(PSTR("CompoundDevice:\t\t"));
  SerialDebug.println(pHub->CompoundDevice, BIN);

  printProgStr(PSTR("OverCurrentProtectMode:\t"));
  SerialDebug.println(pHub->OverCurrentProtectMode, BIN);

  printProgStr(PSTR("TTThinkTime:\t\t"));
  SerialDebug.println(pHub->TTThinkTime, BIN);

  printProgStr(PSTR("PortIndicatorsSupported:"));
  SerialDebug.println(pHub->PortIndicatorsSupported, BIN);

  printProgStr(PSTR("Reserved:\t\t"));
  SerialDebug.println(pHub->Reserved, HEX);

  printProgStr(PSTR("bPwrOn2PwrGood:\t\t"));
  SerialDebug.println(pHub->bPwrOn2PwrGood, HEX);

  printProgStr(PSTR("bHubContrCurrent:\t"));
  SerialDebug.println(pHub->bHubContrCurrent, HEX);

  for (uint8_t i = 7; i < len; i++)
    print_hex(descrptr[i], 8);

  //for (uint8_t i=1; i<=pHub->bNbrPorts; i++)
  //    PrintHubPortStatus(&UsbH, addr, i, 1);
}

uint8_t getconfdescr( uint8_t addr, uint8_t conf )
{
  uint8_t buf[ BUFSIZE ];
  uint8_t* buf_ptr = buf;
  uint8_t rcode;
  uint8_t descr_length;
  uint8_t descr_type;
  uint16_t total_length;
  rcode = UsbH.getConfDescr( addr, 0, 4, conf, buf );  //get total length
  LOBYTE( total_length ) = buf[ 2 ];
  HIBYTE( total_length ) = buf[ 3 ];
  if( total_length > sizeof(buf)) {    //check if total length is larger than buffer
    printProgStr(Conf_Trunc_str);
    total_length = sizeof(buf);
  }
  rcode = UsbH.getConfDescr( addr, 0, total_length, conf, buf ); //get the whole descriptor
  while ( buf_ptr < buf + total_length ) { //parsing descriptors
    descr_length = *( buf_ptr );
    descr_type = *( buf_ptr + 1 );
    switch ( descr_type ) {
      case ( USB_DESCRIPTOR_CONFIGURATION ):
        printconfdescr( buf_ptr );
        break;
      case ( USB_DESCRIPTOR_INTERFACE ):
        printintfdescr( buf_ptr );
        break;
      case ( USB_DESCRIPTOR_ENDPOINT ):
        printepdescr( buf_ptr );
        break;
      case 0x21:  // HID Descriptor
        printHIDdescr( buf_ptr );
        break;
      case 0x29:
        printhubdescr( buf_ptr, addr );
        break;
      default:
        printunkdescr( buf_ptr );
        break;
    }//switch( descr_type
    buf_ptr = ( buf_ptr + descr_length );    //advance buffer pointer
  }//while( buf_ptr <=...
  return ( rcode );
}
/* prints hex numbers with leading zeroes */
// copyright, Peter H Anderson, Baltimore, MD, Nov, '07
// source: http://www.phanderson.com/arduino/arduino_display.html
void print_hex(int v, int num_places)
{
  int mask = 0, n, num_nibbles, digit;

  for (n = 1; n <= num_places; n++) {
    mask = (mask << 1) | 0x0001;
  }
  v = v & mask; // truncate v to specified number of places

  num_nibbles = num_places / 4;
  if ((num_places % 4) != 0) {
    ++num_nibbles;
  }
  do {
    digit = ((v >> (num_nibbles - 1) * 4)) & 0x0f;
    SerialDebug.print(digit, HEX);
  }
  while (--num_nibbles);
}
/* function to print configuration descriptor */
void printconfdescr( uint8_t* descr_ptr )
{
  USB_CONFIGURATION_DESCRIPTOR* conf_ptr = ( USB_CONFIGURATION_DESCRIPTOR* )descr_ptr;
  printProgStr(Conf_Header_str);
  printProgStr(Conf_Totlen_str);
  print_hex( conf_ptr->wTotalLength, 16 );
  printProgStr(Conf_Nint_str);
  print_hex( conf_ptr->bNumInterfaces, 8 );
  printProgStr(Conf_Value_str);
  print_hex( conf_ptr->bConfigurationValue, 8 );
  printProgStr(Conf_String_str);
  print_hex( conf_ptr->iConfiguration, 8 );
  printProgStr(Conf_Attr_str);
  print_hex( conf_ptr->bmAttributes, 8 );
  printProgStr(Conf_Pwr_str);
  print_hex( conf_ptr->bMaxPower, 8 );
  return;
}
/* function to print interface descriptor */
void printintfdescr( uint8_t* descr_ptr )
{
  USB_INTERFACE_DESCRIPTOR* intf_ptr = ( USB_INTERFACE_DESCRIPTOR* )descr_ptr;
  printProgStr(Int_Header_str);
  printProgStr(Int_Number_str);
  print_hex( intf_ptr->bInterfaceNumber, 8 );
  printProgStr(Int_Alt_str);
  print_hex( intf_ptr->bAlternateSetting, 8 );
  printProgStr(Int_Endpoints_str);
  print_hex( intf_ptr->bNumEndpoints, 8 );
  printProgStr(Int_Class_str);
  print_hex( intf_ptr->bInterfaceClass, 8 );
  printProgStr(Int_Subclass_str);
  print_hex( intf_ptr->bInterfaceSubClass, 8 );
  printProgStr(Int_Protocol_str);
  print_hex( intf_ptr->bInterfaceProtocol, 8 );
  printProgStr(Int_String_str);
  print_hex( intf_ptr->iInterface, 8 );
  return;
}

/* function to print HID descriptor */
void printHIDdescr( uint8_t* descr_ptr )
{
	USB_HID_DESCRIPTOR* ep_ptr = ( USB_HID_DESCRIPTOR* )descr_ptr;

    printProgStr(PSTR("\r\n\r\nHID Descriptor:\r\n"));
    printProgStr(PSTR("HID Class Release:\t"));
	print_hex( ep_ptr->bcdHID, 16 );
    printProgStr(PSTR("\r\nCountry Code:\t\t"));
	print_hex( ep_ptr->bCountryCode, 8 );
    printProgStr(PSTR("\r\nNumb Class Descriptor:\t"));
	print_hex( ep_ptr->bNumDescriptors, 8 );
    printProgStr(PSTR("\r\nDescriptor Type:\t"));
	if( ep_ptr->bDescrType == 0x22 )
		printProgStr(PSTR("REPORT DESCRIPTOR"));
	else
		print_hex( ep_ptr->bDescrType, 8 );
    printProgStr(PSTR("\r\nCSize Report Descr:\t"));
	print_hex( ep_ptr->wDescriptorLength, 16 );
}

/* function to print endpoint descriptor */
void printepdescr( uint8_t* descr_ptr )
{
  uint8_t transfer_type;

  USB_ENDPOINT_DESCRIPTOR* ep_ptr = ( USB_ENDPOINT_DESCRIPTOR* )descr_ptr;
  printProgStr(End_Header_str);
  printProgStr(End_Address_str);
  if( 0x80 & ep_ptr->bEndpointAddress ) printProgStr(PSTR("IN\t\t"));
  else printProgStr(PSTR("OUT\t\t"));
  print_hex( (ep_ptr->bEndpointAddress & 0xF), 8 );
  printProgStr(End_Attr_str);
  transfer_type = ep_ptr->bmAttributes & bmUSB_TRANSFER_TYPE;
  if( transfer_type == USB_TRANSFER_TYPE_INTERRUPT ) printProgStr(PSTR("INTERRUPT\t"));
  else if( transfer_type == USB_TRANSFER_TYPE_BULK ) printProgStr(PSTR("BULK\t"));
  else if( transfer_type == USB_TRANSFER_TYPE_ISOCHRONOUS ) printProgStr(PSTR("ISO\t"));
  print_hex( ep_ptr->bmAttributes, 8 );
  printProgStr(End_Pktsize_str);
  print_hex( ep_ptr->wMaxPacketSize, 16 );
  printProgStr(End_Interval_str);
  print_hex( ep_ptr->bInterval, 8 );

  return;
}
/*function to print unknown descriptor */
void printunkdescr( uint8_t* descr_ptr )
{
  uint8_t length = *descr_ptr;
  uint8_t i;
  printProgStr(Unk_Header_str);
  printProgStr(Unk_Length_str);
  print_hex( *descr_ptr, 8 );
  printProgStr(Unk_Type_str);
  print_hex( *(descr_ptr + 1 ), 8 );
  printProgStr(Unk_Contents_str);
  descr_ptr += 2;
  for ( i = 0; i < length; i++ ) {
    print_hex( *descr_ptr, 8 );
    descr_ptr++;
  }
}


/* Print a string from Program Memory directly to save RAM */
void printProgStr(const char* str)
{
  char c;
  if (!str) return;
  while ((c = pgm_read_byte(str++)))
    SerialDebug.print(c);
}
