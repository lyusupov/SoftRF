#include "lib_crc.h"

#if defined(ESP8266)
#include <pgmspace.h>
#endif

    /*******************************************************************\
    *                                                                   *
    *   Library         : lib_crc                                       *
    *   File            : lib_crc.c                                     *
    *   Author          : Lammert Bies  1999-2008                       *
    *   E-mail          : info@lammertbies.nl                           *
    *   Language        : ANSI C                                        *
    *                                                                   *
    *                                                                   *
    *   Description                                                     *
    *   ===========                                                     *
    *                                                                   *
    *   The file lib_crc.c contains the private  and  public  func-     *
    *   tions  used  for  the  calculation of CRC-16, CRC-CCITT and     *
    *   CRC-32 cyclic redundancy values.                                *
    *                                                                   *
    *                                                                   *
    *   Dependencies                                                    *
    *   ============                                                    *
    *                                                                   *
    *   lib_crc.h       CRC definitions and prototypes                  *
    *                                                                   *
    *                                                                   *
    *   Modification history                                            *
    *   ====================                                            *
    *                                                                   *
    *   Date        Version Comment                                     *
    *                                                                   *
    *   2008-04-20  1.16    Added CRC-CCITT calculation for Kermit      *
    *                                                                   *
    *   2007-04-01  1.15    Added CRC16 calculation for Modbus          *
    *                                                                   *
    *   2007-03-28  1.14    Added CRC16 routine for Sick devices        *
    *                                                                   *
    *   2005-12-17  1.13    Added CRC-CCITT with initial 0x1D0F         *
    *                                                                   *
    *   2005-05-14  1.12    Added CRC-CCITT with start value 0          *
    *                                                                   *
    *   2005-02-05  1.11    Fixed bug in CRC-DNP routine                *
    *                                                                   *
    *   2005-02-04  1.10    Added CRC-DNP routines                      *
    *                                                                   *
    *   1999-02-21  1.01    Added FALSE and TRUE mnemonics              *
    *                                                                   *
    *   1999-01-22  1.00    Initial source                              *
    *                                                                   *
    \*******************************************************************/



    /*******************************************************************\
    *                                                                   *
    *   #define P_xxxx                                                  *
    *                                                                   *
    *   The CRC's are computed using polynomials. The  coefficients     *
    *   for the algorithms are defined by the following constants.      *
    *                                                                   *
    \*******************************************************************/

#define                 P_16        0xA001
#define                 P_32        0xEDB88320L
#define                 P_CCITT     0x1021
#define                 P_DNP       0xA6BC
#define                 P_KERMIT    0x8408
#define                 P_SICK      0x8005



    /*******************************************************************\
    *                                                                   *
    *   static int crc_tab...init                                       *
    *   static unsigned ... crc_tab...[]                                *
    *                                                                   *
    *   The algorithms use tables with precalculated  values.  This     *
    *   speeds  up  the calculation dramaticaly. The first time the     *
    *   CRC function is called, the table for that specific  calcu-     *
    *   lation  is set up. The ...init variables are used to deter-     *
    *   mine if the initialization has taken place. The  calculated     *
    *   values are stored in the crc_tab... arrays.                     *
    *                                                                   *
    *   The variables are declared static. This makes them  invisi-     *
    *   ble for other modules of the program.                           *
    *                                                                   *
    \*******************************************************************/

static int              crc_tab16_init          = FALSE;
static int              crc_tab32_init          = FALSE;
static int              crc_tabccitt_init       = FALSE;
static int              crc_tabdnp_init         = FALSE;
static int              crc_tabkermit_init      = FALSE;

static unsigned short   crc_tab16[256];
static unsigned long    crc_tab32[256];
static unsigned short   crc_tabccitt[256];
static unsigned short   crc_tabdnp[256];
static unsigned short   crc_tabkermit[256];



    /*******************************************************************\
    *                                                                   *
    *   static void init_crc...tab();                                   *
    *                                                                   *
    *   Three local functions are used  to  initialize  the  tables     *
    *   with values for the algorithm.                                  *
    *                                                                   *
    \*******************************************************************/

static void             init_crc16_tab( void );
static void             init_crc32_tab( void );
static void             init_crcccitt_tab( void );
static void             init_crcdnp_tab( void );
static void             init_crckermit_tab( void );



    /*******************************************************************\
    *                                                                   *
    *   unsigned short update_crc_ccitt( unsigned long crc, char c );   *
    *                                                                   *
    *   The function update_crc_ccitt calculates  a  new  CRC-CCITT     *
    *   value  based  on the previous value of the CRC and the next     *
    *   byte of the data to be checked.                                 *
    *                                                                   *
    \*******************************************************************/

unsigned short update_crc_ccitt( unsigned short crc, char c ) {

    unsigned short tmp, short_c;

    short_c  = 0x00ff & (unsigned short) c;

    if ( ! crc_tabccitt_init ) init_crcccitt_tab();

    tmp = (crc >> 8) ^ short_c;
    crc = (crc << 8) ^ crc_tabccitt[tmp];

    return crc;

}  /* update_crc_ccitt */



    /*******************************************************************\
    *                                                                   *
    *   unsigned short update_crc_sick(                                 *
    *             unsigned long crc, char c, char prev_byte );          *
    *                                                                   *
    *   The function  update_crc_sick  calculates  a  new  CRC-SICK     *
    *   value  based  on the previous value of the CRC and the next     *
    *   byte of the data to be checked.                                 *
    *                                                                   *
    \*******************************************************************/

unsigned short update_crc_sick( unsigned short crc, char c, char prev_byte ) {

    unsigned short short_c, short_p;

    short_c  =   0x00ff & (unsigned short) c;
    short_p  = ( 0x00ff & (unsigned short) prev_byte ) << 8;

    if ( crc & 0x8000 ) crc = ( crc << 1 ) ^ P_SICK;
    else                crc =   crc << 1;

    crc &= 0xffff;
    crc ^= ( short_c | short_p );

    return crc;

}  /* update_crc_sick */



    /*******************************************************************\
    *                                                                   *
    *   unsigned short update_crc_16( unsigned short crc, char c );     *
    *                                                                   *
    *   The function update_crc_16 calculates a  new  CRC-16  value     *
    *   based  on  the  previous value of the CRC and the next byte     *
    *   of the data to be checked.                                      *
    *                                                                   *
    \*******************************************************************/

unsigned short update_crc_16( unsigned short crc, char c ) {

    unsigned short tmp, short_c;

    short_c = 0x00ff & (unsigned short) c;

    if ( ! crc_tab16_init ) init_crc16_tab();

    tmp =  crc       ^ short_c;
    crc = (crc >> 8) ^ crc_tab16[ tmp & 0xff ];

    return crc;

}  /* update_crc_16 */



    /*******************************************************************\
    *                                                                   *
    *   unsigned short update_crc_kermit( unsigned short crc, char c ); *
    *                                                                   *
    *   The function update_crc_kermit calculates a  new  CRC value     *
    *   based  on  the  previous value of the CRC and the next byte     *
    *   of the data to be checked.                                      *
    *                                                                   *
    \*******************************************************************/

unsigned short update_crc_kermit( unsigned short crc, char c ) {

    unsigned short tmp, short_c;

    short_c = 0x00ff & (unsigned short) c;

    if ( ! crc_tabkermit_init ) init_crckermit_tab();

    tmp =  crc       ^ short_c;
    crc = (crc >> 8) ^ crc_tabkermit[ tmp & 0xff ];

    return crc;

}  /* update_crc_kermit */



    /*******************************************************************\
    *                                                                   *
    *   unsigned short update_crc_dnp( unsigned short crc, char c );    *
    *                                                                   *
    *   The function update_crc_dnp calculates a new CRC-DNP  value     *
    *   based  on  the  previous value of the CRC and the next byte     *
    *   of the data to be checked.                                      *
    *                                                                   *
    \*******************************************************************/

unsigned short update_crc_dnp( unsigned short crc, char c ) {

    unsigned short tmp, short_c;

    short_c = 0x00ff & (unsigned short) c;

    if ( ! crc_tabdnp_init ) init_crcdnp_tab();

    tmp =  crc       ^ short_c;
    crc = (crc >> 8) ^ crc_tabdnp[ tmp & 0xff ];

    return crc;

}  /* update_crc_dnp */



    /*******************************************************************\
    *                                                                   *
    *   unsigned long update_crc_32( unsigned long crc, char c );       *
    *                                                                   *
    *   The function update_crc_32 calculates a  new  CRC-32  value     *
    *   based  on  the  previous value of the CRC and the next byte     *
    *   of the data to be checked.                                      *
    *                                                                   *
    \*******************************************************************/

unsigned long update_crc_32( unsigned long crc, char c ) {

    unsigned long tmp, long_c;

    long_c = 0x000000ffL & (unsigned long) c;

    if ( ! crc_tab32_init ) init_crc32_tab();

    tmp = crc ^ long_c;
    crc = (crc >> 8) ^ crc_tab32[ tmp & 0xff ];

    return crc;

}  /* update_crc_32 */



    /*******************************************************************\
    *                                                                   *
    *   static void init_crc16_tab( void );                             *
    *                                                                   *
    *   The function init_crc16_tab() is used  to  fill  the  array     *
    *   for calculation of the CRC-16 with values.                      *
    *                                                                   *
    \*******************************************************************/

static void init_crc16_tab( void ) {

    int i, j;
    unsigned short crc, c;

    for (i=0; i<256; i++) {

        crc = 0;
        c   = (unsigned short) i;

        for (j=0; j<8; j++) {

            if ( (crc ^ c) & 0x0001 ) crc = ( crc >> 1 ) ^ P_16;
            else                      crc =   crc >> 1;

            c = c >> 1;
        }

        crc_tab16[i] = crc;
    }

    crc_tab16_init = TRUE;

}  /* init_crc16_tab */



    /*******************************************************************\
    *                                                                   *
    *   static void init_crckermit_tab( void );                         *
    *                                                                   *
    *   The function init_crckermit_tab() is used to fill the array     *
    *   for calculation of the CRC Kermit with values.                  *
    *                                                                   *
    \*******************************************************************/

static void init_crckermit_tab( void ) {

    int i, j;
    unsigned short crc, c;

    for (i=0; i<256; i++) {

        crc = 0;
        c   = (unsigned short) i;

        for (j=0; j<8; j++) {

            if ( (crc ^ c) & 0x0001 ) crc = ( crc >> 1 ) ^ P_KERMIT;
            else                      crc =   crc >> 1;

            c = c >> 1;
        }

        crc_tabkermit[i] = crc;
    }

    crc_tabkermit_init = TRUE;

}  /* init_crckermit_tab */



    /*******************************************************************\
    *                                                                   *
    *   static void init_crcdnp_tab( void );                            *
    *                                                                   *
    *   The function init_crcdnp_tab() is used  to  fill  the  array    *
    *   for calculation of the CRC-DNP with values.                     *
    *                                                                   *
    \*******************************************************************/

static void init_crcdnp_tab( void ) {

    int i, j;
    unsigned short crc, c;

    for (i=0; i<256; i++) {

        crc = 0;
        c   = (unsigned short) i;

        for (j=0; j<8; j++) {

            if ( (crc ^ c) & 0x0001 ) crc = ( crc >> 1 ) ^ P_DNP;
            else                      crc =   crc >> 1;

            c = c >> 1;
        }

        crc_tabdnp[i] = crc;
    }

    crc_tabdnp_init = TRUE;

}  /* init_crcdnp_tab */



    /*******************************************************************\
    *                                                                   *
    *   static void init_crc32_tab( void );                             *
    *                                                                   *
    *   The function init_crc32_tab() is used  to  fill  the  array     *
    *   for calculation of the CRC-32 with values.                      *
    *                                                                   *
    \*******************************************************************/

static void init_crc32_tab( void ) {

    int i, j;
    unsigned long crc;

    for (i=0; i<256; i++) {

        crc = (unsigned long) i;

        for (j=0; j<8; j++) {

            if ( crc & 0x00000001L ) crc = ( crc >> 1 ) ^ P_32;
            else                     crc =   crc >> 1;
        }

        crc_tab32[i] = crc;
    }

    crc_tab32_init = TRUE;

}  /* init_crc32_tab */



    /*******************************************************************\
    *                                                                   *
    *   static void init_crcccitt_tab( void );                          *
    *                                                                   *
    *   The function init_crcccitt_tab() is used to fill the  array     *
    *   for calculation of the CRC-CCITT with values.                   *
    *                                                                   *
    \*******************************************************************/

static void init_crcccitt_tab( void ) {

    int i, j;
    unsigned short crc, c;

    for (i=0; i<256; i++) {

        crc = 0;
        c   = ((unsigned short) i) << 8;

        for (j=0; j<8; j++) {

            if ( (crc ^ c) & 0x8000 ) crc = ( crc << 1 ) ^ P_CCITT;
            else                      crc =   crc << 1;

            c = c << 1;
        }

        crc_tabccitt[i] = crc;
    }

    crc_tabccitt_init = TRUE;

}  /* init_crcccitt_tab */

unsigned short update_crc_gdl90( unsigned short crc, char c ) {

    unsigned short tmp, short_c;

    short_c  = 0x00ff & (unsigned short) c;

    if ( ! crc_tabccitt_init ) init_crcccitt_tab();

    tmp = (crc >> 8) ;
    crc = crc_tabccitt[tmp] ^ (crc << 8) ^  short_c;

    return crc;

}  /* update_crc_gdl90 */

 /*
  *
  * Computes a 8-bit CRC
  *
  *   x^8 + x^2 + x + 1
  */

static const unsigned char crc8_table[256]
#if defined(ESP8266)
 PROGMEM
#endif
= {
    0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31,
    0x24, 0x23, 0x2a, 0x2d, 0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65,
    0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d, 0xe0, 0xe7, 0xee, 0xe9,
    0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
    0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85, 0xa8, 0xaf, 0xa6, 0xa1,
    0xb4, 0xb3, 0xba, 0xbd, 0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2,
    0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea, 0xb7, 0xb0, 0xb9, 0xbe,
    0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
    0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16,
    0x03, 0x04, 0x0d, 0x0a, 0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42,
    0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a, 0x89, 0x8e, 0x87, 0x80,
    0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
    0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6, 0xcf, 0xc8,
    0xdd, 0xda, 0xd3, 0xd4, 0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c,
    0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44, 0x19, 0x1e, 0x17, 0x10,
    0x05, 0x02, 0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
    0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f,
    0x6a, 0x6d, 0x64, 0x63, 0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b,
    0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13, 0xae, 0xa9, 0xa0, 0xa7,
    0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
    0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef,
    0xfa, 0xfd, 0xf4, 0xf3
};


void crc8(unsigned char *crc, unsigned char m)
     /*
      * For a byte array whose accumulated crc value is stored in *crc, computes
      * resultant crc obtained by appending m to the byte array
      */
{
#if defined(ESP8266)
  *crc = pgm_read_byte(&crc8_table[(*crc) ^ m]);
#else
  *crc = crc8_table[(*crc) ^ m];
#endif
}