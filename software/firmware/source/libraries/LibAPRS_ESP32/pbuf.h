/********************************************************************
 *  APRX -- 2nd generation APRS-i-gate with                         *
 *          minimal requirement of esoteric facilities or           *
 *          libraries of any kind beyond UNIX system libc.          *
 *                                                                  *
 * (c) Matti Aarnio - OH2MQK,  2007-2014                            *
 *                                                                  *
 ********************************************************************/
/*
 *	Some parts of this code are copied from:
 *
 *	aprsc
 *
 *	(c) Heikki Hannikainen, OH7LZB <hessu@hes.iki.fi>
 *
 *     This program is licensed under the BSD license, which can be found
 *     in the file LICENSE.
 *	
 */

/* Modified for  APRX by Matti Aarnio, OH2MQK
 * Altered name from  worker.h  to pbuf.h, and
 * dropped about 70% of worker.h stuff...
 */

#ifndef PBUF_H
#define PBUF_H

#if defined(ARDUINO) || defined(HACKRF_ONE)
#include <Arduino.h>
#endif /* ARDUINO */

/* minimum and maximum length of a callsign on APRS-IS */
#define CALLSIGNLEN_MIN 3
#define CALLSIGNLEN_MAX 9

/* packet length limiters and buffer sizes */
#define PACKETLEN_MIN 10	/* minimum length for a valid APRS-IS packet: "A1A>B1B:\r\n" */
#define PACKETLEN_MAX 512	/* maximum length for a valid APRS-IS packet (incl. CRLF) */

/*
 *  Packet length statistics:
 *
 *   <=  80:  about  25%
 *   <=  90:  about  36%
 *   <= 100:  about  73%
 *   <= 110:  about  89%
 *   <= 120:  about  94%
 *   <= 130:  about  97%
 *   <= 140:  about  98.7%
 *   <= 150:  about  99.4%
 */

#define PACKETLEN_MAX_SMALL  100 
#define PACKETLEN_MAX_MEDIUM 180 /* about 99.5% are smaller than this */
#define PACKETLEN_MAX_LARGE  PACKETLEN_MAX

/* number of pbuf_t structures to allocate at a time */
#define PBUF_ALLOCATE_BUNCH_SMALL  2000 /* grow to 2000 in production use */
#define PBUF_ALLOCATE_BUNCH_MEDIUM 2000 /* grow to 2000 in production use */
#define PBUF_ALLOCATE_BUNCH_LARGE    50 /* grow to 50 in production use */

/* a packet buffer */
/* Type flags -- some can happen in combinations: T_CWOP + T_WX / T_CWOP + T_POSITION ... */
#define T_POSITION   (1 << 0) // Packet is of position type
#define T_OBJECT     (1 << 1) // packet is an object
#define T_ITEM       (1 << 2) // packet is an item
#define T_MESSAGE    (1 << 3) // packet is a message
#define T_NWS        (1 << 4) // packet is a NWS message
#define T_WX         (1 << 5) // packet is WX data
#define T_TELEMETRY  (1 << 6) // packet is telemetry
#define T_QUERY      (1 << 7) // packet is a query
#define T_STATUS     (1 << 8) // packet is status 
#define T_USERDEF    (1 << 9) // packet is userdefined
#define T_CWOP       (1 << 10) // packet is recognized as CWOP
#define T_STATCAPA   (1 << 11) // packet is station capability response
#define T_THIRDPARTY (1 << 12)
#define T_ALL	     (1 << 15) // set on _all_ packets

#define F_DUPE    	(1 << 0) // Duplicate of a previously seen packet
#define F_HASPOS  	(1 << 1) // This packet has valid parsed position
#define F_HAS_TCPIP	(1 << 2) // There is a TCPIP* in the path
#define F_CSRSPD	(1 << 3)
#define F_ALT		(1 << 4)
#define F_PHG		(1 << 5)
#define F_RNG		(1 << 6)
#define F_UNIT		(1 << 7)
#define F_PARM		(1 << 8)
#define F_EQNS		(1 << 9)
#define F_BITS		(1 << 10)
#define F_TLM		(1 << 11)

#define W_WD	(1 << 0)
#define W_WS	(1 << 1)
#define W_WG	(1 << 2)
#define W_TEMP	(1 << 3)
#define W_R1H	(1 << 4)
#define W_R24H	(1 << 5)
#define W_RMN	(1 << 6)
#define W_HUM	(1 << 7)
#define W_BAR	(1 << 8)
#define W_PAR	(1 << 9)
#define W_UV	(1 << 10)

/// Weather report type.
typedef struct
{
	uint16_t flags;		/* bitmask: one or more of W_* */

	/// Wind gust in m/s.
	double wind_gust;
	/// Wind direction in degrees.
	unsigned int wind_dir;
	/// Wind speed in m/s.
	double wind_speed;

	/// Temperature in degrees Celcius.
	double temp;
	/// Indoor temperature in degrees Celcius.
	double temp_in;

	/// Rain from last 1 hour, in millimeters.
	double rain_1h;
	/// Rain from last day, in millimeters.
	double rain_24h;
	/// Rain since midnight, in millimeters.
	double rain_midnight;

	/// Relative humidity percentage.
	unsigned int humidity;
	/// Relative inside humidity percentage.
	unsigned int humidity_in;

	/// Air pressure in millibars.
	double pressure;
	/// Luminosity in watts per square meter.
	unsigned int luminosity;
	unsigned char uv;

	/// Show depth increasement from last day, in millimeters.
	//double* snow_24h;

	/// Software type indicator.
	//char* soft;
} fap_wx_report_t;



/// Telemetry report type.
typedef struct
{
	/// Id of report.
	unsigned int seq;
	float val[5];
	/*/// First value.
	double val1;
	/// Second value.
	double val2;
	/// Third value.
	double val3;
	/// Fourth value.
	double val4;
	/// Fifth value.
	double val5;*/

	/// Telemetry bits as ASCII 0s and 1s. Unknowns are marked with question marks.
	uint8_t bits;
	uint8_t bitsFlag;
} fap_telemetry_t;

typedef struct
{
	char val[5][10];
	///// First value.
	//char val1[10];
	///// Second value.
	//char val2[10];
	///// Third value.
	//char val3[10];
	///// Fourth value.
	//char val4[10];
	///// Fifth value.
	//char val5[10];

	/// Telemetry bits as ASCII 0s and 1s. Unknowns are marked with question marks.
	//char bits[10];
} fap_telemetry_unit;

typedef struct
{
	float val[15];
} fap_telemetry_eqns;


typedef struct
{
	char val[5][10];
	///// First value.
	//char val1[10];
	///// Second value.
	//char val2[10];
	///// Third value.
	//char val3[10];
	///// Fourth value.
	//char val4[10];
	///// Fifth value.
	//char val5[10];

	/// Telemetry bits as ASCII 0s and 1s. Unknowns are marked with question marks.
	//char bits[10];
} fap_telemetry_parm;

typedef struct {
	const char* body;          /* message body */
	const char* msgid;

	int body_len;
	int msgid_len;
	int is_ack;
	int is_rej;
} aprs_message_t;

struct pbuf_t {
	//struct pbuf_t *next;

	int16_t	 is_aprs;	// If not, then just digipeated frame..
	int16_t	 digi_like_aprs;
	int16_t  source_if_group;

	int16_t  refcount;

	int16_t	 reqcount;      // How many digipeat hops are requested?
	int16_t	 donecount;	// How many digipeat hops are already done?

	time_t   t;		/* when the packet was received */
	uint32_t seqnum;	/* ever increasing counter, dupecheck sets */
	uint16_t packettype;	/* bitmask: one or more of T_* */
	uint16_t flags;		/* bitmask: one or more of F_* */
	uint8_t srcname_len;	/* parsed length of source (object, item, srcall) name 3..9 */
	uint8_t dstcall_len;	/* parsed length of destination callsign *including* SSID */
	uint8_t dstname_len;   /* parsed length of message destination including SSID */
	uint8_t entrycall_len;
	
	int packet_len;		/* the actual length of the TNC2 packet */
	int buf_len;		/* the length of this buffer */

	char* comment;
	/// Length of comment.
	unsigned int comment_len;
	
	const char *srccall_end;   /* source callsign with SSID */
	const char *dstcall_end_or_ssid;   /* end of dest callsign (without SSID) */
	const char *dstcall_end;   /* end of dest callsign with SSID */
	const char *qconst_start;  /* "qAX,incomingSSID:"	-- for q and e filters  */
	const char *info_start;    /* pointer to start of info field */
	const char *srcname;       /* source's name (either srccall or object/item name) */
	const char *dstname;       /* message destination callsign */
	
	float lat;	/* if the packet is PT_POSITION, latitude and longitude go here */
	float lng;	/* .. in RADIAN */
	float cos_lat;	/* cache of COS of LATitude for radial distance filter    */

	/// Position resolution in meters.
	double pos_resolution;
	/// Position ambiguity, number of digits.
	unsigned int pos_ambiguity;

	/// Zero if GPS has no fix, one if it has.
	unsigned char gps_fix_status;
	/// Radio range of the station in km.
	unsigned short radio_range;
	/// TX power, antenna height, antenna gain and possibly beacon rate.
	char phg[6];

		/// Altitude in meters.
	double altitude;
	/// Course in degrees, zero is unknown and 360 is north.
	unsigned short course;
	/// Land speed in km/h.
	double speed;

	/// Weather report.
	fap_wx_report_t wx_report;

	/// Telemetry report.
	fap_telemetry_t telemetry;
	fap_telemetry_parm tlm_parm;
	fap_telemetry_unit tlm_unit;
	fap_telemetry_eqns tlm_eqns;

	aprs_message_t msg;

	char symbol[3]; /* 2(+1) chars of symbol, if any, NUL for not found */

	uint8_t *ax25addr;	// Start of AX.25 address
	int      ax25addrlen;	// length of AX.25 address

	uint8_t *ax25data;	// Start of AX.25 data after addresses
	int      ax25datalen;	// length of that data

	char data[300];
};

/* global packet buffer */
extern struct pbuf_t  *pbuf_global;
extern struct pbuf_t  *pbuf_global_last;
extern struct pbuf_t **pbuf_global_prevp;
extern struct pbuf_t  *pbuf_global_dupe;
extern struct pbuf_t  *pbuf_global_dupe_last;
extern struct pbuf_t **pbuf_global_dupe_prevp;

#endif
