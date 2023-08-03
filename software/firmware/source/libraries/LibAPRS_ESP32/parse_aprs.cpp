#if defined(ARDUINO) || defined(HACKRF_ONE)
#include <Arduino.h>
#endif /* ARDUINO */

#if defined(RASPBERRY_PI)
#include <raspi/raspi.h>
#endif /* RASPBERRY_PI */

#include <stdio.h>
#include <math.h>
#include <parse_aprs.h>
#include <string.h>
#include <pbuf.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define debug 0

//#define DEBUG_LOG(...) Serial.print(__VA_ARGS__)     //DPRINT is a macro, debug print
#define DEBUG_LOG
//#define DEBUG_LOG Serial.printf

/// Max amount of path elements.
#define MAX_DIGIS 8

/// Nautical miles per hour to kilometers per hour.
#define KNOT_TO_KMH 1.852

/// Miles per hour to kilometers per hour.
#define MPH_TO_KMH 1.609344

/// Kilometers per hour to meters per second.
#define KMH_TO_MS 10.0/36.0

/// Miles per hout to meters per second.
#define MPH_TO_MS (MPH_TO_KMH * KMH_TO_MS)

/// Feets to meters.
#define FT_TO_M 0.3048

/// Fahrenheit to celcius degrees.
#define FAHRENHEIT_TO_CELCIUS(x) ((x-32.0)/1.8)

/// Hundredths of an inch to millimeters
#define HINCH_TO_MM 0.254

/// The magic constant.
#ifdef PI
#undef PI
#define PI 3.14159265
#endif
/// Degrees to radians.
#define DEG2RAD(x) (x/360*2*PI)
/// Radians to degrees.
#define RAD2DEG(x) (x*(180/PI))

/*
 *	Check if the given character is a valid symbol table identifier
 *	or an overlay character. The set is different for compressed
 *	and uncompressed packets - the former has the overlaid number (0-9)
 *	replaced with n-j.
 */

double ParseAPRS::direction(double lon0, double lat0, double lon1, double lat1)
{
	double direction;

	/* Convert degrees into radians. */
	lon0 = DEG2RAD(lon0);
	lat0 = DEG2RAD(lat0);
	lon1 = DEG2RAD(lon1);
	lat1 = DEG2RAD(lat1);

	/* Direction from Aviation Formulary V1.42 by Ed Williams by way of
	 * http://mathforum.org/library/drmath/view/55417.html */
	direction = atan2(sin(lon1 - lon0) * cos(lat1), cos(lat0) * sin(lat1) - sin(lat0) * cos(lat1) * cos(lon1 - lon0));
	if (direction < 0)
	{
		/* Make direction positive. */
		direction += 2 * PI;
	}

	return RAD2DEG(direction);
}

double ParseAPRS::distance(double lon0, double lat0, double lon1, double lat1)
{
	double dlon;
	double dlat;
	double a, c;
	/* Convert degrees into radians. */
	lon0 = DEG2RAD(lon0);
	lat0 = DEG2RAD(lat0);
	lon1 = DEG2RAD(lon1);
	lat1 = DEG2RAD(lat1);

	/* Use the haversine formula for distance calculation
	 * http://mathforum.org/library/drmath/view/51879.html */
	dlon = lon1 - lon0;
	dlat = lat1 - lat0;
	a = pow(sin(dlat / 2), 2) + cos(lat0) * cos(lat1) * pow(sin(dlon / 2), 2);
	c = 2 * atan2(sqrt(a), sqrt(1 - a));

	return c * 6366.71; /* in kilometers */
}

float ParseAPRS::filter_lat2rad(float lat)
{
	return (lat * (M_PI / 180.0));
}

float ParseAPRS::filter_lon2rad(float lon)
{
	return (lon * (M_PI / 180.0));
}

int ParseAPRS::is_number(char const* input)
{
	int i;

	if (!input) return 0;

	for (i = 0; i < strlen(input); ++i)
	{
		if (!isdigit(input[i]) || (i == 0 && (input[i] == '-' || input[i] == '+'))) return 0;
	}

	return 1;
}

int ParseAPRS::valid_sym_table_compressed(char c)
{
	return (c == '/' || c == '\\' || (c >= 0x41 && c <= 0x5A)
		    || (c >= 0x61 && c <= 0x6A)); /* [\/\\A-Za-j] */
}

int ParseAPRS::valid_sym_table_uncompressed(char c)
{
	return (c == '/' || c == '\\' || (c >= 0x41 && c <= 0x5A)
		    || (c >= 0x30 && c <= 0x39)); /* [\/\\A-Z0-9] */
}

/*
 *	Fill the pbuf_t structure with a parsed position and
 *	symbol table & code. Also does range checking for lat/lng
 *	and pre-calculates cosf(lat) for range filters.
 */

int ParseAPRS::pbuf_fill_pos(struct pbuf_t *pb, const float lat, const float lng, const char sym_table, const char sym_code)
{
	int bad = 0;
	/* symbol table and code */
	pb->symbol[0] = sym_table;
	pb->symbol[1] = sym_code;
	pb->symbol[2] = 0;
	
	/* Is it perhaps a weather report ? */
	//if (sym_code == '_' && (sym_table == '/' || sym_table == '\\')) 
	//	pb->packettype |= T_WX;
	//if (sym_code == '@' && (sym_table == '/' || sym_table == '\\')) 
	//	pb->packettype |= T_WX;	/* Hurricane */

	bad |= (lat < -89.9 && -0.0001 <= lng && lng <= 0.0001);
	bad |= (lat >  89.9 && -0.0001 <= lng && lng <= 0.0001);

	if (-0.0001 <= lat && lat <= 0.0001) {
	  bad |= ( -0.0001 <= lng && lng <= 0.0001);
	  bad |= ( -90.01  <= lng && lng <= -89.99);
	  bad |= (  89.99  <= lng && lng <=  90.01);
	}


	if (bad || lat < -90.0 || lat > 90.0 || lng < -180.0 || lng > 180.0) {
		if (debug)
			printf("\tposition out of range: lat %.3f lng %.3f", lat, lng);

		return 0; /* out of range */
	}
	
	if (debug)
		printf("\tposition ok: lat %.3f lng %.3f", lat, lng);

	/* Pre-calculations for A/R/F/M-filter tests */
	//pb->lat     = filter_lat2rad(lat);  /* deg-to-radians */
	//pb->cos_lat = cosf(pb->lat);        /* used in range filters */
	//pb->lng     = filter_lon2rad(lng);  /* deg-to-radians */
	pb->lat = lat;
	pb->lng = lng;
	
	pb->flags |= F_HASPOS;	/* the packet has positional data */

	return 1;
}

/*
 *	Parse symbol from destination callsign
 */
int ParseAPRS::get_symbol_from_dstcall_twochar(const char c1, const char c2, char *sym_table, char *sym_code)
{
	//hlog(LOG_DEBUG, "\ttwochar %c %c", c1, c2);
	if (c1 == 'B') {
		if (c2 >= 'B' && c2 <= 'P') {
			*sym_table = '/';
			*sym_code = c2 - 'B' + '!';
			return 1;
		}
		return 0;
	}
	
	if (c1 == 'P') {
		if (c2 >= '0' && c2 <= '9') {
			*sym_table = '/';
			*sym_code = c2;
			return 1;
		}
		if (c2 >= 'A' && c2 <= 'Z') {
			*sym_table = '/';
			*sym_code = c2;
			return 1;
		}
		return 0;
	}
	
	if (c1 == 'M') {
		if (c2 >= 'R' && c2 <= 'X') {
			*sym_table = '/';
			*sym_code = c2 - 'R' + ':';
			return 1;
		}
		return 0;
	}
	
	if (c1 == 'H') {
		if (c2 >= 'S' && c2 <= 'X') {
			*sym_table = '/';
			*sym_code = c2 - 'S' + '[';
			return 1;
		}
		return 0;
	}
	
	if (c1 == 'L') {
		if (c2 >= 'A' && c2 <= 'Z') {
			*sym_table = '/';
			*sym_code = c2 - 'A' + 'a';
			return 1;
		}
		return 0;
	}
	
	if (c1 == 'J') {
		if (c2 >= '1' && c2 <= '4') {
			*sym_table = '/';
			*sym_code = c2 - '1' + '{';
			return 1;
		}
		return 0;
	}
	
	if (c1 == 'O') {
		if (c2 >= 'B' && c2 <= 'P') {
			*sym_table = '\\';
			*sym_code = c2 - 'B' + '!';
			return 1;
		}
		return 0;
	}
	
	if (c1 == 'A') {
		if (c2 >= '0' && c2 <= '9') {
			*sym_table = '\\';
			*sym_code = c2;
			return 1;
		}
		if (c2 >= 'A' && c2 <= 'Z') {
			*sym_table = '\\';
			*sym_code = c2;
			return 1;
		}
		return 0;
	}
	
	if (c1 == 'N') {
		if (c2 >= 'R' && c2 <= 'X') {
			*sym_table = '\\';
			*sym_code = c2 - 'R' + ':';
			return 1;
		}
		return 0;
	}
	
	if (c1 == 'D') {
		if (c2 >= 'S' && c2 <= 'X') {
			*sym_table = '\\';
			*sym_code = c2 - 'S' + '[';
			return 1;
		}
		return 0;
	}
	
	if (c1 == 'S') {
		if (c2 >= 'A' && c2 <= 'Z') {
			*sym_table = '\\';
			*sym_code = c2 - 'A' + 'a';
			return 1;
		}
		return 0;
	}
	
	if (c1 == 'Q') {
		if (c2 >= '1' && c2 <= '4') {
			*sym_table = '\\';
			*sym_code = c2 - '1' + '{';
			return 1;
		}
		return 0;
	}
	
	return 0;
}

int ParseAPRS::get_symbol_from_dstcall(struct pbuf_t *pb, char *sym_table, char *sym_code)
{
	const char *d_start;
	char type;
	char overlay;
	int sublength;
	int numberid;
	
	/* check that the destination call exists and is of the right size for symbol */
	d_start = pb->srccall_end+1;
	if (pb->dstcall_end_or_ssid - d_start < 5)
		return 0; /* too short */
	
	/* length of the parsed string */
	sublength = pb->dstcall_end_or_ssid - d_start - 3;
	if (sublength > 3)
		sublength = 3;
	
#ifdef DEBUG_PARSE_APRS
	if (debug)
	  printf("\tget_symbol_from_dstcall: %.*s (%d)", (int)(pb->dstcall_end_or_ssid - d_start), d_start, sublength);
#endif
	
	if (strncmp(d_start, "GPS", 3) != 0 && strncmp(d_start, "SPC", 3) != 0 && strncmp(d_start, "SYM", 3) != 0)
		return 0;
	
	// hlog(LOG_DEBUG, "\ttesting %c %c %c", d_start[3], d_start[4], d_start[5]);
	if (!isalnum(d_start[3]) || !isalnum(d_start[4]))
		return 0;
	
	if (sublength == 3 && !isalnum(d_start[5]))
		return 0;
	
	type = d_start[3];
	
	if (sublength == 3) {
		if (type == 'C' || type == 'E') {
			if (!isdigit(d_start[4]))
				return 0;
			if (!isdigit(d_start[5]))
				return 0;
			numberid = (d_start[4] - 48) * 10 + (d_start[5] - 48);
			
			*sym_code = numberid + 32;
			if (type == 'C')
				*sym_table = '/';
			else
				*sym_table = '\\';
		
#ifdef DEBUG_PARSE_APRS
			if (debug)
			  printf("\tnumeric symbol id in dstcall: %.*s: table %c code %c",
				(int)(pb->dstcall_end_or_ssid - d_start - 3), d_start + 3, *sym_table, *sym_code);
#endif
			return 1;
		} else {
			/* secondary symbol table, with overlay
			 * Check first that we really are in the secondary symbol table
			 */
			overlay = d_start[5];
			if ((type == 'O' || type == 'A' || type == 'N' ||
				type == 'D' || type == 'S' || type == 'Q')
				&& isalnum(overlay)) {
				return get_symbol_from_dstcall_twochar(d_start[3], d_start[4], sym_table, sym_code);
			}
			return 0;
		}
	} else {
		// primary or secondary table, no overlay
		return get_symbol_from_dstcall_twochar(d_start[3], d_start[4], sym_table, sym_code);
	}
	
	return 0;
}


/*
 *	Parse NMEA position packets.
 */

int ParseAPRS::parse_aprs_nmea(struct pbuf_t *pb, const char *body, const char *body_end)
{
	float lat, lng;
	const char *latp, *lngp;
	int i, la, lo;
	char lac, loc;
	char sym_table, sym_code;
	
	if (memcmp(body,"ULT",3) == 0) {
		/* Ah..  "$ULT..." - that is, Ultimeter 2000 weather instrument */
		pb->packettype |= T_WX;
		return 1;
	}
	
	lat  = lng  = 0.0;
	latp = lngp = NULL;
	
	/* NMEA sentences to understand:
	   $GPGGA  Global Positioning System Fix Data
	   $GPGLL  Geographic Position, Latitude/Longitude Data
	   $GPRMC  Remommended Minimum Specific GPS/Transit Data
	   $GPWPT  Way Point Location ?? (bug in APRS specs ?)
	   $GPWPL  Waypoint Load (not in APRS specs, but in NMEA specs)
	   $PNTS   Seen on APRS-IS, private sentense based on NMEA..
	   $xxTLL  Not seen on radio network, usually $RATLL - Target positions
	           reported by RAdar.
	 */
	 
	if (memcmp(body, "GPGGA,", 6) == 0) {
		/* GPGGA,175059,3347.4969,N,11805.7319,W,2,12,1.0,6.8,M,-32.1,M,,*7D
		//   v=1, looks fine
		// GPGGA,000000,5132.038,N,11310.221,W,1,09,0.8,940.0,M,-17.7,,
		//   v=1, timestamp odd, coords look fine
		// GPGGA,,,,,,0,00,,,,,,,*66
		//   v=0, invalid
		// GPGGA,121230,4518.7931,N,07322.3202,W,2,08,1.0,40.0,M,-32.4,M,,*46
		//   v=2, looks valid ?
		// GPGGA,193115.00,3302.50182,N,11651.22581,W,1,08,01.6,00465.90,M,-32.891,M,,*5F
		// $GPGGA,hhmmss.dd,xxmm.dddd,<N|S>,yyymm.dddd,<E|W>,v,
		//        ss,d.d,h.h,M,g.g,M,a.a,xxxx*hh<CR><LF>
		*/
		
		latp = body+6; // over the keyword
		while (latp < body_end && *latp != ',')
			latp++; // scan over the timestamp
		if (*latp == ',')
			latp++; // .. and into latitude.
		lngp = latp;
		while (lngp < body_end && *lngp != ',')
			lngp++;
		if (*lngp == ',')
			lngp++;
		if (*lngp != ',')
			lngp++;
		if (*lngp == ',')
			lngp++;
			
		/* latp, and lngp  point to start of latitude and longitude substrings
		// respectively.
		*/
	
	} else if (memcmp(body, "GPGLL,", 6) == 0) {
		/* $GPGLL,xxmm.dddd,<N|S>,yyymm.dddd,<E|W>,hhmmss.dd,S,M*hh<CR><LF>  */
		latp = body+6; // over the keyword
		lngp = latp;
		while (lngp < body_end && *lngp != ',') // over latitude
			lngp++;
		if (*lngp == ',')
			lngp++; // and lat designator
		if (*lngp != ',')
			lngp++; // and lat designator
		if (*lngp == ',')
			lngp++;
		/* latp, and lngp  point to start of latitude and longitude substrings
		// respectively
		*/
	} else if (memcmp(body, "GPRMC,", 6) == 0) {
		/* $GPRMC,hhmmss.dd,S,xxmm.dddd,<N|S>,yyymm.dddd,<E|W>,s.s,h.h,ddmmyy,d.d, <E|W>,M*hh<CR><LF>
		// ,S, = Status:  'A' = Valid, 'V' = Invalid
		// 
		// GPRMC,175050,A,4117.8935,N,10535.0871,W,0.0,324.3,100208,10.0,E,A*3B
		// GPRMC,000000,V,0000.0000,0,00000.0000,0,000,000,000000,,*01/It wasn't me :)
		//    invalid..
		// GPRMC,000043,V,4411.7761,N,07927.0448,W,0.000,0.0,290697,10.7,W*57
		// GPRMC,003803,A,3347.1727,N,11812.7184,W,000.0,000.0,140208,013.7,E*67
		// GPRMC,050058,A,4609.1143,N,12258.8184,W,0.000,0.0,100208,18.0,E*5B
		*/
		
		latp = body+6; // over the keyword
		while (latp < body_end && *latp != ',')
			latp++; // scan over the timestamp
		if (*latp == ',')
			latp++; // .. and into VALIDITY
		if (*latp != 'A' && *latp != 'V')
			return 0; // INVALID !
		if (*latp != ',')
			latp++;
		if (*latp == ',')
			latp++;
		
		/* now it points to latitude substring */
		lngp = latp;
		while (lngp < body_end && *lngp != ',')
			lngp++;
		
		if (*lngp == ',')
			lngp++;
		if (*lngp != ',')
			lngp++;
		if (*lngp == ',')
			lngp++;
		
		/* latp, and lngp  point to start of latitude and longitude substrings
		// respectively.
		*/
		
	} else if (memcmp(body, "GPWPL,", 6) == 0) {
		/* $GPWPL,4610.586,N,00607.754,E,4*70
		// $GPWPL,4610.452,N,00607.759,E,5*74
		*/
		latp = body+6;
		
	} else if (memcmp(body, "PNTS,1,", 7) == 0) { /* PNTS version 1 */
		/* $PNTS,1,0,11,01,2002,231932,3539.687,N,13944.480,E,0,000,5,Roppongi UID RELAY,000,1*35
		// $PNTS,1,0,14,01,2007,131449,3535.182,N,13941.200,E,0,0.0,6,Oota-Ku KissUIDigi,000,1*1D
		// $PNTS,1,0,17,02,2008,120824,3117.165,N,13036.481,E,49,059,1,Kagoshima,000,1*71
		// $PNTS,1,0,17,02,2008,120948,3504.283,N,13657.933,E,00,000.0,6,,000,1*36
		// 
		// From Alinco EJ-41U Terminal Node Controller manual:
		// 
		// 5-4-7 $PNTS
		// This is a private-sentence based on NMEA-0183.  The data contains date,
		// time, latitude, longitude, moving speed, direction, altitude plus a short
		// message, group codes, and icon numbers. The EJ-41U does not analyze this
		// format but can re-structure it.
		// The data contains the following information:
		//  l $PNTS Starts the $PNTS sentence
		//  l version
		//  l the registered information. [0]=normal geographical location data.
		//    This is the only data EJ-41U can re-structure. [s]=Initial position
		//    for the course setting [E]=ending position for the course setting
		//    [1]=the course data between initial and ending [P]=the check point
		//    registration [A]=check data when the automatic position transmission
		//    is set OFF [R]=check data when the course data or check point data is
		//    received.
		//  l dd,mm,yyyy,hhmmss: Date and time indication.
		//  l Latitude in DMD followed by N or S
		//  l Longitude in DMD followed by E or W
		//  l Direction: Shown with the number 360 degrees divided by 64.
		//    00 stands for true north, 16 for east. Speed in Km/h
		//  l One of 15 characters [0] to [9], [A] to [E].
		//    NTSMRK command determines this character when EJ-41U is used.
		//  l A short message up to 20 bites. Use NTSMSG command to determine this message.
		//  l A group code: 3 letters with a combination of [0] to [9], [A] to [Z].
		//    Use NTSGRP command to determine.
		//  l Status: [1] for usable information, [0] for non-usable information.
		//  l *hh<CR><LF> the check-sum and end of PNTS sentence.
		*/

		if (body+55 > body_end) {
			DEBUG_LOG("body too short");
			return 0; /* Too short.. */
		}
		latp = body+7; /* Over the keyword */
		/* Accept any registered information code */
		if (*latp++ == ',') return 0;
		if (*latp++ != ',') return 0;
		/* Scan over date+time info */
		while (*latp != ',' && latp <= body_end) ++latp;
		if (*latp == ',') ++latp;
		while (*latp != ',' && latp <= body_end) ++latp;
		if (*latp == ',') ++latp;
		while (*latp != ',' && latp <= body_end) ++latp;
		if (*latp == ',') ++latp;
		while (*latp != ',' && latp <= body_end) ++latp;
		if (*latp == ',') ++latp;
		/* now it points to latitude substring */
		lngp = latp;
		while (lngp < body_end && *lngp != ',')
			lngp++;
		
		if (*lngp == ',')
			lngp++;
		if (*lngp != ',')
			lngp++;
		if (*lngp == ',')
			lngp++;
		
		/* latp, and lngp  point to start of latitude and longitude substrings
		// respectively.
		*/
#if 1
	} else if (memcmp(body, "GPGSA,", 6) == 0 ||
		   memcmp(body, "GPVTG,", 6) == 0 ||
		   memcmp(body, "GPGSV,", 6) == 0) {
		/* Recognized but ignored */
		return 1;
#endif
	}
	
	if (!latp || !lngp) {
		if (debug)
			fprintf(stderr, "Unknown NMEA: '%.11s' %.*s", pb->data, (int)(body_end - body), body);
		return 0; /* Well..  Not NMEA frame */
	}

	// hlog(LOG_DEBUG, "NMEA parsing: %.*s", (int)(body_end - body), body);
	// hlog(LOG_DEBUG, "     lat=%.10s   lng=%.10s", latp, lngp);

	i = sscanf(latp, "%2d%f,%c,", &la, &lat, &lac);
	if (i != 3)
		return 0; // parse failure
	
	i = sscanf(lngp, "%3d%f,%c,", &lo, &lng, &loc);
	if (i != 3)
		return 0; // parse failure
	
	if (lac != 'N' && lac != 'S' && lac != 'n' && lac != 's')
		return 0; // bad indicator value
	if (loc != 'E' && loc != 'W' && loc != 'e' && loc != 'w')
		return 0; // bad indicator value
		
	// hlog(LOG_DEBUG, "   lat: %c %2d %7.4f   lng: %c %2d %7.4f",
	//                 lac, la, lat, loc, lo, lng);

	lat = (float)la + lat/60.0;
	lng = (float)lo + lng/60.0;
	
	if (lac == 'S' || lac == 's')
		lat = -lat;
	if (loc == 'W' || loc == 'w')
		lng = -lng;
	
	pb->packettype |= T_POSITION;
	
	// Parse symbol from destination callsign
	get_symbol_from_dstcall(pb, &sym_table, &sym_code);
#ifdef DEBUG_PARSE_APRS
	if (debug) {
	  printf("\tget_symbol_from_dstcall: %.*s => %c%c",
		 (int)(pb->dstcall_end_or_ssid - pb->srccall_end-1), pb->srccall_end+1, sym_table, sym_code);
	}
#endif

	return pbuf_fill_pos(pb, lat, lng, sym_table, sym_code);
}

int ParseAPRS::parse_aprs_telem(struct pbuf_t *pb, const char *body, const char *body_end)
{
	char* p;
	int i = 0;
	char* token;
	//const char s[2] = ",";
	// float lat = 0.0, lng = 0.0;
	if (memcmp(body, "UNIT.", 5) == 0) {
		p = (char*)body + 5;
		pb->flags |= F_UNIT;
		/* get the first token */
		token = strtok(p, ",");
		/* walk through other tokens */
		i = 0;
		while (token != NULL) {
			//printf(" %s\n", token);
			strcpy(pb->tlm_unit.val[i++], token);
			if (i >= 5) break;
			token = strtok(NULL, ",");
		}
	}
	else if (memcmp(body, "PARM.", 5) == 0) {
		p = (char*)body + 5;
		pb->flags |= F_PARM;
		/* get the first token */
		token = strtok(p, ",");
		/* walk through other tokens */
		i = 0;
		while (token != NULL) {
			//printf(" %s\n", token);
			strcpy(pb->tlm_parm.val[i++], token);
			if (i >= 5) break;
			token = strtok(NULL, ",");
		}
	}
	else if (memcmp(body, "EQNS.", 5) == 0) {
		p = (char*)body + 5;
		pb->flags |= F_EQNS;
		/* get the first token */
		token = strtok(p, ",");
		/* walk through other tokens */
		i = 0;
		while (token != NULL) {
			//printf(" %s\n", token);
			pb->tlm_eqns.val[i] = atof(token);
			if (++i >= 15) break;
			token = strtok(NULL, ",");
		}
	}
	else if (memcmp(body, "BITS.", 5) == 0) {
		p = (char*)body + 5;
		pb->flags |= F_BITS;
		//pb->telemetry.bits=
		//strncpy(pb->telemetry.bits, p, 8);
		uint8_t value = 0;
		char bits[8];
		memcpy(&bits[0], p, 8);
		for (int i = 0; i < 8; i++)  // for every character in the string  strlen(s) returns the length of a char array
		{
			//value *= 2; // double the result so far
			value <<= 1;
			if (bits[i] == '1') value|=0x01;  //add 1 if needed
		}
		pb->telemetry.bitsFlag = value;
	}
	else if (*body == '#') {
		p = (char*)body + 1;
		pb->flags |= F_TLM;
		/* get the first token */
		token = strtok(p, ",");
		/* walk through other tokens */
		i = 0;
		while (token != NULL) {
			//printf(" %s\n", token);
			if (i == 0) {
				pb->telemetry.seq = atoi(token);
			}
			else if (i == 6) {
				uint8_t value = 0;
				char bits[8];
				memcpy(&bits[0], token, 8);
				for (int i = 0; i < 8; i++)  // for every character in the string  strlen(s) returns the length of a char array
				{
					//value *= 2; // double the result so far
					value <<= 1;
					if (bits[i] == '1') value |= 0x01;  //add 1 if needed
				}
				pb->telemetry.bits = value;
				//strncpy(pb->telemetry.bits, token, 8);
			}
			else {
				pb->telemetry.val[i-1] = atof(token);
			}
			if (++i >= 7) break;
			token = strtok(NULL, ",");
		}
	}


	//DEBUG_LOG("parse_aprs_telem");

	//pbuf_fill_pos(pb, lat, lng, 0, 0);
	return 1; // okay
}

/*
 *	Parse a MIC-E position packet
 *
 *	APRS PROTOCOL REFERENCE 1.0.1 Chapter 10, page 42 (52 in PDF)
 *
 */

int ParseAPRS::parse_aprs_mice(struct pbuf_t *pb, const unsigned char *body, const unsigned char *body_end)
{
	float lat = 0.0, lng = 0.0;
	unsigned int lat_deg = 0, lat_min = 0, lat_min_frag = 0, lng_deg = 0, lng_min = 0, lng_min_frag = 0;
	const char *d_start;
	char dstcall[7];
	char *p;
	char sym_table, sym_code;
	int posambiguity = 0;
	int i;
	double speed, course_speed, course_speed_tmp, course;
	
        DEBUG_LOG("parse_aprs_mice: %.*s", pb->packet_len-2, pb->data);
	
	/* check packet length */
	if (body_end - body < 8)
		return 0;
	DEBUG_LOG("-----------------");
	DEBUG_LOG(pb->srccall_end);
	DEBUG_LOG(pb->dstcall_end_or_ssid);
	/* check that the destination call exists and is of the right size for mic-e */
	d_start = pb->srccall_end+1;
	if (pb->dstcall_end_or_ssid - d_start != 6) {
		DEBUG_LOG(".. bad destcall length! ");
		return 0; /* eh...? */
	}
	
	/* validate destination call:
	 * A-K characters are not used in the last 3 characters
	 * and MNO are never used
	 */
 	//if (debug)printf(" destcall='%6.6s'",d_start);
	for (i = 0; i < 3; i++)
		if (!((d_start[i] >= '0' && d_start[i] <= '9')
			|| (d_start[i] >= 'A' && d_start[i] <= 'L')
			|| (d_start[i] >= 'P' && d_start[i] <= 'Z'))) {
			DEBUG_LOG(".. bad destcall characters in posits 1..3");
			return 0;
		}
	
	for (i = 3; i < 6; i++)
		if (!((d_start[i] >= '0' && d_start[i] <= '9')
			|| (d_start[i] == 'L')
			|| (d_start[i] >= 'P' && d_start[i] <= 'Z'))) {
			DEBUG_LOG(".. bad destcall characters in posits 4..6");
			return 0;
		}
	
	DEBUG_LOG("\tpassed dstcall format check");
	
	/* validate information field (longitude, course, speed and
	 * symbol table and code are checked). Not bullet proof..
	 *
	 *   0          1          23            4          5          6              7
	 * /^[\x26-\x7f][\x26-\x61][\x1c-\x7f]{2}[\x1c-\x7d][\x1c-\x7f][\x21-\x7b\x7d][\/\\A-Z0-9]/
	 */
	if (body[0] < 0x26 || body[0] > 0x7f) {
		DEBUG_LOG("..bad infofield column 1");
		return 0;
	}
	if (body[1] < 0x26 || body[1] > 0x61) {
		DEBUG_LOG("..bad infofield column 2");
		return 0;
	}
	if (body[2] < 0x1c || body[2] > 0x7f) {
		DEBUG_LOG("..bad infofield column 3");
		return 0;
	}
	if (body[3] < 0x1c || body[3] > 0x7f) {
		DEBUG_LOG("..bad infofield column 4");
		return 0;
	}
	if (body[4] < 0x1c || body[4] > 0x7d) {
		DEBUG_LOG("..bad infofield column 5");
		return 0;
	}
	if (body[5] < 0x1c || body[5] > 0x7f) {
		DEBUG_LOG("..bad infofield column 6");
		return 0;
	}
	if ((body[6] < 0x21 || body[6] > 0x7b)
		&& body[6] != 0x7d) {
		DEBUG_LOG("..bad infofield column 7");
		return 0;
	}
	if (!valid_sym_table_uncompressed(body[7])) {
		DEBUG_LOG("..bad symbol table entry on column 8");
		return 0;
	}
	
	DEBUG_LOG("\tpassed info format check");
	
	/* make a local copy, we're going to modify it */
	strncpy(dstcall, d_start, 6);
	dstcall[6] = 0;
	
	/* First do the destination callsign
	 * (latitude, message bits, N/S and W/E indicators and long. offset)
	 *
	 * Translate the characters to get the latitude
	 */
	 
	//fprintf(stderr, "\tuntranslated dstcall: %s\n", dstcall);
	for (p = dstcall; *p; p++) {
		if (*p >= 'A' && *p <= 'J')
			*p -= 'A' - '0';
		else if (*p >= 'P' && *p <= 'Y')
			*p -= 'P' - '0';
		else if (*p == 'K' || *p == 'L' || *p == 'Z')
			*p = '_';
	}
	//fprintf(stderr, "\ttranslated dstcall: %s\n", dstcall);
	
	// position ambiquity is going to get ignored now,
	// it's not needed in this application.

	if (dstcall[5] == '_') { dstcall[5] = '5'; posambiguity = 1; }
	if (dstcall[4] == '_') { dstcall[4] = '5'; posambiguity = 2; }
	if (dstcall[3] == '_') { dstcall[3] = '5'; posambiguity = 3; }
	if (dstcall[2] == '_') { dstcall[2] = '3'; posambiguity = 4; }
	if (dstcall[1] == '_' || dstcall[0] == '_') {
		DEBUG_LOG("..bad pos-ambiguity on destcall");
		return 0;
	} // cannot use posamb here
	
	// convert to degrees, minutes and decimal degrees,
	//  and then to a float lat

	if (sscanf(dstcall, "%2u%2u%2u",
	    &lat_deg, &lat_min, &lat_min_frag) != 3) {
		DEBUG_LOG("\tsscanf failed");
		return 0;
	}
	lat = (float)lat_deg + (float)lat_min / 60.0 + (float)lat_min_frag / 6000.0;
	
	// check the north/south direction and correct the latitude if necessary
	if (d_start[3] <= 0x4c)
		lat = 0 - lat;
	
	/* Decode the longitude, the first three bytes of the body
	 * after the data type indicator. First longitude degrees,
	 * remember the longitude offset.
	 */
	lng_deg = body[0] - 28;
	if (d_start[4] >= 0x50)
		lng_deg += 100;
	if (lng_deg >= 180 && lng_deg <= 189)
		lng_deg -= 80;
	else if (lng_deg >= 190 && lng_deg <= 199)
		lng_deg -= 190;

	/* Decode the longitude minutes */
	lng_min = body[1] - 28;
	if (lng_min >= 60)
		lng_min -= 60;
		
	/* ... and minute decimals */
	lng_min_frag = body[2] - 28;
	
	/* apply position ambiguity to longitude */
	switch (posambiguity) {
	case 0:
		/* use everything */
		lng = (float)lng_deg + (float)lng_min / 60.0
			+ (float)lng_min_frag / 6000.0;
		break;
	case 1:
		/* ignore last number of lng_min_frag */
		lng = (float)lng_deg + (float)lng_min / 60.0
			+ (float)(lng_min_frag - lng_min_frag % 10 + 5) / 6000.0;
		break;
	case 2:
		/* ignore lng_min_frag */
		lng = (float)lng_deg + ((float)lng_min + 0.5) / 60.0;
		break;
	case 3:
		/* ignore lng_min_frag and last number of lng_min */
		lng = (float)lng_deg + (float)(lng_min - lng_min % 10 + 5) / 60.0;
		break;
	case 4:
		/* minute is unused -> add 0.5 degrees to longitude */
		lng = (float)lng_deg + 0.5;
		break;
	default:
		DEBUG_LOG(".. posambiguity code BUG!");
		return 0;
	}
	
	/* check the longitude E/W sign */
	if (d_start[5] >= 0x50)
		lng = 0 - lng;
	
	/* save the symbol table and code */
	sym_code = body[6];
	sym_table = body[7];
	
	/* ok, we're done */
	/*
	fprintf(stderr, "\tlat %u %u.%u (%.4f) lng %u %u.%u (%.4f)\n",
	 	lat_deg, lat_min, lat_min_frag, lat,
	 	lng_deg, lng_min, lng_min_frag, lng);
	fprintf(stderr, "\tsym '%c' '%c'\n", sym_table, sym_code);
	*/
	/* Now onto speed and course. */
	speed = (body[3] - 28) * 10;
	course_speed = body[4] - 28;
	course_speed_tmp = floor(course_speed / 10);
	speed += course_speed_tmp;
	course_speed -= course_speed_tmp * 10;
	course = 100 * course_speed;
	course += body[5] - 28;
	
	/* Some adjustment. */
	if ( speed >= 800 )
	{
		speed -= 800;
	}
	if ( course >= 400 )
	{
		course -= 400;
	}
  
	/* Save Speed/Course values. */
// 	packet->speed = malloc(sizeof(double));
// 	if ( !packet->speed ) return 0;
	pb->speed = speed * KNOT_TO_KMH;
	if ( course >= 0 )
	{
		pb->flags |= F_CSRSPD;
// 		packet->course = malloc(sizeof(unsigned int));
// 		if ( !packet->course ) return 0;
		pb->course = course;
	}
	
	return pbuf_fill_pos(pb, lat, lng, sym_table, sym_code);
}

/*
 *	Parse a compressed APRS position packet
 *
 *	APRS PROTOCOL REFERENCE 1.0.1 Chapter 9, page 36 (46 in PDF)
 *
 */

int ParseAPRS::parse_aprs_compressed(struct pbuf_t *pb, const char *body, const char *body_end)
{
	char sym_table, sym_code;
	int i;
	int lat1, lat2, lat3, lat4;
	int lng1, lng2, lng3, lng4;
	float lat, lng;
	char c1, s1, comptype;
	char cs;
	
	DEBUG_LOG("parse_aprs_compressed");
	
	/* A compressed position is always 13 characters long.
	 * Make sure we get at least 13 characters and that they are ok.
	 * Also check the allowed base-91 characters at the same time.
	 */ 
	
	if (body_end - body < 13) {
		DEBUG_LOG("\ttoo short");
		return 0; /* too short. */
	}
	
	sym_table = body[0]; /* has been validated before entering this function */
	sym_code = body[9];
	
	/* base-91 check */
	for (i = 1; i <= 8; i++)
		if (body[i] < 0x21 || body[i] > 0x7b)
			return 0;
	
	// fprintf(stderr, "\tpassed length and format checks, sym %c%c\n", sym_table, sym_code);
	
	/* decode */
	lat1 = (body[1] - 33);
	lat2 = (body[2] - 33);
	lat3 = (body[3] - 33);
	lat4 = (body[4] - 33);

	lat1 = ((((lat1 * 91) + lat2) * 91) + lat3) * 91 + lat4;

	lng1 = (body[5] - 33);
	lng2 = (body[6] - 33);
	lng3 = (body[7] - 33);
	lng4 = (body[8] - 33);

	/* Get other data. */
	c1 = body[10] - 33;
	s1 = body[11] - 33;
	comptype = body[12] - 33;

	lng1 = ((((lng1 * 91) + lng2) * 91) + lng3) * 91 + lng4;
	
	/* calculate latitude and longitude */
	
	lat =   90.0F - ((float)(lat1) / 380926.0F);
	lng = -180.0F + ((float)(lng1) / 190463.0F);

	/* Save symbol table. Table chars (a-j) are converted to numbers 0-9. */
	if (sym_table >= 'a' && sym_table <= 'j')
	{
		sym_table -= 81;
	}

	pb->pos_resolution = 0.291;

	/* GPS fix status, only if csT is used. */
	if (c1 != -1)
	{
		if ((comptype & 0x20) == 0x20)
		{
			pb->gps_fix_status = 1;
		}
		else
		{
			pb->gps_fix_status = 0;
		}
	}

	/* Check the compression type, if GPGGA, then the cs bytes are altitude.
	 * Otherwise try to decode it as course and speed and finally as radio range.
	 * If c is space, then csT is not used. Also require that s is not a space. */
	if (c1 == -1 || s1 == -1)
	{
		/* csT not used. */
	}
	else if ((comptype & 0x18) == 0x10)
	{
		/* cs is altitude. */
		cs = c1 * 91 + s1;
		// 		packet->altitude = malloc(sizeof(double));
		// 		if ( !packet->altitude ) return 0;
				/* Convert directly to meters. */
		pb->altitude = pow(1.002, cs) * 0.3048;
		pb->flags |= F_ALT;
	}
	else if (c1 >= 0 && c1 <= 89)
	{
		// 		packet->course = malloc(sizeof(unsigned int));
		// 		if ( !packet->course ) return 0;
		if (c1 == 0)
		{
			/* Special case of north, APRS spec uses zero for unknown and 360 for north.
			 * So remember to convert north here. */
			pb->course = 360;
		}
		else
		{
			pb->course = c1 * 4;
		}
		/* Convert directly to km/h. */
		pb->speed = (double)((pow(1.08, s1) - 1) * KNOT_TO_KMH);
		pb->flags |= F_CSRSPD;
	}
	else if (c1 == 90)
	{
		/* Convert directly to km. */
		pb->radio_range = 2 * pow(1.08, s1) * MPH_TO_KMH;
	}
	
	parse_aprs_comment(pb, body + 13, (unsigned int)(body_end - body - 13));
	return pbuf_fill_pos(pb, lat, lng, sym_table, sym_code);
}

/*
 *	Parse an uncompressed "normal" APRS packet
 *
 *	APRS PROTOCOL REFERENCE 1.0.1 Chapter 8, page 32 (42 in PDF)
 *
 */

int ParseAPRS::parse_aprs_uncompressed(struct pbuf_t *pb, const char *body, const char *body_end)
{
	char posbuf[20];
	unsigned int lat_deg = 0, lat_min = 0, lat_min_frag = 0, lng_deg = 0, lng_min = 0, lng_min_frag = 0;
	float lat, lng;
	char lat_hemi, lng_hemi;
	char sym_table, sym_code;
	int issouth = 0;
	int iswest = 0;
	
	DEBUG_LOG("parse_aprs_uncompressed");
	
	if (body_end - body < 19) {
		DEBUG_LOG("\ttoo short");
		return 0;
	}
	
	/* make a local copy, so we can overwrite it at will. */
	memcpy(posbuf, body, 19);
	posbuf[19] = 0;
	// fprintf(stderr, "\tposbuf: %s\n", posbuf);
	
	// position ambiquity is going to get ignored now,
        // it's not needed in this application.

	/* lat */
	if (posbuf[2] == ' ') posbuf[2] = '3';
	if (posbuf[3] == ' ') posbuf[3] = '5';
	if (posbuf[5] == ' ') posbuf[5] = '5';
	if (posbuf[6] == ' ') posbuf[6] = '5';
	/* lng */
	if (posbuf[12] == ' ') posbuf[12] = '3';
	if (posbuf[13] == ' ') posbuf[13] = '5';
	if (posbuf[15] == ' ') posbuf[15] = '5';
	if (posbuf[16] == ' ') posbuf[16] = '5';
	
	// fprintf(stderr, "\tafter filling amb: %s\n", posbuf);
	/* 3210.70N/13132.15E# */
	if (sscanf(posbuf, "%2u%2u.%2u%c%c%3u%2u.%2u%c%c",
	    &lat_deg, &lat_min, &lat_min_frag, &lat_hemi, &sym_table,
	    &lng_deg, &lng_min, &lng_min_frag, &lng_hemi, &sym_code) != 10) {
		DEBUG_LOG("\tsscanf failed");
		return 0;
	}
	
	if (!valid_sym_table_uncompressed(sym_table))
		sym_table = 0;
	
	if (lat_hemi == 'S' || lat_hemi == 's')
		issouth = 1;
	else if (lat_hemi != 'N' && lat_hemi != 'n')
		return 0; /* neither north or south? bail out... */
	
	if (lng_hemi == 'W' || lng_hemi == 'w')
		iswest = 1;
	else if (lng_hemi != 'E' && lng_hemi != 'e')
		return 0; /* neither west or east? bail out ... */
	
	if (lat_deg > 89 || lng_deg > 179)
		return 0; /* too large values for lat/lng degrees */
	
	lat = (float)lat_deg + (float)lat_min / 60.0 + (float)lat_min_frag / 6000.0;
	lng = (float)lng_deg + (float)lng_min / 60.0 + (float)lng_min_frag / 6000.0;
	
	/* Finally apply south/west indicators */
	if (issouth)
		lat = 0.0 - lat;
	if (iswest)
		lng = 0.0 - lng;
	
	// fprintf(stderr, "\tlat %u %u.%u %c (%.3f) lng %u %u.%u %c (%.3f)\n",
	// 	lat_deg, lat_min, lat_min_frag, (int)lat_hemi, lat,
	// 	lng_deg, lng_min, lng_min_frag, (int)lng_hemi, lng);
	// fprintf(stderr, "\tsym '%c' '%c'\n", sym_table, sym_code);
	char *rest = (char*)body + 19;
	//if (sym_code == '_') {
	if (((rest[3] == '/')||(rest[0]=='c'))) {
		char* wx = strchr(rest, 'g');
		if (wx) {
			wx += 4;
			if (*wx == 't') {
				//pb->packettype |= T_WX;
				parse_aprs_wx(pb, body + 19, (unsigned int)(body_end - body - 19));
				return pbuf_fill_pos(pb, lat, lng, sym_table, sym_code);
			}
		}
		parse_aprs_comment(pb, body + 19, (unsigned int)(body_end - body - 19));
	}
	else {
		parse_aprs_comment(pb, body + 19, (unsigned int)(body_end - body - 19));
	}

	return pbuf_fill_pos(pb, lat, lng, sym_table, sym_code);
}

/*
 *	Parse an APRS object 
 *
 *	APRS PROTOCOL REFERENCE 1.0.1 Chapter 11, page 58 (68 in PDF)
 *
 */

int ParseAPRS::parse_aprs_object(struct pbuf_t *pb, const char *body, const char *body_end)
{
	int i;
	int namelen = -1;
	
	pb->packettype |= T_OBJECT;
	
	DEBUG_LOG("parse_aprs_object");
	
	/* check that the object name ends with either * or _ */
	if (*(body + 9) != '*' && *(body + 9) != '_') {
		DEBUG_LOG("\tinvalid object kill character");
		return 0;
	}

	/* check that the timestamp ends with one of the valid timestamp type IDs */
        char tz_end = body[16];
        if (tz_end != 'z' && tz_end != 'h' && tz_end != '/') {
        	DEBUG_LOG("\tinvalid object timestamp character: '%c'", tz_end);
		return 0;
	}
	
	/* check object's name - scan for non-printable characters and the last
	 * non-space character
	 */
	for (i = 0; i < 9; i++) {
		if (body[i] < 0x20 || body[i] > 0x7e) {
			DEBUG_LOG("\tobject name has unprintable characters");
			return 0; // non-printable
		}
		if (body[i] != ' ')
			namelen = i;
	}
	
	if (namelen < 0) {
		DEBUG_LOG("\tobject has empty name");
		return 0;
	}
	
	pb->srcname = body;
	pb->srcname_len = namelen+1;
	
	DEBUG_LOG("object name: '%.*s'\n", pb->srcname_len, pb->srcname);
	
	/* Forward the location parsing onwards */
	if (valid_sym_table_compressed(body[17]))
		return parse_aprs_compressed(pb, body + 17, body_end);
	
	if (body[17] >= '0' && body[17] <= '9')
		return parse_aprs_uncompressed(pb, body + 17, body_end);
	
	DEBUG_LOG("no valid position in object");
	
	return 0;
}

/*
 *	Parse an APRS item
 *
 *	APRS PROTOCOL REFERENCE 1.0.1 Chapter 11, page 59 (69 in PDF)
 *
 */

int ParseAPRS::parse_aprs_item(struct pbuf_t *pb, const char *body, const char *body_end)
{
	int i;
	
	pb->packettype |= T_ITEM;
	
	DEBUG_LOG("parse_aprs_item");
	
	/* check item's name - scan for non-printable characters and the
	 * ending character ! or _
	 */
	for (i = 0; i < 9 && body[i] != '!' && body[i] != '_'; i++) {
		if (body[i] < 0x20 || body[i] > 0x7e) {
			DEBUG_LOG("\titem name has unprintable characters");
			return 0; /* non-printable */
		}
	}
	
	if (body[i] != '!' && body[i] != '_') {
		DEBUG_LOG("\titem name ends with neither ! or _");
		return 0;
	}
	
	if (i < 3 || i > 9) {
		DEBUG_LOG("\titem name has invalid length");
		return 0;
	}
	
	pb->srcname = body;
	pb->srcname_len = i;
	
	//fprintf(stderr, "\titem name: '%.*s'\n", pb->srcname_len, pb->srcname);
	
	/* Forward the location parsing onwards */
	i++;
	if (valid_sym_table_compressed(body[i]))
		return parse_aprs_compressed(pb, body + i, body_end);
	
	if (body[i] >= '0' && body[i] <= '9')
		return parse_aprs_uncompressed(pb, body + i, body_end);
	
	//parse_aprs_comment(pb, body + 29, (unsigned int)(body_end - body - 29));
	DEBUG_LOG("\tno valid position in item");
	
	return 0;
}


#if 0
int parse_aprs_txgate(struct pbuf_t *pb, int look_inside_3rd_party, historydb_t *historydb)
{
	int rc = parse_aprs(pb, look_inside_3rd_party, historydb);

	if (pb->packettype & T_THIRDPARTY) {
	  // Tx-IGate needs to know from RF received frames, if there is
	  // source address that arrived from an Tx-IGate...

	  const char *body;
	  const char *body_end;
	  const char *pos_start;
	  const char *info_start = pb->info_start;
	
	  

	}
	return rc;
}
#endif

/*
 *	Try to parse an APRS packet.
 *	Returns 1 if position was parsed successfully,
 *	0 if parsing failed.
 *
 *	Does also front-end part of the output filter's
 *	packet type classification job.
 *
 * TODO: Recognize TELEM packets in !/=@ packets too!
 *
 *	Return 0 for parse failures, 1 for OK.
 */


int ParseAPRS::parse_aprs(struct pbuf_t* pb)
{
	char packettype, poschar;
	int paclen;
	int rc;
	const char *body;
	const char *body_end;
	const char *pos_start;
	const char *info_start = pb->info_start;

	int look_inside_3rd_party = 1; // Look there once..

	pb->packettype = T_ALL;
	pb->flags = 0;

	if (!pb->info_start)
		return 0;

	if (pb->data[0] == 'C' && /* Perhaps CWOP ? */
		pb->data[1] == 'W') {
		const char *s = pb->data + 2;
		const char *pe = pb->data + pb->packet_len;
		for (; *s && s < pe; ++s) {
			int c = *s;
			if (c < '0' || c > '9')
				break;
		}
		if (*s == '>')
			pb->packettype |= T_CWOP;
	}

	/* the following parsing logic has been translated from Ham::APRS::FAP
	 * Perl module to C
	 */

	 // ignore the CRLF in the end of the body
	body_end = pb->data + pb->packet_len;

	do {
		// body is right after the packet type character
		body = info_start + 1;

		// length of the info field:
		paclen = body_end - info_start;

		if (paclen < 1) return 0; // consumed all, or empty packet

		// Check the first character of the packet and
		// determine the packet type
		packettype = *info_start;
//Serial.println(packettype);
		// Exit this loop unless it is 3rd-party frame
		if (packettype != '}') break;

		// Look for ':' character separating address block
		// from 3rd-party body
		info_start = (char*)memchr(body, ':', (int)(body_end - body));
		if (info_start == NULL) {
			// Not valid 3rd party frame!
			return 0;
		}
		pb->packettype |= T_THIRDPARTY;
		if (!look_inside_3rd_party)
			return 1; // Correct 3rd-party, don't look further.

		// Look once inside the 3rd party frame,
		// this is used in aprx's tx-igate, which builds
		// the 3rd-party frame before parsing message-to-be-tx:ed
		// .. and doing content filters.
		--look_inside_3rd_party;
		pb->packettype = 0;

		// Skip over the ':'
		++info_start;
		continue;  // and loop..

	} while (1);

	switch (packettype) {
		/* the following are obsolete mic-e types: 0x1c 0x1d
		 * case 0x1c:
		 * case 0x1d:
		 */
	case 0x27: /* ' */
	case 0x60: /* ` */
		/* could be mic-e, minimum body length 9 chars */
		if (paclen >= 9) {
			pb->packettype |= T_POSITION;
			rc = parse_aprs_mice(pb,(const unsigned char*)body,(const unsigned char*)body_end);
			parse_aprs_comment(pb, body + 9, (unsigned int)(body_end - body - 9));
			DEBUG_LOG("\n");
			return rc;
		}
		return 0; // bad

	case '!':
		if (*body == '!') { /* Ultimeter 2000 - "tnc2addr:!!" */
			pb->symbol[0] = '/';
			pb->symbol[1] = '_';
			pb->symbol[2] = 0;
			pb->packettype |= T_WX;
			parse_aprs_wx(pb, body, (unsigned int)(body_end - body));
			return 1; // Known Ultimeter format
		}
	case '=':
	case '/':
	case '@':
		/* check that we won't run over right away */
		if (body_end - body < 10)
			return 0; // bad
		/* Normal or compressed location packet, with or without
		 * timestamp, with or without messaging capability
		 *
		 * ! and / have messaging, / and @ have a prepended timestamp
		 */
		pb->packettype |= T_POSITION;
		if (packettype == '/' || packettype == '@') {
			/* With a prepended timestamp, jump over it. */
			body += 7;
		}
		poschar = *body;
		if (valid_sym_table_compressed(poschar)) { /* [\/\\A-Za-j] */
				/* compressed position packet */
			rc = 0;
			if (body_end - body >= 13) {
				rc = parse_aprs_compressed(pb, body, body_end);
			}
			DEBUG_LOG("\n");
			return rc;

		}
		else if (poschar >= 0x30 && poschar <= 0x39) { /* [0-9] */
		 /* normal uncompressed position */
			rc = 0;
			if (body_end - body >= 19) {
				rc = parse_aprs_uncompressed(pb, body, body_end);
			}
			DEBUG_LOG("\n");
			return rc;
		}
		return 0;

	case '$':
		pb->symbol[0] = '\\';
		pb->symbol[1] = 's';
		pb->symbol[2] = 0;
		if (body_end - body > 10) {
			// Is it OK to declare it as position packet ?
			rc = parse_aprs_nmea(pb, body, body_end);
			parse_aprs_comment(pb, body + 10, (unsigned int)(body_end - body - 10));
			DEBUG_LOG("\n");
			return rc;
		}
		return 0;

	case ':':
		pb->symbol[0] = '/';
		pb->symbol[1] = ']';
		pb->symbol[2] = 0;
		pb->packettype |= T_MESSAGE;
		// quick and loose way to identify NWS and SKYWARN messages
		// they do apparently originate from "WXSRV", but that is not
		// guaranteed thing...
		if (memcmp(body, "NWS-", 4) == 0) // as seen on specification
			pb->packettype |= T_NWS;
		if (memcmp(body, "NWS_", 4) == 0) // as seen on data
			pb->packettype |= T_NWS;
		if (memcmp(body, "SKY", 3) == 0)  // as seen on specification
			pb->packettype |= T_NWS;

		// Is it perhaps TELEMETRY related "message" ?
		if (body[9] == ':' &&
			(memcmp(body + 9, ":PARM.", 6) == 0 ||
				memcmp(body + 9, ":UNIT.", 6) == 0 ||
				memcmp(body + 9, ":EQNS.", 6) == 0 ||
				memcmp(body + 9, ":BITS.", 6) == 0)) {
			pb->packettype &= ~T_MESSAGE;
			pb->packettype |= T_TELEMETRY;
			// Fall through to recipient location lookup
			pb->symbol[0] = '/';
			pb->symbol[1] = 't';
			pb->symbol[2] = 0;
			parse_aprs_telem(pb, body+10, body_end);
		}

		// Or perhaps a DIRECTED QUERY ?
		if (body[9] == ':' && body[10] == '?') {
			pb->packettype &= ~T_MESSAGE;
			pb->packettype |= T_QUERY;
			// Fall through to recipient location lookup
		}

		// Now find out if the message RECIPIENT address is known
		// to have some location data ?  Because then we can treat
		// them the same way in filters as we do those with real
		// positions..
		{
			const char *p;
			int i;
			//#ifndef DISABLE_IGATE
			//			history_cell_t *history;
			//#endif
			pb->dstname = body;
			p = body;
			for (i = 0; i < CALLSIGNLEN_MAX; ++i) {
				// the recipient address is space padded
				// to 9 chars, while our historydb is not.
				if (*p == 0 || *p == ' ' || *p == ':')
					break;
				p++;
			}
//Serial.println(p);
			pb->dstname_len = p - body;
			//#ifndef DISABLE_IGATE
			//                        if (historydb != NULL) {
			//                        	history = historydb_lookup( historydb, pb->dstname, i );
			//                                if (history != NULL) {
			//					pb->lat     = history->lat;
			//                                        pb->lng     = history->lon;
			//                                        pb->cos_lat = history->coslat;
			//			    
			//                                        pb->flags  |= F_HASPOS;
			//                                }
			//                        }
			//#endif
		}
		parse_aprs_message(pb);
		parse_aprs_comment(pb, body, (unsigned int)(body_end - body));
		return 1;

	case ';':
		if (body_end - body > 29) {
			rc = parse_aprs_object(pb, body, body_end);
			DEBUG_LOG("\n");
			return rc;
		}
		return 0; // too short

	case '>':
		pb->symbol[0] = '/';
		pb->symbol[1] = 'B';
		pb->symbol[2] = 0;
		pb->packettype |= T_STATUS;
		parse_aprs_comment(pb, body, (unsigned int)(body_end - body));
		return 1; // ok

	case '<':
		pb->packettype |= T_STATCAPA;
		return 1; // ok

	case '?':
		pb->symbol[0] = '\\';
		pb->symbol[1] = '?';
		pb->symbol[2] = 0;
		pb->packettype |= T_QUERY;
		parse_aprs_comment(pb, body, (unsigned int)(body_end - body));
		return 1; // ok at igate/digi

	case ')':
		if (body_end - body > 18) {
			rc = parse_aprs_item(pb, body, body_end);
			DEBUG_LOG("\n");
			return rc;
		}
		return 0; // too short


	case 'T':
		pb->symbol[0] = '/';
		pb->symbol[1] = '?';
		pb->symbol[2] = 0;
		if (body_end - body > 18) {
			pb->packettype |= T_TELEMETRY;
			rc = parse_aprs_telem(pb, body, body_end);
			//DEBUG_LOG("\n");
			parse_aprs_comment(pb, body, (unsigned int)(body_end - body));
			return rc;
		}
		return 0; // too short

	case '#': /* Peet Bros U-II Weather Station */
	case '*': /* Peet Bros U-I  Weather Station */
	case '_': /* Weather report without position */
		/* symbol table and code */
		pb->symbol[0] = '/';
		pb->symbol[1] = '_';
		pb->symbol[2] = 0;
		//pb->packettype |= T_WX;
		parse_aprs_wx(pb, body, (unsigned int)(body_end - body));
		return 1; // good

	case '{':
		pb->packettype |= T_USERDEF;
		return 1; // okay at digi?

		// the packettype is never '}'
		// case '}':
				// pb->packettype |= T_THIRDPARTY;
		// return 1; // 3rd-party is okay at digi

	default:
		break;
	}

	/* When all else fails, try to look for a !-position that can
	 * occur anywhere within the 40 first characters according
	 * to the spec.  (X1J TNC digipeater bugs...)
	 */
	pos_start = (char*)memchr(body, '!', body_end - body);
	if ((pos_start) && pos_start - body <= 39) {
		poschar = *pos_start;
		if (valid_sym_table_compressed(poschar)) { /* [\/\\A-Za-j] */
				/* compressed position packet */
			int rc = 0;
			if (body_end - pos_start >= 13) {
				rc = parse_aprs_compressed(pb, pos_start, body_end);
				//parse_aprs_comment(pb, body + 19, (unsigned int)(body_end - body - 19));
				DEBUG_LOG("\n");
			}
			return rc;
		}
		else if (poschar >= 0x30 && poschar <= 0x39) { /* [0-9] */
		 /* normal uncompressed position */
			int rc = 0;
			if (body_end - pos_start >= 19)
				rc = parse_aprs_uncompressed(pb, pos_start, body_end);
			DEBUG_LOG("\n");
			return rc;
		}
	}

	return 0; // bad
}

/*
 *      Parse an aprs text message (optional, only done to messages addressed to
 *      SERVER
 */

int ParseAPRS::parse_aprs_message(struct pbuf_t *pb)
{
        const char *p;
		aprs_message_t* am;
		am = &pb->msg;
        
        memset(am, 0, sizeof(*am));
        
        if (!(pb->packettype & T_MESSAGE))
                return -1;
                
        if (pb->info_start[10] != ':')
                return -2;
        
        am->body = pb->info_start + 11;
        /* -2 for the CRLF already in place */
        am->body_len = pb->packet_len - 2 - (pb->info_start - pb->data);
 
		//DEBUG_LOG("Body: ");
		//DEBUG_LOG(am->body);
		//DEBUG_LOG("Len: ");
		//DEBUG_LOG(am->body_len);
        /* search for { looking backwards from the end of the packet,
         * it separates the msgid
         */
        p = am->body + am->body_len - 1;
        while (p > am->body && *p != '{')
                p--;
		//DEBUG_LOG("ACK: ");
		//DEBUG_LOG(p);
        if (*p == '{') {
                am->msgid = p+1;
                am->msgid_len = pb->packet_len - 2 - (am->msgid - pb->data);
                am->body_len = p - am->body;
        }
		//p[am->body_len] = 0;
        /* check if this is an ACK */
        if ((!am->msgid_len) && am->body_len > 3
            && am->body[0] == 'a' && am->body[1] == 'c' && am->body[2] == 'k') {
                am->is_ack = 1;
                am->msgid = am->body + 3;
                am->msgid_len = am->body_len - 3;
                am->body_len = 0;
//				am->msgid[am->msgid_len] = 0;
                return 0;
        }

        /* check if this is an REJ */
        if ((!am->msgid_len) && am->body_len > 3
            && am->body[0] == 'r' && am->body[1] == 'e' && am->body[2] == 'j') {
                am->is_rej = 1;
                am->msgid = am->body + 3;
                am->msgid_len = am->body_len - 3;
                am->body_len = 0;
                return 0;
        }
        
        return 0;
}

char* ParseAPRS::parse_remove_part(char const* input, unsigned int const input_len,unsigned int const part_so, unsigned int const part_eo,unsigned int* result_len)
{
	unsigned int i, part_i;
	char* result;


	/* Check params. */
	if (!input || !input_len || part_so >= input_len || part_eo > input_len || part_so >= part_eo)
	{
		*result_len = 0;
		return NULL;
	}

	/* Calculate size of result. */
	*result_len = input_len - (part_eo - part_so);
	if (*result_len == 0)
	{
		return NULL;
	}

	/* Copy input into result. */
	result = (char*)input;
	part_i = 0;
	for (i = 0; i < input_len; ++i)
	{
		/* Skip given part. */
		if (i < part_so || i >= part_eo)
		{
			result[part_i] = input[i];
			++part_i;
		}
	}

	///* Add 0 to the end. */
	result[part_i] = 0;

	return result;
}

int ParseAPRS::parse_aprs_wx(struct pbuf_t* pb, char const* input, unsigned int const input_len)
{
	int flage = 0;
	static char wind_dir[4], wind_speed[4], wind_gust[4], temperature[4],rain[4],rain24[4],rainMn[4],humidity[3],barometric[6], luminosity[4],uv[3];
	bool luminosityAbove = false;
	char buf_5b[6];
	int len, retval = 1;
	char* rest = NULL, * tmp_str;
	unsigned int rest_len, tmp_us;

	/* Check that we have something to look at. */
	if (!pb || !input || !input_len)
	{
		return 0;
	}


	/* Initialize result vars. */
	memset(wind_dir, 0, 4);
	memset(wind_speed, 0, 4);
	memset(luminosity, 0, 4);
	memset(uv, 0, 3);

	/* Look for wind and temperature. Remaining bytes are copied to report var. */
		rest = (char*)input;
		rest_len = input_len;

//Serial.print(rest);
		if (rest[3] == '/') {
			memcpy(wind_dir, rest, 3);
			wind_dir[3] = 0;
			memcpy(wind_speed, rest + 4, 3);
			wind_speed[3] = 0;
			tmp_str = parse_remove_part(rest, rest_len, 0, 7, &tmp_us);
			rest = tmp_str;
			rest_len = tmp_us;
		}else {
			if (tmp_str = (strchr(rest, 'c'))) {
				memcpy(wind_dir, tmp_str + 1, 3);
				wind_dir[3] = 0;
				tmp_str = parse_remove_part(rest, rest_len, tmp_str - rest, tmp_str - rest + 4, &tmp_us);
				rest = tmp_str;
				rest_len = tmp_us;
				flage++;
			}
			if (tmp_str = (strchr(rest, 's'))) {
				memcpy(wind_speed, tmp_str + 1, 3);
				wind_speed[3] = 0;
				tmp_str = parse_remove_part(rest, rest_len, tmp_str - rest, tmp_str - rest + 4, &tmp_us);
				rest = tmp_str;
				rest_len = tmp_us;
			}
		}

		if (tmp_str = (strchr(rest, 'g'))) {
			memcpy(wind_gust, tmp_str + 1, 3);
			wind_gust[3] = 0;
			tmp_str = parse_remove_part(rest, rest_len, tmp_str - rest, tmp_str - rest + 4, &tmp_us);
			rest = tmp_str;
			rest_len = tmp_us;
			flage++;
		}

		if (tmp_str = (strchr(rest, 't'))) {
			memcpy(temperature, tmp_str + 1, 3);
			temperature[3] = 0;
			tmp_str = parse_remove_part(rest, rest_len, tmp_str - rest, tmp_str - rest + 4, &tmp_us);
			rest = tmp_str;
			rest_len = tmp_us;
			flage++;
		}

		if (tmp_str = (strchr(rest, 'r'))) {
			memcpy(rain, tmp_str + 1, 3);
			rain[3] = 0;
			tmp_str = parse_remove_part(rest, rest_len, tmp_str - rest, tmp_str - rest + 4, &tmp_us);
			rest = tmp_str;
			rest_len = tmp_us;
			flage++;
		}

		if (tmp_str = (strchr(rest, 'p'))) {
			memcpy(rain24, tmp_str + 1, 3);
			rain24[3] = 0;
			tmp_str = parse_remove_part(rest, rest_len, tmp_str - rest, tmp_str - rest + 4, &tmp_us);
			rest = tmp_str;
			rest_len = tmp_us;
			flage++;
		}
		if (tmp_str = (strchr(rest, 'P'))) {
			memcpy(rainMn, tmp_str + 1, 3);
			rainMn[3] = 0;
			tmp_str = parse_remove_part(rest, rest_len, tmp_str - rest, tmp_str - rest + 4, &tmp_us);
			rest = tmp_str;
			rest_len = tmp_us;
			flage++;
		}

		if (tmp_str = (strchr(rest, 'h'))) {
			memcpy(humidity, tmp_str + 1, 2);
			humidity[2] = 0;
			tmp_str = parse_remove_part(rest, rest_len, tmp_str - rest, tmp_str - rest + 3, &tmp_us);
			rest = tmp_str;
			rest_len = tmp_us;
			flage++;
		}
		if (tmp_str = (strchr(rest, 'b'))) {
			memcpy(barometric, tmp_str + 1,5);
			barometric[5] = 0;
			tmp_str = parse_remove_part(rest, rest_len, tmp_str - rest, tmp_str - rest + 6, &tmp_us);
			rest = tmp_str;
			rest_len = tmp_us;
			flage++;
		}

		if (tmp_str = (strchr(rest, 'l'))) {
			memcpy(luminosity, tmp_str + 1, 3);
			if (is_number(luminosity)) {
				luminosityAbove = true;
				luminosity[3] = 0;
				tmp_str = parse_remove_part(rest, rest_len, tmp_str - rest, tmp_str - rest + 4, &tmp_us);
				rest = tmp_str;
				rest_len = tmp_us;
				flage++;
			}
		}
		if (tmp_str = (strchr(rest, 'L'))) {
			memcpy(luminosity, tmp_str + 1, 3);
			if (is_number(luminosity)) {
				luminosityAbove = false;
				luminosity[3] = 0;
				tmp_str = parse_remove_part(rest, rest_len, tmp_str - rest, tmp_str - rest + 4, &tmp_us);
				rest = tmp_str;
				rest_len = tmp_us;
				flage++;
			}
		}

		if (tmp_str = (strchr(rest, 'u'))) {
			memcpy(uv, tmp_str + 1, 2);
			uv[2] = 0;
			tmp_str = parse_remove_part(rest, rest_len, tmp_str - rest, tmp_str - rest + 3, &tmp_us);
			rest = tmp_str;
			rest_len = tmp_us;
			flage++;
		}

	//packet->format = fapPOS_WX;
	/* Save values. */

		if (is_number(wind_gust))
		{
			pb->wx_report.flags |= W_WG;
			pb->wx_report.wind_gust = atof(wind_gust) * MPH_TO_MS;
		}
		if (is_number(wind_dir))
		{
			pb->wx_report.flags |= W_WD;
			pb->wx_report.wind_dir = atoi(wind_dir);
		}
		if (is_number(wind_speed))
		{
			pb->wx_report.flags |= W_WS;
			pb->wx_report.wind_speed = atof(wind_speed) * MPH_TO_MS;
		}
		if (is_number(temperature))
		{
			pb->wx_report.flags |= W_TEMP;
			pb->wx_report.temp = FAHRENHEIT_TO_CELCIUS(atof(temperature));
		}
		/* Then some rain values. */
		if (is_number(rain))
		{
			pb->wx_report.flags |= W_R1H;
			pb->wx_report.rain_1h = atof(rain) * HINCH_TO_MM;
		}
		if (is_number(rain24))
		{
			pb->wx_report.flags |= W_R24H;
			pb->wx_report.rain_24h = atof(rain24) * HINCH_TO_MM;
		}
		if (is_number(rainMn))
		{
			pb->wx_report.flags |= W_RMN;
			pb->wx_report.rain_midnight = atof(rainMn) * HINCH_TO_MM;
		}
	
	/* Humidity. */
		if (is_number(humidity))
		{
			pb->wx_report.flags |= W_HUM;
			pb->wx_report.humidity = atoi(humidity);
		}

	/* Pressure. */
		if (is_number(barometric))
		{
			pb->wx_report.flags |= W_BAR;
			pb->wx_report.pressure = atoi(barometric) / 10.0; // tenths of mbars to mbars
		}

	/* Luminosity. */
		if (is_number(luminosity))
		{
			pb->wx_report.flags |= W_PAR;
			pb->wx_report.luminosity = atoi(luminosity);
			if (luminosityAbove) {
				pb->wx_report.luminosity += 1000;
			}
		}
	/* UV Index. */
		if (is_number(uv))
		{
			pb->wx_report.flags |= W_UV;
			pb->wx_report.uv = atoi(uv);
		}
	
		//if (pb->wx_report.flags != 0) pb->packettype |= T_WX;

	if (rest_len > 0)
	{
		pb->comment = rest;
		pb->comment_len = rest_len;
	}
	if(flage>3) pb->packettype |= T_WX;
//	Serial.println("WX Decode compress");
	return 1;
}

void ParseAPRS::parse_aprs_comment(struct pbuf_t* pb, char const* input, unsigned int const input_len)
{
	char course[4], speed[4], range[5], altitude[7], dao[3], id[9];
	int i, tmp_s;
	char* tmp_str, * rest = NULL;
	unsigned int rest_len = 0, tmp_us;
	char *res;

	DEBUG_LOG("\nDATA: %s\nLEN: %d\n",input,input_len);

	if (input_len >= 7)
	{
		/* Look for data. */
		if (input[3] == '/')
		{
			pb->flags |= F_CSRSPD;
			/* Get and validate course, if not already available. 0 stands for invalid. */
			if (!pb->course)
			{
				memcpy(course, input, 3);
				course[3] = 0;
				pb->course = 0;
//Serial.println(course);
				if (isdigit(course[0]) && isdigit(course[1]) && isdigit(course[2]))
				{
					tmp_s = atoi(course);
					if (tmp_s >= 1 && tmp_s <= 360)
					{
						/* It's valid, let's save it. */
						pb->course = tmp_s;
					}
				}
			}

			/* Get and validate speed, if not available already. */
			if (!pb->speed)
			{
				/* There's no speed value for invalid, so we leave it unallocated by default. */
				memcpy(speed, input + 4, 3);
				speed[3] = 0;
//Serial.println(speed);
				if (isdigit(speed[0]) && isdigit(speed[1]) && isdigit(speed[2]))
				{
					tmp_s = atoi(&speed[0]);
					pb->speed = (double)(tmp_s * KNOT_TO_KMH);
				}
			}

			/* Save the rest. */
			//rest = parse_remove_part(input, input_len, res - input, res - input + 7, &rest_len);
			tmp_str = parse_remove_part(input, input_len, 0, 7, &tmp_us);
			rest = tmp_str;
			rest_len = tmp_us;
//Serial.println(rest);
			//skip = 7;
		}
		/* Look for PHGR. */
		//else if (strstr(input,"PHG") == 0 && (input[4] >= 0x30 && input[4] <= 0x7e))
		//{
		//	/* Save PHGR. */
		//	memcpy(pb->phg, input + 3, 5);
		//	pb->phg[5] = 0;

		//	/* Save the rest. */
		//	rest = parse_remove_part(input, input_len, 0, 8, &rest_len);
		//	//skip = 8;
		//}
		/* Look for PHG. */
		//else if (strstr(input, "PHG") == 0 && (input[3] >= 0x30 && input[3] <= 0x39))
		if (res = strstr((char *) input, "PHG"))
		{
			pb->flags |= F_PHG;
			/* Save PHG. */
			memcpy(pb->phg, res + 3, 4);
			pb->phg[4] = 0;
//Serial.println(pb->phg);
			/* Save the rest. */
			//skip = 7;
			//rest = parse_remove_part(input, input_len, 0, 7, &rest_len);
			rest = parse_remove_part(input, input_len, res - input, res - input + 7, &rest_len);
		}
		/* Look for RNG. */
		else if (res=strstr((char *) input, "RNG"))
		{
			pb->flags |= F_RNG;
			/* Save and validate range. There's no invalid range value. */
			memcpy(range, res + 3, 4);
			range[4] = 0;
			tmp_s = atoi(range);
			pb->radio_range = (double)(tmp_s * MPH_TO_KMH);

			/* Save the rest. */
			//skip = 7;
			rest = parse_remove_part(input, input_len, res-input, res-input+7, &rest_len);
		}
		else
		{
			rest = (char*)input;
			rest_len = input_len;
			rest[rest_len] = 0;
		}
	}
	else if (input_len > 0)
	{
		rest = (char*)input;
		rest_len = input_len;
		rest[rest_len] = 0;
	}

	/* Check if we still have something left. */
	if (rest_len >=9)
	{
		//char* res;
		/* Check for optional altitude anywhere in the comment, take the first occurrence. */
		res = strstr(rest, "/A=");
		if (res)
		{
			DEBUG_LOG("Found Alt : ");
			DEBUG_LOG(res);
			DEBUG_LOG("\n");
			/* Save altitude, if not already there. */
			if (!pb->altitude)
			{
				memcpy(altitude, res +3, 6);
				altitude[6] = 0;
				tmp_s = atoi(altitude);
				pb->altitude = (double)(tmp_s * FT_TO_M);
				pb->flags |= F_ALT;
			}

			/* Remove altitude. */
			rest = parse_remove_part(rest, rest_len, res - rest, res - rest + 9, &rest_len);
			//rest = (char*)res + 9;
			//rest_len = rest_len - 9;
			/*tmp_str = parse_remove_part(res, rest_len, input-res, input - res+9, &tmp_us);
			rest = tmp_str;
			rest_len = tmp_us;*/
		}
	}

	/* If we still hafe stuff left, check for !DAO!, take the last occurrence (per recommendation). */
	//if (rest_len > 0)
	//{
	//	for (i = rest_len - 1; i >= 0; --i)
	//	{
	//		if (i + 4 < rest_len && rest[i] == '!' &&
	//			0x21 <= rest[i + 1] && rest[i + 1] <= 0x7b &&
	//			0x20 <= rest[i + 2] && rest[i + 2] <= 0x7b &&
	//			0x20 <= rest[i + 3] && rest[i + 3] <= 0x7b &&
	//			rest[i + 4] == '!')
	//		{
	//			memcpy(dao, rest + i + 1, 3);
	//			/* Validate and save dao. */
	//			//if (fapint_parse_dao(packet, dao))
	//			//{
	//				/* Remove !DAO!. */
	//				tmp_str = parse_remove_part(rest, rest_len, i, i + 5, &tmp_us);
	//				//free(rest);
	//				rest = tmp_str;
	//				rest_len = tmp_us;
	//				break;
	//			//}
	//		}
	//	}
	//}

	/* Check for base-91 comment telemetry. */
	/*fapint_parse_comment_telemetry(packet, &rest, &rest_len);*/

	if (rest_len >=11)
	{
		/* Check for optional OGN ID anywhere in the comment, take the first occurrence. */
		res = strstr(rest, " id");
		if (res)
		{
			DEBUG_LOG("Found ID : ");
			DEBUG_LOG(res);
			DEBUG_LOG("\n");

			if (!pb->ogn_id)
			{
				memcpy(id, res +3, 8);
				id[8] = 0;
				pb->ogn_id = strtol(id, 0, 16);
				pb->flags |= F_OGNID;
			}

			/* Remove OGN ID. */
			rest = parse_remove_part(rest, rest_len, res - rest, res - rest + 11, &rest_len);
		}
	}

	/* If there's something left, save it as a comment. */
	rest_len=strlen(rest);
	if (rest_len > 0)
	{
		pb->comment = rest;
		pb->comment_len = rest_len;
		pb->comment[rest_len] = 0;
		DEBUG_LOG("Comment: ");
		DEBUG_LOG(rest);
	}
}

uint8_t ParseAPRS::pkgType(const char* raw) {
	uint8_t type = 0;
	char packettype = 0;
	const char* info_start,*body;
	int paclen = strlen(raw);
	info_start = (char*)strchr(raw, ':');
	if (info_start == NULL) return 0;
	packettype = *(info_start+1);
	body = info_start+2;
	
	switch (packettype) {
	case '=':
	case '/':
	case '@':
		if (strchr(body, 'r') != NULL) {
			if (strchr(body, 'g') != NULL) {
				if (strchr(body, 't') != NULL) {
					if (strchr(body, 'P') != NULL) {
						type = PKG_WX;
					}
				}
			}
		}
		break;
	case ':':
		type = PKG_MESSAGE;
		if (body[9] == ':' &&
			(memcmp(body + 9, ":PARM.", 6) == 0 ||
				memcmp(body + 9, ":UNIT.", 6) == 0 ||
				memcmp(body + 9, ":EQNS.", 6) == 0 ||
				memcmp(body + 9, ":BITS.", 6) == 0)) {
			type = PKG_TELEMETRY;
		}
		break;
	case '>':
		type = PKG_STATUS;
		break;
	case '?':
		type = PKG_QUERY;
		break;
	case ';':
		type = PKG_OBJECT;
		break;
	case ')':
		type = PKG_ITEM;
		break;
	case 'T':
		type = PKG_TELEMETRY;
		break;
	case '#': /* Peet Bros U-II Weather Station */
	case '*': /* Peet Bros U-I  Weather Station */
	case '_': /* Weather report without position */
		type = PKG_WX;
		break;
	default: type = 0;
		break;
	}
	return type;
}