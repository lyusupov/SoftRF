
#include <math.h>
#include <string.h>
#include "adsb_encoder.h"


#define latz	(15.0)
#ifndef M_PI 
#define M_PI       3.14159265358979323846   // pi
#endif

#define MODES_GENERATOR_POLY 0xfff409U

#if !defined(ESP8266) && !defined(ESP32) && \
    !defined(ENERGIA_ARCH_CC13XX) && !defined(ENERGIA_ARCH_CC13X2) && \
    !defined(__ASR6501__) && !defined(ARDUINO_ARCH_STM32) && \
    !defined(ARDUINO_ARCH_ASR650X) && !defined(ARDUINO_ARCH_RENESAS) && \
    !defined(ARDUINO_ARCH_SILABS)
static unsigned int crc_table[256];
#else
#if defined(ESP8266) || defined(ESP32) || defined(__ASR6501__) || \
    defined(ARDUINO_ARCH_ASR650X)
#include <pgmspace.h>
#endif

#if defined(ENERGIA_ARCH_CC13XX) || defined(ENERGIA_ARCH_CC13X2)  || \
    defined(ARDUINO_ARCH_STM32)  || defined(ARDUINO_ARCH_RENESAS) || \
    defined(ARDUINO_ARCH_SILABS)
#include <avr/pgmspace.h>
#endif

static const unsigned int crc_table[256] PROGMEM = 
{
 0x000000, 0xFFF409, 0x001C1B, 0xFFE812, 0x003836, 0xFFCC3F, 0x00242D, 0xFFD024,
 0x00706C, 0xFF8465, 0x006C77, 0xFF987E, 0x00485A, 0xFFBC53, 0x005441, 0xFFA048,
 0x00E0D8, 0xFF14D1, 0x00FCC3, 0xFF08CA, 0x00D8EE, 0xFF2CE7, 0x00C4F5, 0xFF30FC,
 0x0090B4, 0xFF64BD, 0x008CAF, 0xFF78A6, 0x00A882, 0xFF5C8B, 0x00B499, 0xFF4090,
 0x01C1B0, 0xFE35B9, 0x01DDAB, 0xFE29A2, 0x01F986, 0xFE0D8F, 0x01E59D, 0xFE1194,
 0x01B1DC, 0xFE45D5, 0x01ADC7, 0xFE59CE, 0x0189EA, 0xFE7DE3, 0x0195F1, 0xFE61F8,
 0x012168, 0xFED561, 0x013D73, 0xFEC97A, 0x01195E, 0xFEED57, 0x010545, 0xFEF14C,
 0x015104, 0xFEA50D, 0x014D1F, 0xFEB916, 0x016932, 0xFE9D3B, 0x017529, 0xFE8120,
 0x038360, 0xFC7769, 0x039F7B, 0xFC6B72, 0x03BB56, 0xFC4F5F, 0x03A74D, 0xFC5344,
 0x03F30C, 0xFC0705, 0x03EF17, 0xFC1B1E, 0x03CB3A, 0xFC3F33, 0x03D721, 0xFC2328,
 0x0363B8, 0xFC97B1, 0x037FA3, 0xFC8BAA, 0x035B8E, 0xFCAF87, 0x034795, 0xFCB39C,
 0x0313D4, 0xFCE7DD, 0x030FCF, 0xFCFBC6, 0x032BE2, 0xFCDFEB, 0x0337F9, 0xFCC3F0,
 0x0242D0, 0xFDB6D9, 0x025ECB, 0xFDAAC2, 0x027AE6, 0xFD8EEF, 0x0266FD, 0xFD92F4,
 0x0232BC, 0xFDC6B5, 0x022EA7, 0xFDDAAE, 0x020A8A, 0xFDFE83, 0x021691, 0xFDE298,
 0x02A208, 0xFD5601, 0x02BE13, 0xFD4A1A, 0x029A3E, 0xFD6E37, 0x028625, 0xFD722C,
 0x02D264, 0xFD266D, 0x02CE7F, 0xFD3A76, 0x02EA52, 0xFD1E5B, 0x02F649, 0xFD0240,
 0x0706C0, 0xF8F2C9, 0x071ADB, 0xF8EED2, 0x073EF6, 0xF8CAFF, 0x0722ED, 0xF8D6E4,
 0x0776AC, 0xF882A5, 0x076AB7, 0xF89EBE, 0x074E9A, 0xF8BA93, 0x075281, 0xF8A688,
 0x07E618, 0xF81211, 0x07FA03, 0xF80E0A, 0x07DE2E, 0xF82A27, 0x07C235, 0xF8363C,
 0x079674, 0xF8627D, 0x078A6F, 0xF87E66, 0x07AE42, 0xF85A4B, 0x07B259, 0xF84650,
 0x06C770, 0xF93379, 0x06DB6B, 0xF92F62, 0x06FF46, 0xF90B4F, 0x06E35D, 0xF91754,
 0x06B71C, 0xF94315, 0x06AB07, 0xF95F0E, 0x068F2A, 0xF97B23, 0x069331, 0xF96738,
 0x0627A8, 0xF9D3A1, 0x063BB3, 0xF9CFBA, 0x061F9E, 0xF9EB97, 0x060385, 0xF9F78C,
 0x0657C4, 0xF9A3CD, 0x064BDF, 0xF9BFD6, 0x066FF2, 0xF99BFB, 0x0673E9, 0xF987E0,
 0x0485A0, 0xFB71A9, 0x0499BB, 0xFB6DB2, 0x04BD96, 0xFB499F, 0x04A18D, 0xFB5584,
 0x04F5CC, 0xFB01C5, 0x04E9D7, 0xFB1DDE, 0x04CDFA, 0xFB39F3, 0x04D1E1, 0xFB25E8,
 0x046578, 0xFB9171, 0x047963, 0xFB8D6A, 0x045D4E, 0xFBA947, 0x044155, 0xFBB55C,
 0x041514, 0xFBE11D, 0x04090F, 0xFBFD06, 0x042D22, 0xFBD92B, 0x043139, 0xFBC530,
 0x054410, 0xFAB019, 0x05580B, 0xFAAC02, 0x057C26, 0xFA882F, 0x05603D, 0xFA9434,
 0x05347C, 0xFAC075, 0x052867, 0xFADC6E, 0x050C4A, 0xFAF843, 0x051051, 0xFAE458,
 0x05A4C8, 0xFA50C1, 0x05B8D3, 0xFA4CDA, 0x059CFE, 0xFA68F7, 0x0580E5, 0xFA74EC,
 0x05D4A4, 0xFA20AD, 0x05C8BF, 0xFA3CB6, 0x05EC92, 0xFA189B, 0x05F089, 0xFA0480
};
#endif

typedef struct cpr_pair
{
	unsigned int YZ;
	unsigned int XZ;
}cpr_pair_t;

unsigned int modes_crc(unsigned char *buf, size_t  len)
{
	unsigned int rem = 0;
	size_t  i;
	for (rem = 0, i = len; i > 0; --i) {
#if !defined(ESP8266) && !defined(ESP32) && \
    !defined(ENERGIA_ARCH_CC13XX) && !defined(ENERGIA_ARCH_CC13X2) && \
    !defined(__ASR6501__) && !defined(ARDUINO_ARCH_STM32) && \
    !defined(ARDUINO_ARCH_ASR650X) && !defined(ARDUINO_ARCH_RENESAS) && \
    !defined(ARDUINO_ARCH_SILABS)
		rem = ((rem & 0x00ffff) << 8) ^ crc_table[*buf++ ^ ((rem & 0xff0000) >> 16)];
#else
		rem = ((rem & 0x00ffff) << 8) ^ pgm_read_dword(&crc_table[*buf++ ^ ((rem & 0xff0000) >> 16)]);
#endif
	}

	return rem;
}


int    CPR_NL(double lat)
{
#if 0 
	if (lat < 0)
		lat =  -lat;

	if (fabs(lat) >= 87.0)
		return 1;

	double U = 1 - cos(M_PI / (2 * latz));
	double T = cos(M_PI / 180.0 * fabs(lat));
	return static_cast<int>(floor(2.0*M_PI* 1.0 / (acos(1 - U / (T*T)))));
#endif

	if (lat < 0) lat = -lat;
	if (lat < 10.47047130) return 59;
	if (lat < 14.82817437) return 58;
	if (lat < 18.18626357) return 57;
	if (lat < 21.02939493) return 56;
	if (lat < 23.54504487) return 55;
	if (lat < 25.82924707) return 54;
	if (lat < 27.93898710) return 53;
	if (lat < 29.91135686) return 52;
	if (lat < 31.77209708) return 51;
	if (lat < 33.53993436) return 50;
	if (lat < 35.22899598) return 49;
	if (lat < 36.85025108) return 48;
	if (lat < 38.41241892) return 47;
	if (lat < 39.92256684) return 46;
	if (lat < 41.38651832) return 45;
	if (lat < 42.80914012) return 44;
	if (lat < 44.19454951) return 43;
	if (lat < 45.54626723) return 42;
	if (lat < 46.86733252) return 41;
	if (lat < 48.16039128) return 40;
	if (lat < 49.42776439) return 39;
	if (lat < 50.67150166) return 38;
	if (lat < 51.89342469) return 37;
	if (lat < 53.09516153) return 36;
	if (lat < 54.27817472) return 35;
	if (lat < 55.44378444) return 34;
	if (lat < 56.59318756) return 33;
	if (lat < 57.72747354) return 32;
	if (lat < 58.84763776) return 31;
	if (lat < 59.95459277) return 30;
	if (lat < 61.04917774) return 29;
	if (lat < 62.13216659) return 28;
	if (lat < 63.20427479) return 27;
	if (lat < 64.26616523) return 26;
	if (lat < 65.31845310) return 25;
	if (lat < 66.36171008) return 24;
	if (lat < 67.39646774) return 23;
	if (lat < 68.42322022) return 22;
	if (lat < 69.44242631) return 21;
	if (lat < 70.45451075) return 20;
	if (lat < 71.45986473) return 19;
	if (lat < 72.45884545) return 18;
	if (lat < 73.45177442) return 17;
	if (lat < 74.43893416) return 16;
	if (lat < 75.42056257) return 15;
	if (lat < 76.39684391) return 14;
	if (lat < 77.36789461) return 13;
	if (lat < 78.33374083) return 12;
	if (lat < 79.29428225) return 11;
	if (lat < 80.24923213) return 10;
	if (lat < 81.19801349) return 9;
	if (lat < 82.13956981) return 8;
	if (lat < 83.07199445) return 7;
	if (lat < 83.99173563) return 6;
	if (lat < 84.89166191) return 5;
	if (lat < 85.75541621) return 4;
	if (lat < 86.53536998) return 3;
	if (lat < 87.00000000) return 2;
	else return 1;
	
}

int CPR_N(double lat, int odd)
{
	int nl = CPR_NL(lat) - (odd ? 1 : 0);
	if (nl < 1)
		nl = 1;
	return nl;

}
double CPR_MOD(double x, double y)
{
	return x - y*floor(x / y);
}

double  CPR_DLAT(int odd, int surface)
{
	double tmp = 0;

	if (surface == 1)
		tmp = 90.0;
	else
		tmp = 360.0;
	double  nzcalc = (odd ? 59.0 : 60.0);
	if (nzcalc == 0)
		return tmp;
	else
		return tmp / nzcalc;
}
double  CPR_DLON(double rlat, int odd, int surface)
{
	double tmp = 0;

	if (surface == 1)
		tmp = 90.0;
	else
		tmp = 360.0;
	double  nzcalc = CPR_N(rlat, odd);
	if (nzcalc == 0)
		return tmp;
	else
		return tmp / nzcalc;
}



cpr_pair_t cpr_encode(double lat, double lon, int odd, int surface)
{
	double NbPow = 0;
	if (surface)
		NbPow = pow(2.0, 19);
	else
		NbPow = pow(2.0, 17); 
	double Dlat = CPR_DLAT(odd, 0);
	unsigned int YZ = static_cast<unsigned int>(floor(NbPow*CPR_MOD(lat, Dlat) / Dlat + 0.5));
	double Rlat = Dlat*(1.0*YZ / NbPow + floor(lat / Dlat));
	double Dlon = CPR_DLON(Rlat, odd, 0);
	unsigned int XZ = static_cast<unsigned int>(floor(NbPow*CPR_MOD(lon, Dlon) / Dlon + 0.5));

	cpr_pair_t v;
	v.YZ = YZ & 0x1FFFF;
	v.XZ = XZ & 0x1FFFF;
	return v;
}

unsigned int encode_altitude(double ft)
{
	unsigned int i = static_cast<unsigned int>((ft + 1012.5) / 25);
	if (i < 0)
		i = 0;
	if (i > 0x7FF)
		i = 0x7FF;
	return ((i & 0x7F0) << 1) | 0x010 | (i & 0x00F);
}

unsigned char surface_movement(unsigned int knot)
{
	if (knot > 175)
		return 124;
	else if (knot > 100)
		return static_cast<unsigned char>((knot - 100) / 5.0 + 109);
	else if (knot > 70)
		return static_cast<unsigned char>((knot - 70) / 2.0 + 94);
	else if (knot > 15)
		return static_cast<unsigned char>((knot - 15) / 1.0 + 39);
	else if (knot > 2)
		return static_cast<unsigned char>((knot - 2) / 0.5 + 13);
	else if (knot > 1)
		return static_cast<unsigned char>((knot - 1) / 0.25 + 9);
	else if (knot > 0.125)
		return static_cast<unsigned char>((knot - 0.125) / 0.2700833 + 3);
	else if (knot > 0)
		return 2;
	else if (knot == 0)
		return 1;
	else
		return 0;
}

unsigned char surface_heading(double heading)
{
	double n = 360.0 / 128.0;
	return static_cast<unsigned char>(heading / n);
}


frame_data_t _make_surface_position_frame(
	unsigned short metype,
	unsigned int addr,
	unsigned int elat, unsigned int elon,
	unsigned char  knot, bool heading_valid, unsigned char heading,
	unsigned int oddflag, DF df)
{
	frame_data_t framev;
	memset((void*)&framev, 0, sizeof(frame_data_t));
	unsigned char* frame = framev.msg;

	unsigned char imf = 0;
	if (df == DF17)
	{
		frame[0] = (17 << 3) | (6);
		imf = 0;
	}
	else if (df == DF18)
	{
		frame[0] = (18 << 3) | (2);
		imf = 0;
	}
	else if (df == DF18ANON)
	{
		frame[0] = (18 << 3) | (5);
		imf = 0;
	}
	else
	{
		frame[0] = (18 << 3) | (2);
		imf = 1;
	}


	frame[1] = (addr >> 16) & 255;
	frame[2] = (addr >> 8) & 255;
	frame[3] = addr & 255;

	frame[4] = (metype << 3);

	frame[4] |= ((knot >> 4) & 0x07);
	frame[5] = knot << 4;

	
	if (heading_valid)
	{
		frame[5] |= 0x08;
		frame[5] |= (heading >> 4) & 0x07;
		frame[6] = (heading << 4)&0xF0;
	}

	if (imf)
		frame[6] |= 0x08; 

	if (oddflag)
		frame[6] |= 4;

	frame[6] |= (elat >> 15) & 3;
	frame[7] = (elat >> 7) & 255;
	frame[8] = (elat & 127) << 1;
	frame[8] |= (elon >> 16) & 1;
	frame[9] = (elon >> 8) & 255;
	frame[10] = elon & 255;

	unsigned int  	crc = modes_crc(frame, 11);
	frame[11] = (crc >> 16) & 255;
	frame[12] = (crc >> 8) & 255;
	frame[13] = crc & 255;
	return  framev;
}




frame_data_t _make_air_position_frame(unsigned short metype, unsigned int addr,
	unsigned int elat, unsigned int elon, unsigned int ealt,
	unsigned int oddflag, DF df)
{
	frame_data_t framev;
	memset((void*)&framev, 0, sizeof(frame_data_t));
	unsigned char* frame = framev.msg;

	unsigned char imf = 0;
	if (df == DF17)
	{
		frame[0] = (17 << 3) | (6);
		imf = 0;
	}
	else if (df == DF18)
	{
		frame[0] = (18 << 3) | (2);
		imf = 0;
	}
	else if (df == DF18ANON)
	{
		frame[0] = (18 << 3) | (5);
		imf = 0;
	}
	else
	{
		frame[0] = (18 << 3) | (2);
		imf = 1;
	}

	frame[1] = (addr >> 16) & 255;
	frame[2] = (addr >> 8) & 255;
	frame[3] = addr & 255;
	frame[4] = (metype << 3);
	frame[4] |= imf;
	frame[5] = (ealt >> 4) & 255;
	frame[6] = (ealt & 15) << 4;

	if (oddflag)
		frame[6] |= 4;

	frame[6] |= (elat >> 15) & 3;
	frame[7] = (elat >> 7) & 255;
	frame[8] = (elat & 127) << 1;
	frame[8] |= (elon >> 16) & 1;
	frame[9] = (elon >> 8) & 255;
	frame[10] = elon & 255;

	unsigned int  	crc = modes_crc(frame, 11);
	frame[11] = (crc >> 16) & 255;
	frame[12] = (crc >> 8) & 255;
	frame[13] = crc & 255;
	return  framev;
}


frame_data_t  make_air_position_frame(unsigned short metype,  //[5,18] , [20,22]
	unsigned int addr,
	double lat, double  lon,
	double alt, //ft
	unsigned int oddflag, DF df)
{

	unsigned int ealt = encode_altitude(alt);
	cpr_pair_t cpr_value = cpr_encode(lat, lon, oddflag, AIR_POS);

	return _make_air_position_frame(metype, addr, cpr_value.YZ, cpr_value.XZ, ealt, oddflag, df);
}


frame_data_t  make_surface_position_frame(
	unsigned short metype,   //[5,8]
	unsigned int addr,
	double lat, double  lon,
	unsigned int knot, bool heading_valid, double heading,
	unsigned int oddflag, DF df)
{
	cpr_pair_t cpr_value = cpr_encode(lat, lon, oddflag, SURFACE_POS);

	unsigned char mv_knot = surface_movement(knot);
	unsigned char mv_heading = surface_heading(heading);

	return _make_surface_position_frame(metype, 
		addr, cpr_value.YZ, cpr_value.XZ,
		mv_knot, heading_valid, mv_heading,
		oddflag, df);
}


unsigned short encode_vrate(double vr)
{
	unsigned short signbit = 0;
	if (vr < 0)
	{
		signbit = 0x200;
		vr = 0 - vr;
	}
	unsigned short vr_s = static_cast<unsigned short>(vr / 64.0 + 1.5);

	if (vr_s > 511)
		return 511 | signbit;
	else
		return vr_s | signbit;
}



unsigned short  encode_velocity(double kts, bool supersonic)
{
	unsigned short signbit = 0;
	if (kts < 0)
	{
		signbit = 0x400;
		kts = 0 - kts;
	}

	if (supersonic)
		kts /= 4.0;

	unsigned short kts_s = static_cast<unsigned short>(kts + 1.5);

	if (kts_s > 1023)
		return 1023 | signbit;
	else
		return kts_s | signbit;
}
static char *ais_charset = "@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_ !\"#$%&'()*+,-./0123456789:;<=>?";
unsigned char ais_charset_idx(unsigned char c)
{
	size_t len = strlen(ais_charset);
	for (size_t i = 0; i < len; i++)
	{
		if (ais_charset[i] == c)
			return static_cast<unsigned char>(i & 0xFF);
	}

	return 0;
}

frame_data_t make_aircraft_identification_frame(unsigned int addr,
	unsigned char callsign[8],
	unsigned short category_set,
	unsigned short Category,
	DF df)
{
	frame_data_t framev;
	memset((void*)&framev, 0, sizeof(frame_data_t));
	unsigned char* frame = framev.msg;

	unsigned char c1, c2, c3, c4, c5, c6, c7, c8;

	if (df == DF17)
		frame[0] = (17 << 3) | (6);
	else if (df == DF18)
		frame[0] = (18 << 3) | (2);
	else if (df == DF18ANON)
		frame[0] = (18 << 3) | (5);
	else
		frame[0] = (18 << 3) | (2);

	frame[1] = (addr >> 16) & 255;
	frame[2] = (addr >> 8) & 255;
	frame[3] = addr & 255;
	frame[4] = (category_set << 3);
	frame[4] |= (Category & 0x07);


#define GET_CHAR(n) \
		c = callsign[n]; \
		if (c == 0) \
			goto calc_crc;


	unsigned char c = 0;

	GET_CHAR(0);
	c1 = ais_charset_idx(c);
	frame[5] = c1 << 2;

	GET_CHAR(1);
	c2 = ais_charset_idx(c);
	frame[5] |= ((c2 >> 4) & 0x03);
	frame[6] = (c2 << 4);

	GET_CHAR(2);
	c3 = ais_charset_idx(c);
	frame[6] |= ((c3 >> 2) & 0x0F);
	frame[7] = c3 << 6;

	GET_CHAR(3);
	c4 = ais_charset_idx(c);
	frame[7] |= c4 & 0x3F;

	GET_CHAR(4);
	c5 = ais_charset_idx(c);
	frame[8] = c5 << 2;

	GET_CHAR(5);
	c6 = ais_charset_idx(c);
	frame[8] |= ((c6 >> 4) & 0x03);
	frame[9] = (c6 << 4);

	GET_CHAR(6);
	c7 = ais_charset_idx(c);
	frame[9] |= ((c7 >> 2) & 0x0F);
	frame[10] = c7 << 6;

	GET_CHAR(7);
	c8 = ais_charset_idx(c);
	frame[10] |= c8 & 0x3F;


calc_crc:
	unsigned int  	crc = modes_crc(frame, 11);
	frame[11] = (crc >> 16) & 255;
	frame[12] = (crc >> 8) & 255;
	frame[13] = crc & 255;
	return  framev;

}

frame_data_t make_velocity_frame(unsigned int addr,
	double nsvel,  //南北方向速度(kts ,北正向)
	double ewvel,  //东西方向速度(kts ,东正向)
	double vrate,  //升速(ft/min,上升正向)
	DF df)
{
	frame_data_t framev;
	memset((void*)&framev, 0, sizeof(frame_data_t));
	unsigned char* frame = framev.msg;

	bool 	supersonic = (fabs(nsvel) > 1000) || (fabs(ewvel) > 1000);
	unsigned short e_ns = encode_velocity(nsvel, supersonic);
	unsigned short	e_ew = encode_velocity(ewvel, supersonic);
	unsigned short	e_vr = encode_vrate(vrate);

	unsigned char imf = 0;
	if (df == DF17)
	{
		frame[0] = (17 << 3) | (6);
		imf = 0;
	}
	else if (df == DF18)
	{
		frame[0] = (18 << 3) | (2);
		imf = 0;
	}
	else if (df == DF18ANON)
	{
		frame[0] = (18 << 3) | (5);
		imf = 0;
	}
	else
	{
		frame[0] = (18 << 3) | (2);
		imf = 1;
	}


	frame[1] = (addr >> 16) & 255;
	frame[2] = (addr >> 8) & 255;
	frame[3] = addr & 255;
	frame[4] = (19 << 3);

	if (supersonic)
		frame[4] |= 2;
	else
		frame[4] |= 1;

	frame[5] = (imf << 7);
	frame[5] |= (e_ew >> 8) & 7;
	frame[6] = (e_ew & 255);
	frame[7] = (e_ns >> 3) & 255;
	frame[8] = (e_ns & 7) << 5;
	frame[8] |= 16;
	frame[8] |= (e_vr >> 6) & 15;
	frame[9] = (e_vr & 63) << 2;
	frame[10] = 0;


	unsigned int  	crc = modes_crc(frame, 11);
	frame[11] = (crc >> 16) & 255;
	frame[12] = (crc >> 8) & 255;
	frame[13] = crc & 255;
	return  framev;
}


int modescrc_module_init()
{
#if !defined(ESP8266) && !defined(ESP32) && \
    !defined(ENERGIA_ARCH_CC13XX) && !defined(ENERGIA_ARCH_CC13X2) && \
    !defined(__ASR6501__) && !defined(ARDUINO_ARCH_STM32) && \
    !defined(ARDUINO_ARCH_ASR650X) && !defined(ARDUINO_ARCH_RENESAS) && \
    !defined(ARDUINO_ARCH_SILABS)
	int i;

	for (i = 0; i < 256; ++i) {
		unsigned int  c = i << 16;
		int j;
		for (j = 0; j < 8; ++j) {
			if (c & 0x800000)
				c = (c << 1) ^ MODES_GENERATOR_POLY;
			else
				c = (c << 1);
		}

		crc_table[i] = c & 0x00ffffff;
	}
#endif
	return 0;
}


void adsb_encoder_init()
{
	modescrc_module_init();
}
