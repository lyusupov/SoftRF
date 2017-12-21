#ifndef __ADSB_ENCODER_INCLUDE__
#define __ADSB_ENCODER_INCLUDE__


enum DF { DF17  , DF18 , DF18ANON , DF18TRACK};


typedef struct  frame_data
{
	unsigned char msg[14];
}frame_data_t;


#define CPR_ODD		1
#define CPR_EVEN	0

#define Category_Set_D	1
#define Category_Set_C	2
#define Category_Set_B	3
#define Category_Set_A	4

#define AIR_POS			0
#define SURFACE_POS		1


/*
adsb编码初始化
*/
void adsb_encoder_init(); 


/*
生成空中位置报文
*/
frame_data_t  make_air_position_frame(
	unsigned short metype,  //[9,18] , [20,22]
	unsigned int addr,
	double lat, 
	double  lon,
	double alt, //ft
	unsigned int oddflag,
	DF df); 

/*
生成地面位置报文
*/
frame_data_t  make_surface_position_frame(
	unsigned short metype,   //[5,8]
	unsigned int addr,
	double lat, double  lon,
	unsigned int knot, //地面速度(knot)
	bool heading_valid,  //heading是否有效
	double heading, //速度方向(0~360)
	unsigned int oddflag, DF df); 


/*
生成身份类别报文
*/
frame_data_t make_aircraft_identification_frame(
	unsigned int addr,
	unsigned char callsign[8], //航班号，不足8字节的位置补0x00
	unsigned short category_set,
	unsigned short Category,
	DF df);

/*
生成速度(ground speed)报文
*/
frame_data_t make_velocity_frame(
	unsigned int addr,
	double nsvel,  //南北方向速度(kts ,北正向)
	double ewvel,  //东西方向速度(kts ,东正向)
	double vrate,  //升速(ft/min,上升正向)
	DF df); 


#endif
