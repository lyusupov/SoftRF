#ifndef FLARM_CODEC

#define FLARM_CODEC

#define FLARM_KEY1 { 0xe43276df, 0xdca83759, 0x9802b8ac, 0x4675a56b }
#define FLARM_KEY2 0x045d9f3b
#define FLARM_KEY3 0x87b562f4

typedef struct {
    /********************/
    unsigned int addr:24;
    unsigned int magic:8;
    /********************/
    int vs:10;
    unsigned int _unk0:3;
    unsigned int stealth:1;
    unsigned int no_track:1;
    unsigned int _unk1:1;
    unsigned int gps:12;
    unsigned int type:4;
    /********************/
    unsigned int lat:19;
    unsigned int alt:13;
    /********************/
    unsigned int lon:20;
    unsigned int _unk2:10;
    unsigned int vsmult:2;
    /********************/
    int8_t ns[4];
    int8_t ew[4];
    /********************/
} flarm_packet;

char *flarm_decode(flarm_packet *pkt, float ref_lat, float ref_lon, int16_t ref_alt, double timestamp, float rssi, int16_t channel);

#endif
