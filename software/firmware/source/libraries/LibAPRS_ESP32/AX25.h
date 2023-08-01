#ifndef PROTOCOL_AX25_H
#define PROTOCOL_AX25_H

#if defined(ARDUINO) || defined(HACKRF_ONE)
#include <Arduino.h>
#endif /* ARDUINO */

#include <stdio.h>
#include <stdbool.h>

#define AX25_MIN_FRAME_LEN 18
#define AX25_MAX_INFO_LEN 255

#ifndef CUSTOM_FRAME_SIZE
    #define AX25_MAX_FRAME_LEN 330
#else
    #define AX25_MAX_FRAME_LEN CUSTOM_FRAME_SIZE
#endif

#define AX25_CRC_CORRECT  0xF0B8

#define AX25_CTRL_UI      0x03
#define AX25_PID_NOLAYER3 0xF0

struct AX25Ctx;     // Forward declarations
struct AX25Msg;

typedef void (*ax25_callback_t)(struct AX25Msg *msg);

typedef struct AX25Ctx {
    uint8_t buf[AX25_MAX_FRAME_LEN];
    FILE *ch;
    size_t frame_len;
    uint16_t crc_in;
    uint16_t crc_out;
    ax25_callback_t hook;
    bool sync;
    bool escape;
} AX25Ctx;

typedef struct ax25header_struct{   
//#if !defined(SOFTRF_SKETCH)
    char addr[7];
//#else
//    char addr[10];
//#endif
    char ssid;   
}ax25header;

typedef struct ax25frame_struct{   
    ax25header  header[10];
    char data[AX25_MAX_INFO_LEN];   
}ax25frame;

#define AX25_CALL(str, id) {.call = (str), .ssid = (id) }
#define AX25_MAX_RPT 8
#define AX25_REPEATED(msg, n) ((msg)->rpt_flags & BV(n))

//#if !defined(SOFTRF_SKETCH)
#define CALL_OVERSPACE 1
//#else
//#define CALL_OVERSPACE 4
//#endif

typedef struct AX25Call {
    char call[6+CALL_OVERSPACE];
    //char STRING_TERMINATION = 0;
    uint8_t ssid;
} AX25Call;

typedef struct AX25Msg {
    AX25Call src;
    AX25Call dst;
    AX25Call rpt_list[AX25_MAX_RPT];
    uint8_t  rpt_count;
    uint8_t  rpt_flags;
    uint16_t ctrl;
    uint8_t  pid;
    uint8_t info[AX25_MAX_INFO_LEN];
    size_t len;
} AX25Msg;

void ax25_sendVia(AX25Ctx *ctx, const AX25Call *path, size_t path_len, const void *_buf, size_t len);
#define ax25_send(ctx, dst, src, buf, len) ax25_sendVia(ctx, ({static AX25Call __path[]={dst, src}; __path;}), 2, buf, len)

void ax25_poll(AX25Ctx *ctx);
void ax25_sendRaw(AX25Ctx *ctx, void *_buf, size_t len);
void ax25_init(AX25Ctx *ctx, ax25_callback_t hook);
char ax25_encode(ax25frame &frame, char *txt,int size);
void ax25sendFrame(AX25Ctx *ctx,ax25frame *pkg);

#endif
