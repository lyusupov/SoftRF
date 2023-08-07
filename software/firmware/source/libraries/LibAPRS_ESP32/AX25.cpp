// Based on work by Francesco Sacchi

#if defined(ARDUINO) || defined(HACKRF_ONE)
#include <Arduino.h>
#endif /* ARDUINO */

#if defined(RASPBERRY_PI)
#include <raspi/raspi.h>
#endif /* RASPBERRY_PI */

#include <string.h>
#include <stdlib.h>
#ifndef __AVR__
#include <strings.h>
#endif /* __AVR__ */
#include <ctype.h>
#include <stdio.h>
#include "AX25.h"
#include "HDLC.h"
#include "CRC-CCIT.h"
#include "AFSK.h"

size_t ctxbufflen;
uint8_t *ctxbuffer;

#ifndef _BV
#define _BV(bit) (1 << (bit))
#endif

#define typeof(x) __typeof__(x)
#define countof(a) sizeof(a) / sizeof(a[0])
#define MIN(a, b) (                          \
    {                                        \
        typeof(a) _a = (a);                  \
        typeof(b) _b = (b);                  \
        ((typeof(_a))((_a < _b) ? _a : _b)); \
    })
#define DECODE_CALL(buf, addr)                                     \
    for (unsigned i = 0; i < sizeof((addr)) - CALL_OVERSPACE; i++) \
    {                                                              \
        char c = (*(buf)++ >> 1);                                  \
        (addr)[i] = (c == ' ') ? '\x0' : c;                        \
    }
#define AX25_SET_REPEATED(msg, idx, val)   \
    do                                     \
    {                                      \
        if (val)                           \
        {                                  \
            (msg)->rpt_flags |= _BV(idx);  \
        }                                  \
        else                               \
        {                                  \
            (msg)->rpt_flags &= ~_BV(idx); \
        }                                  \
    } while (0)

extern int LibAPRS_vref;

static void ax25_putchar(AX25Ctx *ctx, uint8_t c);

void ax25_init(AX25Ctx *ctx, ax25_callback_t hook)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->hook = hook;
    ctx->crc_in = ctx->crc_out = CRC_CCIT_INIT_VAL;
}

static void ax25_decode(AX25Ctx *ctx)
{
    AX25Msg msg;
    ctxbufflen = ctx->frame_len - 2;
    ctxbuffer = ctx->buf;
    uint8_t *buf = ctx->buf;

    DECODE_CALL(buf, msg.dst.call);
    msg.dst.ssid = (*buf++ >> 1) & 0x0F;
    msg.dst.call[6] = 0;

    DECODE_CALL(buf, msg.src.call);
    msg.src.ssid = (*buf >> 1) & 0x0F;
    msg.src.call[6] = 0;

    for (msg.rpt_count = 0; !(*buf++ & 0x01) && (msg.rpt_count < countof(msg.rpt_list)); msg.rpt_count++)
    {
        DECODE_CALL(buf, msg.rpt_list[msg.rpt_count].call);
        msg.rpt_list[msg.rpt_count].ssid = (*buf >> 1) & 0x0F;
        AX25_SET_REPEATED(&msg, msg.rpt_count, (*buf & 0x80));
        msg.rpt_list[msg.rpt_count].call[6] = 0;
    }

    msg.ctrl = *buf++;
    if (msg.ctrl != AX25_CTRL_UI)
    {
        return;
    }

    msg.pid = *buf++;
    if (msg.pid != AX25_PID_NOLAYER3)
    {
        return;
    }

    memset(msg.info,0,sizeof(msg.info));
    msg.len = ctx->frame_len - 2 - (buf - ctx->buf);
    memcpy(msg.info,buf,msg.len);
    //msg.info[msg.len]=0;
    //msg.info = buf;

    if (ctx->hook)
    {
#if defined(ESP32)
        cli();
#endif
        ctx->hook(&msg);
#if defined(ESP32)
        sei();
#endif
    }
}

void ax25_poll(AX25Ctx *ctx)
{
    int c;

    while ((c = afsk_getchar()) != EOF)
    {
        if (!ctx->escape && c == HDLC_FLAG)
        {
            if (ctx->frame_len >= AX25_MIN_FRAME_LEN)
            {
                if (ctx->crc_in == AX25_CRC_CORRECT)
                {
                    ax25_decode(ctx);
                }
            }
            ctx->sync = true;
            ctx->crc_in = CRC_CCIT_INIT_VAL;

            ctx->frame_len = 0;
            continue;
        }

        if (!ctx->escape && c == HDLC_RESET)
        {
            ctx->sync = false;
            continue;
        }

        if (!ctx->escape && c == AX25_ESC)
        {
            ctx->escape = true;
            continue;
        }

        if (ctx->sync)
        {
            if (ctx->frame_len < AX25_MAX_FRAME_LEN)
            {
                ctx->buf[ctx->frame_len++] = c;
                ctx->crc_in = update_crc_ccit(c, ctx->crc_in);
            }
            else
            {
                ctx->sync = false;
            }
        }
        ctx->escape = false;
    }
}

static void ax25_putchar(AX25Ctx *ctx, uint8_t c)
{
    if (c == HDLC_FLAG || c == HDLC_RESET || c == AX25_ESC)
        afsk_putchar(AX25_ESC);
    ctx->crc_out = update_crc_ccit(c, ctx->crc_out);
    afsk_putchar(c);
}

void ax25_sendRaw(AX25Ctx *ctx, void *_buf, size_t len)
{
    ctx->crc_out = CRC_CCIT_INIT_VAL;
    afsk_putchar(HDLC_FLAG);
    const uint8_t *buf = (const uint8_t *)_buf;
    while (len--)
        ax25_putchar(ctx, *buf++);

    uint8_t crcl = (ctx->crc_out & 0xff) ^ 0xff;
    uint8_t crch = (ctx->crc_out >> 8) ^ 0xff;
    ax25_putchar(ctx, crcl);
    ax25_putchar(ctx, crch);

    afsk_putchar(HDLC_FLAG);
}

static void ax25_sendCall(AX25Ctx *ctx, const AX25Call *addr, bool last)
{
//#if !defined(SOFTRF_SKETCH)
    unsigned len = MIN((sizeof(addr->call) - CALL_OVERSPACE), strlen(addr->call));
//#else
//    unsigned len = strlen(addr->call);
//#endif
    for (unsigned i = 0; i < len; i++)
    {
        uint8_t c = addr->call[i];
        c = toupper(c);
        ax25_putchar(ctx, c << 1);
    }

    if (len < (sizeof(addr->call) - CALL_OVERSPACE))
    {
        for (unsigned i = 0; i < (sizeof(addr->call) - CALL_OVERSPACE) - len; i++)
        {
            ax25_putchar(ctx, ' ' << 1);
        }
    }

    uint8_t ssid = 0x60 | (addr->ssid << 1) | (last ? 0x01 : 0);
    ax25_putchar(ctx, ssid);
}

void ax25_sendVia(AX25Ctx *ctx, const AX25Call *path, size_t path_len, const void *_buf, size_t len)
{
    const uint8_t *buf = (const uint8_t *)_buf;

    ctx->crc_out = CRC_CCIT_INIT_VAL;
    afsk_putchar(HDLC_FLAG);

    for (size_t i = 0; i < path_len; i++)
    {
        ax25_sendCall(ctx, &path[i], (i == path_len - 1));
    }

    ax25_putchar(ctx, AX25_CTRL_UI);
    ax25_putchar(ctx, AX25_PID_NOLAYER3);

    while (len--)
    {
        ax25_putchar(ctx, *buf++);
    }

    uint8_t crcl = (ctx->crc_out & 0xff) ^ 0xff;
    uint8_t crch = (ctx->crc_out >> 8) ^ 0xff;
    ax25_putchar(ctx, crcl);
    ax25_putchar(ctx, crch);

    afsk_putchar(HDLC_FLAG);
}

unsigned int strpos(char *txt, char chk)
{
    char *pch;
    unsigned int idx = 0;
    pch = strchr(txt, chk);
    idx = pch - txt;
    return idx;
}

void convPath(ax25header *hdr, char *txt, unsigned int size)
{
    unsigned int i, p, j;
    char num[5];
    hdr->ssid = 0;
    memset(hdr->addr, 0, 7);
    memset(&num[0], 0, sizeof(num));

    p = strpos(txt, '-');
    if (p > 0 && p < size)
    {
        for (i = 0; i < p; i++)
        { // Get CallSign/Path
            hdr->addr[i] = txt[i];
        }
        j = 0;
        for (i = p + 1; i < size; i++)
        { // get SSID
            //            if(txt[i]=='*') break;
            //            if(txt[i]==',') break;
            //            if(txt[i]==':') break;
            if (txt[i] < 0x30)
                break;
            if (txt[i] > 0x39)
                break;
            num[j++] = txt[i];
        }
        if (j > 0)
        {
            hdr->ssid = atoi(num);
        }
        hdr->ssid <<= 1;
    }
    else
    {
        for (i = 0; i < size; i++)
        { // Get CallSign/Path
            if (txt[i] == '*')
                break;
            if (txt[i] == ',')
                break;
            if (txt[i] == ':')
                break;
            hdr->addr[i] = txt[i];
        }

        hdr->ssid = 0;
    }
    p = strpos(txt, '*');
    if (p > 0 && p < size)
        hdr->ssid |= 0x80;
    hdr->ssid |= 0x60;
}

char ax25_encode(ax25frame &frame, char *txt, int size)
{
    char *token, *ptr;
    int i;
    unsigned int p, p2, p3;
    char j;
    ptr = (char *)&frame;
    memset(ptr, 0, sizeof(ax25frame)); // Clear frame
    p = strpos(txt, ':');
    if (p > 0 && p < size)
    {
        // printf("p{:}=%d\r\n",p);
        // Get String APRS
        memset(&frame.data, 0, sizeof(frame.data));
        for (i = 0; i < (size - p); i++)
            frame.data[i] = txt[p + i + 1];
        p2 = strpos(txt, '>');
        if (p2 > 0 && p2 < size)
        {
            // printf("p2{>}=%d\r\n",p2);
            convPath(&frame.header[1], &txt[0], p2); // Get callsign src
            j = strpos(txt, ',');
            if ((j < 1) || (j > p))
                j = p;
            convPath(&frame.header[0], &txt[p2 + 1], j - p2 - 1); // Get callsign dest
                                                                  // if(j<p){
            p3 = 0;
            for (i = j; i < size; i++)
            { // copy path to origin
                if (txt[i] == ':')
                {
                    for (; i < size; i++)
                        txt[p3++] = 0x00;
                    break;
                }
                txt[p3++] = txt[i];
            }
            // printf("Path:%s\r\n",txt);
            token = strtok(txt, ",");
            j = 0;
            while (token != NULL)
            {
                ptr = token;
                convPath(&frame.header[j + 2], ptr, strlen(ptr));
                token = strtok(NULL, ",");
                j++;
                if (j > 7)
                    break;
            }

            for (i = 0; i < 10; i++)
                frame.header[i].ssid &= 0xFE; // Clear All END Path
            // Fix END path
            for (i = 2; i < 10; i++)
            {
                if (frame.header[i].addr[0] == 0x00)
                {
                    frame.header[i - 1].ssid |= 0x01;
                    break;
                }
            }
            // }
            return 1;
        }
    }
    return 0;
}

void ax25sendFrame(AX25Ctx *ctx, ax25frame *pkg)
{
    int i, j, c = 0;
    uint8_t data = 0;
    ctx->crc_out = CRC_CCIT_INIT_VAL;
    afsk_putchar(HDLC_FLAG);

    for (i = 0; i < 10; i++)
        pkg->header[i].ssid &= 0xFE; // Clear All END Path
    // Fix END path
    for (i = 1; i < 10; i++)
    {
        if (pkg->header[i].addr[0] == 0x00)
        {
            pkg->header[i - 1].ssid |= 0x01;
            break;
        }
    }

    for (i = 0; i < 10; i++)
    {
        if (pkg->header[i].addr[0] == 0)
            break;
        for (j = 0; j < 6; j++)
        {
            data = (uint8_t)pkg->header[i].addr[j];
            if (data == 0)
                data = 0x20;
            // putchar(data);
            data <<= 1;
            ax25_putchar(ctx, data);
            c++;
        }
        ax25_putchar(ctx, (uint8_t)pkg->header[i].ssid);
        if (pkg->header[i].ssid & 0x01)
            break;
    }

    ax25_putchar(ctx, AX25_CTRL_UI);      // Control field - 0x03 is APRS UI-frame
    ax25_putchar(ctx, AX25_PID_NOLAYER3); // Protocol ID - 0xF0 is no layer 3
                                          // ax25sendString(&pkg->data[0]);

    for (i = 0; i < strlen(pkg->data); i++)
    {
        ax25_putchar(ctx, (uint8_t)pkg->data[i]);
    }

    uint8_t crcl = (ctx->crc_out & 0xff) ^ 0xff;
    uint8_t crch = (ctx->crc_out >> 8) ^ 0xff;
    ax25_putchar(ctx, crcl);
    ax25_putchar(ctx, crch);

    afsk_putchar(HDLC_FLAG);
    return;
}
