/*
This is free and unencumbered software released into the public domain.

Anyone is free to copy, modify, publish, use, compile, sell, or
distribute this software, either in source code form or as a compiled
binary, for any purpose, commercial or non-commercial, and by any
means.

In jurisdictions that recognize copyright laws, the author or authors
of this software dedicate any and all copyright interest in the
software to the public domain. We make this dedication for the benefit
of the public at large and to the detriment of our heirs and
successors. We intend this dedication to be an overt act of
relinquishment in perpetuity of all present and future rights to this
software under copyright law.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.

For more information, please refer to <http://unlicense.org/>
*/

#ifndef RGPIOD_H
#define RGPIOD_H

#include <inttypes.h>

#include "lgCmd.h"

#define RGPIOD_VERSION 0x00010000

#define LG_ENVPORT "LG_PORT"
#define LG_ENVADDR "LG_ADDR"

#define LG_DEFAULT_USER "default"

#define LG_DEFAULT_IF_FLAGS           0
#define LG_DEFAULT_SOCKET_PORT        8889
#define LG_DEFAULT_SOCKET_PORT_STR    "8889"
#define LG_DEFAULT_SOCKET_ADDR_STR    "localhost"

#ifdef __cplusplus
extern "C" {
#endif

/* prototypes
*/

int lgExecCmd(lgCmd_p h, int bufSize);

/* port */

#define LG_MIN_SOCKET_PORT 1024
#define LG_MAX_SOCKET_PORT 32000

/* ifFlags: */

#define LG_LOCALHOST_SOCK_IF 4

/* Allowed socket connect addresses */

#define MAX_CONNECT_ADDRESSES 256

/* File API
*/

int lgFileOpen(char *file, int mode);
int lgFileClose(int handle);
int lgFileRead(int handle, char *buf, int count);
int lgFileWrite(int handle, char *buf, int count);
int lgFileSeek(int handle, int32_t seekOffset, int seekFrom);
int lgFileList(char *fpat,  char *buf, int count);

/* Script API
*/

int lgScriptStore(char *script);
int lgScriptRun(int handle, int count, uint32_t *scriptParam);
int lgScriptUpdate(int handle, int count, uint32_t *scriptParam);
int lgScriptStatus(int handle, uint32_t *scriptParam);
int lgScriptStop(int handle);
int lgScriptDelete(int handle);

int lgShell(char *scriptName, char *scriptString);

/* globals
*/

extern int gPermits;
extern int gNumSockNetAddr;
extern uint32_t gSockNetAddr[MAX_CONNECT_ADDRESSES];
extern int gFdSock;

#ifdef __cplusplus
}
#endif

/*DEF_S Socket Command Codes*/

#define LG_CMD_FO    1  // file open
#define LG_CMD_FC    2  // file close
#define LG_CMD_FR    3  // file read
#define LG_CMD_FW    4  // file write
#define LG_CMD_FS    5  // file seek
#define LG_CMD_FL    6  // file list

#define LG_CMD_GO    10 // gpiochip open
#define LG_CMD_GC    11 // gpiochip close

#define LG_CMD_GSIX  12 // gpio claim for input
#define LG_CMD_GSOX  13 // gpio claim for output
#define LG_CMD_GSAX  14 // gpio claim for alerts
#define LG_CMD_GSF   15 // gpio free

#define LG_CMD_GSGIX 16 // gpio group claim for input
#define LG_CMD_GSGOX 17 // gpio group claim for output
#define LG_CMD_GSGF  18 // gpio group free

#define LG_CMD_GR    19 // gpio read
#define LG_CMD_GW    20 // gpio write
#define LG_CMD_GGR   21 // gpio group read
#define LG_CMD_GGWX  22 // gpio group write

#define LG_CMD_GPX   23 // gpio software timed pulses
#define LG_CMD_PX    24 // gpio software timed PWM
#define LG_CMD_SX    25 // gpio software timed servo pulses
#define LG_CMD_GWAVE 26 // gpio software timed waves
#define LG_CMD_GBUSY 27 // tx busy
#define LG_CMD_GROOM 28 // tx room
#define LG_CMD_GDEB  29 // gpio set debounce time
#define LG_CMD_GWDOG 30 // gpio set watchdog time

#define LG_CMD_GIC   31 // gpiochip get chip info
#define LG_CMD_GIL   32 // gpiochip get line info
#define LG_CMD_GMODE 33 // gpio get mode

#define LG_CMD_I2CO  40 // I2C open
#define LG_CMD_I2CC  41 // I2C close
#define LG_CMD_I2CRD 42 // I2C read device
#define LG_CMD_I2CWD 43 // I2C write device
#define LG_CMD_I2CWQ 44 // SMBus Write Quick
#define LG_CMD_I2CRS 45 // SMBus Read Byte
#define LG_CMD_I2CWS 46 // SMBus Write Byte
#define LG_CMD_I2CRB 47 // SMBus Read Byte Data
#define LG_CMD_I2CWB 48 // SMBus Write Byte Data
#define LG_CMD_I2CRW 49 // SMBus Read Word
#define LG_CMD_I2CWW 50 // SMBus Write Word
#define LG_CMD_I2CRK 51 // SMBus Read Block Data
#define LG_CMD_I2CWK 52 // SMBus Write Block Data
#define LG_CMD_I2CRI 53 // SMBus Read I2C Block Data
#define LG_CMD_I2CWI 54 // SMBus Write I2C Block Data
#define LG_CMD_I2CPC 55 // SMBus Process Call
#define LG_CMD_I2CPK 56 // SMBus Block Process Call
#define LG_CMD_I2CZ  57 // I2C zip (multiple commands)

#define LG_CMD_NO    70 // notification open
#define LG_CMD_NC    71 // notification close
#define LG_CMD_NR    72 // notification resume
#define LG_CMD_NP    73 // notification pause

#define LG_CMD_PARSE 80 // script parse
#define LG_CMD_PROC  81 // script store
#define LG_CMD_PROCD 82 // script delete
#define LG_CMD_PROCP 83 // script status
#define LG_CMD_PROCR 84 // script run
#define LG_CMD_PROCS 85 // script stop
#define LG_CMD_PROCU 86 // script update parameters

#define LG_CMD_SERO  90 // serial open
#define LG_CMD_SERC  91 // serial close
#define LG_CMD_SERRB 92 // serial read byte
#define LG_CMD_SERWB 93 // serial write byte
#define LG_CMD_SERR  94 // serial read bytes
#define LG_CMD_SERW  95 // serial write bytes
#define LG_CMD_SERDA 96 // serial data available

#define LG_CMD_SPIO  100 // SPI open
#define LG_CMD_SPIC  101 // SPI close
#define LG_CMD_SPIR  102 // SPI read bytes
#define LG_CMD_SPIW  103 // SPI write bytes
#define LG_CMD_SPIX  104 // SPI transfer bytes

#define LG_CMD_MICS  113 // delay for a number of microseconds
#define LG_CMD_MILS  114 // delay for a number of milliseconds
#define LG_CMD_CGI   115 // get internals setting
#define LG_CMD_CSI   116 // set internals setting
#define LG_CMD_NOIB  117 // open a notification inband in a socket
#define LG_CMD_SHELL 118 // run a shell command

#define LG_CMD_SBC   120 // print the SBC's host name
#define LG_CMD_FREE  121 // release resources

#define LG_CMD_SHARE 130 // set the share id for handles
#define LG_CMD_USER  131 // set the user
#define LG_CMD_PASSW 132 // submit the password
#define LG_CMD_LCFG  133 // reload the permits file
#define LG_CMD_SHRU  134 // use this share to access handles
#define LG_CMD_SHRS  135 // set this share on created handles
#define LG_CMD_PWD   136 // print the daemon working directory
#define LG_CMD_PCD   137 // print the daemon configuration directory

#define LG_CMD_LGV   140 // print the lg library version
#define LG_CMD_TICK  141 // print the number of nanonseconds since the epoch

/*DEF_E*/

/*DEF_S Convenience Command Codes*/

#define LG_CMD_GGW   600 // simple GPIO group write
#define LG_CMD_GP    601 // simple GPIO tx pulses
#define LG_CMD_GSA   602 // simple GPIO claim for alerts
#define LG_CMD_GSGI  603 // simple GPIO group claim for inputs
#define LG_CMD_GSGO  604 // simple GPIO group claim for outputs
#define LG_CMD_GSI   605 // simple GPIO claim for input
#define LG_CMD_GSO   606 // simple GPIO claim for output
#define LG_CMD_P     607 // simple GPIO tx PWM
#define LG_CMD_S     608 // simple GPIO tx servo pulses

/*DEF_E*/

/*DEF_S Script Command Codes*/

#define LG_CMD_SCRIPT 800

#define LG_CMD_ADD   800
#define LG_CMD_AND   801
#define LG_CMD_CALL  802
#define LG_CMD_CMDR  803
#define LG_CMD_CMDW  804
#define LG_CMD_CMP   805
#define LG_CMD_DCR   806
#define LG_CMD_DCRA  807
#define LG_CMD_DIV   808
#define LG_CMD_HALT  809
#define LG_CMD_INR   810
#define LG_CMD_INRA  811
#define LG_CMD_JGE   812
#define LG_CMD_JGT   813
#define LG_CMD_JLE   814
#define LG_CMD_JLT   815
#define LG_CMD_JMP   816
#define LG_CMD_JNZ   817
#define LG_CMD_JZ    818
#define LG_CMD_TAG   819
#define LG_CMD_LD    820
#define LG_CMD_LDA   821
#define LG_CMD_LDAB  822
#define LG_CMD_MLT   823
#define LG_CMD_MOD   824
#define LG_CMD_NOP   825
#define LG_CMD_OR    826
#define LG_CMD_POP   827
#define LG_CMD_POPA  828
#define LG_CMD_PUSH  829
#define LG_CMD_PUSHA 830
#define LG_CMD_RET   831
#define LG_CMD_RL    832
#define LG_CMD_RLA   833
#define LG_CMD_RR    834
#define LG_CMD_RRA   835
#define LG_CMD_SHL   836
#define LG_CMD_SHLA  837
#define LG_CMD_SHR   838
#define LG_CMD_SHRA  839
#define LG_CMD_STA   840
#define LG_CMD_STAB  841
#define LG_CMD_SUB   842
#define LG_CMD_SYS   843
#define LG_CMD_WAIT  844
#define LG_CMD_X     845
#define LG_CMD_XA    846
#define LG_CMD_XOR   847
#define LG_CMD_EVTWT 848

/*DEF_E*/

#endif

