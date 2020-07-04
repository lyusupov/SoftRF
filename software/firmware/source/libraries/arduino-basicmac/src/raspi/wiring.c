/*
  Arduino.c -  Registers initialization for Raspberry Pi
  Copyright (c) 2015 Hristo Gochkov.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifdef RASPBERRY_PI

#define ARDUINO_MAIN
#include <errno.h>
#include "Arduino.h"
#include "raspberry_pi_revision.h"

static inline void _halt(uint32_t microseconds){
  uint32_t start = STCLO;
  uint32_t compare = start + microseconds;
  if(compare < start)
    while(STCLO - compare);
  else
    while(STCLO < compare);
}

void sleepMicroseconds(uint32_t m){
  usleep(m);
}

void delay(uint32_t m){
  while(m--) usleep(1000);
}

void delayMicroseconds(uint32_t m){
  if(m < 1000)
    return _halt(m);
  usleep(m);
}

void analogReference(uint8_t mode){}
int analogRead(uint8_t pin){ return 0; }

/*
 * CORE INIT AND CLOSE
 *
 * */

volatile uint32_t        *bcmreg_st;
volatile uint32_t        *bcmreg_irq;
volatile uint32_t        *bcmreg_pm;
volatile uint32_t        *bcmreg_cm;
volatile uint32_t        *bcmreg_gpio;
volatile uint32_t        *bcmreg_uart0;
volatile uint32_t        *bcmreg_pcm;
volatile uint32_t        *bcmreg_spi0;
volatile uint32_t        *bcmreg_bsc0;
volatile uint32_t        *bcmreg_pwm;
volatile uint32_t        *bcmreg_bscs;
volatile uint32_t        *bcmreg_aux;
volatile uint32_t        *bcmreg_bsc1;

static void *mapreg(int fd, off_t off){
    return mmap(NULL, 4096, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, off);
}

static void unmapreg(void **pmem){
    if (*pmem == MAP_FAILED) return;
    munmap(*pmem, 4096);
    *pmem = MAP_FAILED;
}

static void unmap_registers(void){
    unmapreg((void**) &bcmreg_st);
    unmapreg((void**) &bcmreg_irq);
    unmapreg((void**) &bcmreg_pm);
    unmapreg((void**) &bcmreg_cm);
    unmapreg((void**) &bcmreg_gpio);
    unmapreg((void**) &bcmreg_uart0);
    unmapreg((void**) &bcmreg_pcm);
    unmapreg((void**) &bcmreg_spi0);
    unmapreg((void**) &bcmreg_bsc0);
    unmapreg((void**) &bcmreg_pwm);
    unmapreg((void**) &bcmreg_bscs);
    unmapreg((void**) &bcmreg_aux);
    unmapreg((void**) &bcmreg_bsc1);
}

static int map_registers(uint32_t reg_offset){
    int memfd = -1;
    if ((memfd = open("/dev/mem", O_RDWR | O_SYNC) ) < 0) goto exit;

    bcmreg_st = (volatile uint32_t *)mapreg(memfd, BCM2835_ST_BASE + reg_offset);
    if(bcmreg_st == MAP_FAILED) goto exit;

    bcmreg_irq = (volatile uint32_t *)mapreg(memfd, BCM2835_IRQ_BASE + reg_offset);
    if(bcmreg_irq == MAP_FAILED) goto exit;

    bcmreg_pm = (volatile uint32_t *)mapreg(memfd, BCM2835_PM_BASE + reg_offset);
    if(bcmreg_pm == MAP_FAILED) goto exit;

    bcmreg_cm = (volatile uint32_t *)mapreg(memfd, BCM2835_CM_BASE + reg_offset);
    if(bcmreg_cm == MAP_FAILED) goto exit;

    bcmreg_gpio = (volatile uint32_t *)mapreg(memfd, BCM2835_GPIO_BASE + reg_offset);
    if(bcmreg_gpio == MAP_FAILED) goto exit;

    bcmreg_uart0 = (volatile uint32_t *)mapreg(memfd, BCM2835_UART0_BASE + reg_offset);
    if(bcmreg_uart0 == MAP_FAILED) goto exit;

    bcmreg_pcm = (volatile uint32_t *)mapreg(memfd, BCM2835_PCM_BASE + reg_offset);
    if(bcmreg_pcm == MAP_FAILED) goto exit;

    bcmreg_spi0 = (volatile uint32_t *)mapreg(memfd, BCM2835_SPI0_BASE + reg_offset);
    if(bcmreg_spi0 == MAP_FAILED) goto exit;

    bcmreg_bsc0 = (volatile uint32_t *)mapreg(memfd, BCM2835_BSC0_BASE + reg_offset);
    if(bcmreg_bsc0 == MAP_FAILED) goto exit;

    bcmreg_pwm = (volatile uint32_t *)mapreg(memfd, BCM2835_PWM_BASE + reg_offset);
    if(bcmreg_pwm == MAP_FAILED) goto exit;

    bcmreg_bscs = (volatile uint32_t *)mapreg(memfd, BCM2835_BSCS_BASE + reg_offset);
    if(bcmreg_bscs == MAP_FAILED) goto exit;

    bcmreg_aux = (volatile uint32_t *)mapreg(memfd, BCM2835_AUX_BASE + reg_offset);
    if(bcmreg_aux == MAP_FAILED) goto exit;

    bcmreg_bsc1 = (volatile uint32_t *)mapreg(memfd, BCM2835_BSC1_BASE + reg_offset);
    if(bcmreg_bsc1 == MAP_FAILED) goto exit;

    return 0;

exit:
    if (errno == EACCES) {
      fprintf(stderr, "No permissions to access /dev/mem.  You should probably run as root.\n");
    } else {
      fprintf(stderr, "init_registers failed: %s\n", strerror(errno));
    }
    if (memfd >= 0){
      close(memfd);
      unmap_registers();
    }

    return 1;
}

static uint32_t _board_revision = 0;

static void init_pins(){
  uint32_t pinmask = rpi_model_pinmasks[_board_revision];
  int i;
  for(i=0;i<32;i++){
    if((pinmask & (1 << i))){
      pinMode(i,INPUT);
    }
  }
}

void uninit(){
  init_pins();
  unmap_registers();
}

/**
 * @return Return 0 on success and not 1 on failure.
 */
int init(){
  RASPBERRY_PI_INFO_T info;
  getRaspberryPiInformation(&info);

  uint32_t offset = info.peripheralBase - 0x20000000;

  if (info.model == RPI_MODEL_B_PI_3 || info.model == RPI_MODEL_B_PI_2 || info.model == RPI_MODEL_ZERO || info.model == RPI_MODEL_ZERO_W) {
    info.revisionNumber = 0x10;
  }
  if(info.revisionNumber >= PINMASKS_LEN || !rpi_model_pinmasks[info.revisionNumber]){
    fprintf(stderr, "UNKNOWN_REVISION: 0x%08X\n", info.revisionNumber);
    return 1;
  }
  if(map_registers(offset))
    return 1;
  
  _board_revision = info.revisionNumber;
  init_pins();
  srand(time(NULL));
  return 0;
}

#endif // RASPBERRY_PI
