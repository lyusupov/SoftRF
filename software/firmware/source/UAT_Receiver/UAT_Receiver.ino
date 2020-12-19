/*
 * UAT_Receiver(.ino) firmware
 * Copyright (C) 2019-2020 Linar Yusupov
 *
 * Author: Linar Yusupov, linar.r.yusupov@gmail.com
 *
 * Web: http://github.com/lyusupov/SoftRF
 *
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "src/system/SoC.h"
#include "src/system/Time.h"
#include "src/driver/LED.h"
#include "src/driver/RF.h"
#include "src/driver/EEPROM.h"
#include "src/protocol/data/GDL90.h"
#include "src/protocol/data/D1090.h"
#include "src/driver/Baro.h"
#include "src/driver/Battery.h"

#include "EasyLink.h"

//#define DEBUG_UAT

EasyLink_RxPacket rxPacket;
EasyLink myLink;

ufo_t ThisAircraft;

hardware_info_t hw_info = {
  .model    = DEFAULT_SOFTRF_MODEL,
  .revision = 0,
  .soc      = SOC_NONE,
  .rf       = RF_IC_NONE,
  .gnss     = GNSS_MODULE_NONE,
  .baro     = BARO_MODULE_NONE,
  .display  = DISPLAY_NONE
};

Stratux_frame_t LPUATRadio_frame = {
  .magic1     = STRATUX_UATRADIO_MAGIC_1,
  .magic2     = STRATUX_UATRADIO_MAGIC_2,
  .magic3     = STRATUX_UATRADIO_MAGIC_3,
  .magic4     = STRATUX_UATRADIO_MAGIC_4,

  .msgLen     = LONG_FRAME_BYTES,
  .rssi       = 0,
  .timestamp  = 0UL,
};

#if defined(DEBUG_UAT)
#include <xdc/std.h>

#include <xdc/runtime/System.h>
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/Types.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/utils/Load.h>
#include <ti/sysbios/hal/Hwi.h>

static void printUtilization()
{
    xdc_UInt i;
    Memory_Stats memStat;
    Hwi_StackInfo hwiStackStat;
    Load_Stat loadStat;
    Task_Handle tsk;
    float idleLoad;
    uint32_t idleLoadInt, idleLoadFrac;

    /* collect current stats */
    Load_update();

    /* use time NOT spent in idle task for Total CPU Load */
    Load_getTaskLoad(Task_getIdleTask(), &loadStat);
    idleLoad = 100.0 - 100.0*(float)loadStat.threadTime/(float)loadStat.totalTime;
    idleLoadInt = idleLoad;
    idleLoadFrac = 10.0*idleLoad - 10.0*idleLoadInt;

    Serial.write("Total CPU Load: ");
    Serial.print(idleLoadInt);
    Serial.print(".");
    Serial.println(idleLoadFrac);
    Serial.println("");
#if 0
    /* collect stats on all statically Created tasks */
    Serial.println("Task info:");
    for (i = 0; i < Task_Object_count(); i++) {
        tsk = Task_Object_get(NULL, i);
        printTaskInfo(tsk);
    }

    /* collect stats on all dynamically Created tasks */
    tsk = Task_Object_first();
    while (tsk) {
        printTaskInfo(tsk);
        tsk = Task_Object_next(tsk);
    }
    Serial.println("");
#endif
    Hwi_getStackInfo(&hwiStackStat, TRUE);
    Serial.print(F("Hwi stack usage: "));
    Serial.print(hwiStackStat.hwiStackPeak);
    Serial.print("/");
    Serial.println(hwiStackStat.hwiStackSize);
    Serial.println("");

    Memory_getStats(NULL, &memStat);
    Serial.print(F("Heap usage: "));
    Serial.print(memStat.totalSize - memStat.totalFreeSize);
    Serial.print("/");
    Serial.println(memStat.totalSize);
}
#endif /* DEBUG_UAT */

static bool UAT_Receive_Sync()
{
  // rxTimeout is in Radio time and needs to be converted from miliseconds to Radio Time
  rxPacket.rxTimeout = EasyLink_ms_To_RadioTime(2000);

  // Turn the receiver on immediately
  rxPacket.absTime = EasyLink_ms_To_RadioTime(0);

  EasyLink_Status status = myLink.receive(&rxPacket);

  if (status == EasyLink_Status_Success) {

#if defined(DEBUG_UAT)
    Serial.print("Packet received with length ");
    Serial.print(rxPacket.len);
    Serial.print(" RSSI ");
    Serial.print(rxPacket.rssi);
#if 0
    Serial.println(" and value:");
    Serial.println(Bin2Hex((byte *) rxPacket.payload, rxPacket.len));
#else
    Serial.println();
#endif
#endif

    rx_packets_counter++;

    return(true);
  } else {
#if defined(DEBUG_UAT)
    Serial.print("Error receiving packet with status code: ");
    Serial.print(status);
    Serial.print(" (");
    Serial.print(myLink.getStatusString(status));
    Serial.println(")");
#endif
    return(false);
  }
}

static bool UAT_receive_complete  = false;
static bool UAT_receive_active    = false;

void UAT_Receive_callback(EasyLink_RxPacket *rxPacket_ptr, EasyLink_Status status)
{
  UAT_receive_active = false;

  if (status == EasyLink_Status_Success) {
    memcpy(&rxPacket, rxPacket_ptr, sizeof(rxPacket));
    UAT_receive_complete  = true;
  }
}

static bool UAT_Receive_Async()
{
  bool success = false;
  EasyLink_Status status;

  if (!UAT_receive_active) {
    status = myLink.receive(&UAT_Receive_callback);

    if (status == EasyLink_Status_Success) {
      UAT_receive_active = true;
    }
  }

  if (UAT_receive_complete == true) {

    success = true;
    UAT_receive_complete = false;

    rx_packets_counter++;
  }

  return success;
}

#define UAT_Receive UAT_Receive_Async

void setup() {
  hw_info.soc = SoC_setup(); // Has to be very first procedure in the execution order

  Serial.begin(UAT_RECEIVER_BR, SERIAL_OUT_BITS);

  Serial.println();
  Serial.print(F(SOFTRF_IDENT));
  Serial.print(SoC->name);
  Serial.print(F(" FW.REV: " SOFTRF_FIRMWARE_VERSION " DEV.ID: "));
  Serial.println(String(SoC->getChipId(), HEX));
  Serial.println(F("Copyright (C) 2015-2020 Linar Yusupov. All rights reserved."));
  Serial.flush();

  EEPROM_setup();

  settings->mode        = SOFTRF_MODE_RECEIVER;
  settings->rf_protocol = RF_PROTOCOL_ADSB_UAT;

  SoC->Button_setup();

  ThisAircraft.addr = SoC->getChipId() & 0x00FFFFFF;

  hw_info.display = SoC->Display_setup();

  Battery_setup();

  LED_setup();

  /*
   * Display 'U' (UAT) on OLED for Rx only modes.
   */
  ThisAircraft.protocol = RF_PROTOCOL_ADSB_UAT ;

  Serial.println("Receiver mode.");

  if (hw_info.display != DISPLAY_NONE)  delay(3000);

  myLink.begin(EasyLink_Phy_Custom);
  Serial.println("Listening...");

  hw_info.rf = RF_IC_CC13XX;

  SoC->post_init();

  SoC->WDT_setup();
}

void loop() {

  bool success = UAT_Receive();

  if (success) {
    LPUATRadio_frame.timestamp  = now();
    LPUATRadio_frame.rssi       = rxPacket.rssi;
    LPUATRadio_frame.msgLen     = rxPacket.len;

    memcpy(LPUATRadio_frame.data, rxPacket.payload, rxPacket.len);

    Serial.write((uint8_t *) &LPUATRadio_frame, sizeof(LPUATRadio_frame));
  }

  // Show status info on tiny OLED display
  SoC->Display_loop();

  // battery status LED
  LED_loop();

  SoC->loop();

  Battery_loop();

  SoC->Button_loop();

  yield();
}

void shutdown(int reason)
{
  SoC->WDT_fini();

  SoC->Display_fini(reason);

  SoC->Button_fini();

  SoC_fini(reason);
}
