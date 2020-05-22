/// \addtogroup module_scif_driver_setup
//@{
#include "scif.h"
#include "scif_framework.h"
#undef DEVICE_FAMILY_PATH
#if defined(DEVICE_FAMILY)
    #define DEVICE_FAMILY_PATH(x) <ti/devices/DEVICE_FAMILY/x>
#elif defined(DeviceFamily_CC26X0)
    #define DEVICE_FAMILY_PATH(x) <ti/devices/cc26x0/x>
#elif defined(DeviceFamily_CC26X0R2)
    #define DEVICE_FAMILY_PATH(x) <ti/devices/cc26x0r2/x>
#elif defined(DeviceFamily_CC13X0)
    #define DEVICE_FAMILY_PATH(x) <ti/devices/cc13x0/x>
#else
    #define DEVICE_FAMILY_PATH(x) <x>
#endif
#include DEVICE_FAMILY_PATH(inc/hw_types.h)
#include DEVICE_FAMILY_PATH(inc/hw_memmap.h)
#include DEVICE_FAMILY_PATH(inc/hw_aon_event.h)
#include DEVICE_FAMILY_PATH(inc/hw_aon_rtc.h)
#include DEVICE_FAMILY_PATH(inc/hw_aon_wuc.h)
#include DEVICE_FAMILY_PATH(inc/hw_aux_sce.h)
#include DEVICE_FAMILY_PATH(inc/hw_aux_smph.h)
#include DEVICE_FAMILY_PATH(inc/hw_aux_evctl.h)
#include DEVICE_FAMILY_PATH(inc/hw_aux_aiodio.h)
#include DEVICE_FAMILY_PATH(inc/hw_aux_timer.h)
#include DEVICE_FAMILY_PATH(inc/hw_aux_wuc.h)
#include DEVICE_FAMILY_PATH(inc/hw_event.h)
#include DEVICE_FAMILY_PATH(inc/hw_ints.h)
#include DEVICE_FAMILY_PATH(inc/hw_ioc.h)
#include <string.h>
#if defined(__IAR_SYSTEMS_ICC__)
    #include <intrinsics.h>
#endif


// OSAL function prototypes
uint32_t scifOsalEnterCriticalSection(void);
void scifOsalLeaveCriticalSection(uint32_t key);




/// Firmware image to be uploaded to the AUX RAM
static const uint16_t pAuxRamImage[] = {
    /*0x0000*/ 0x1408, 0x040C, 0x1408, 0x042C, 0x1408, 0x0447, 0x1408, 0x044D, 0x4436, 0x2437, 0xAEFE, 0xADB7, 0x6442, 0x7000, 0x7C6B, 0x6872, 
    /*0x0020*/ 0x0068, 0x1425, 0x6873, 0x0069, 0x1425, 0x6874, 0x006A, 0x1425, 0x786B, 0xF801, 0xFA01, 0xBEF2, 0x7870, 0x6872, 0xFD0E, 0x6874, 
    /*0x0040*/ 0xED92, 0xFD06, 0x7C70, 0x642D, 0x0450, 0x786B, 0x8F1F, 0xED8F, 0xEC01, 0xBE01, 0xADB7, 0x8DB7, 0x6630, 0x6542, 0x0000, 0x1870, 
    /*0x0060*/ 0x9D88, 0x9C01, 0xB60D, 0x1067, 0xAF19, 0xAA00, 0xB609, 0xA8FF, 0xAF39, 0xBE06, 0x0C6B, 0x8869, 0x8F08, 0xFD47, 0x9DB7, 0x086B, 
    /*0x0080*/ 0x8801, 0x8A01, 0xBEEC, 0x262F, 0xAEFE, 0x4630, 0x0450, 0x5527, 0x6642, 0x0000, 0x0C6B, 0x140B, 0x0450, 0x6742, 0x03FF, 0x0C6D, 
    /*0x00A0*/ 0x786C, 0x686D, 0xED37, 0xB605, 0x0000, 0x0C6C, 0x7C71, 0x652D, 0x0C6D, 0x786D, 0x686E, 0xFD0E, 0xF801, 0xE92B, 0xFD0E, 0xBE01, 
    /*0x00C0*/ 0x6436, 0xBDB7, 0x241A, 0xA6FE, 0xADB7, 0x641A, 0xADB7, 0x0000, 0x0104, 0x0105, 0x0107, 0x0000, 0x0000, 0xFFFF, 0x0000, 0x0000, 
    /*0x00E0*/ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0016, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 
    /*0x0100*/ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 
    /*0x0120*/ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 
    /*0x0140*/ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 
    /*0x0160*/ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 
    /*0x0180*/ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 
    /*0x01A0*/ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 
    /*0x01C0*/ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 
    /*0x01E0*/ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 
    /*0x0200*/ 0x0000, 0x0000, 0x0000, 0x0000, 0xADB7, 0x1508, 0xADB7, 0xADB7, 0x5000, 0x2879, 0x0122, 0x0E1D, 0x014B, 0x0E1E, 0x01BC, 0x0E1F, 
    /*0x0220*/ 0x8602, 0x0206, 0x0E20, 0x8602, 0x0211, 0x0E21, 0x0A1E, 0xCDB1, 0x9DB7, 0x0A20, 0xCDB1, 0x9DB7, 0x0A1E, 0xCDB1, 0x9DB7, 0x0A21, 
    /*0x0240*/ 0xCDB1, 0x8DB7, 0x6902, 0x1903, 0x9D2E, 0xB604, 0x987A, 0xEF09, 0x112B, 0x1E1D, 0xADB7, 0x8600, 0xEAFF, 0x9E03, 0x86FF, 0xE800, 
    /*0x0260*/ 0x8E04, 0x550E, 0x7008, 0x1136, 0x1E1D, 0xADB7, 0xEC01, 0xBE02, 0x550E, 0x8E01, 0x750E, 0xEDA9, 0xF8FF, 0xBE02, 0x1141, 0x1E1D, 
    /*0x0280*/ 0xADB7, 0x1122, 0x1E1D, 0x750E, 0x1903, 0x9801, 0x9A40, 0xAE01, 0x1000, 0x1D03, 0xADB7, 0xDA00, 0x9E09, 0xD8FF, 0xBE07, 0x8608, 
    /*0x02A0*/ 0xC200, 0xCF2B, 0x18FA, 0x9202, 0x1CFA, 0xADB7, 0x08FE, 0x18FF, 0x8D29, 0x9E0E, 0xAA00, 0xB605, 0xA8FF, 0x3612, 0xAE01, 0x2879, 
    /*0x02C0*/ 0xADB7, 0x8602, 0x1209, 0x1E20, 0x8602, 0x1214, 0x1E21, 0x0CFF, 0xADB7, 0x8602, 0x0206, 0x0E20, 0x8602, 0x0211, 0x0E21, 0x014B, 
    /*0x02E0*/ 0x0E1E, 0x5000, 0x5CFF, 0x2879, 0xADB7, 0x8602, 0x1206, 0x1E20, 0x8602, 0x1211, 0x1E21, 0x5008, 0x4000, 0x3900, 0x8600, 0xB8BA, 
    /*0x0300*/ 0x1183, 0x1E1E, 0xADB7, 0xCDA9, 0x1187, 0x1E1E, 0xADB7, 0xFD47, 0x3612, 0xA601, 0xC280, 0xD8FF, 0xBE02, 0x1192, 0x8E01, 0x1183, 
    /*0x0320*/ 0x1E1E, 0xADB7, 0x5900, 0xD801, 0xDA40, 0xAE01, 0x5000, 0x019A, 0x0E1E, 0xADB7, 0xFD47, 0x3612, 0xAE0B, 0xCCFF, 0xBE03, 0x8602, 
    /*0x0340*/ 0xC200, 0x8E02, 0x8601, 0xC200, 0x18FA, 0x9204, 0x1CFA, 0xADB7, 0x014B, 0x0E1E, 0x8602, 0x0209, 0x0E20, 0x8602, 0x0214, 0x0E21, 
    /*0x0360*/ 0x0901, 0xDD28, 0xBE05, 0x8604, 0xC200, 0xCF2B, 0x5878, 0xADB7, 0xCF2B, 0x5D00, 0x5878, 0xADB7, 0x18FD, 0x9A00, 0xB601, 0xADB7, 
    /*0x0380*/ 0x08FE, 0x18FF, 0x8D29, 0xA602, 0x0169, 0x0E1E, 0x01C9, 0x0E1F, 0x0516, 0x0875, 0x0CFC, 0x18FA, 0x9D00, 0x9006, 0x1CFA, 0x01D2, 
    /*0x03A0*/ 0x0E1F, 0x0516, 0x0900, 0x1901, 0x8D19, 0xD601, 0x8840, 0x0E1C, 0x01DB, 0x0E1F, 0x0516, 0x0A1C, 0x1876, 0x8D29, 0xAE03, 0x08FA, 
    /*0x03C0*/ 0x8201, 0x0CFA, 0x01E5, 0x0E1F, 0x0516, 0x0902, 0x1903, 0x8D19, 0xD601, 0x8840, 0x0E1C, 0x01EE, 0x0E1F, 0x0516, 0x0A1C, 0x1877, 
    /*0x03E0*/ 0x8D29, 0x9603, 0x08FA, 0x8208, 0x0CFA, 0x01F8, 0x0E1F, 0x0516, 0x08FB, 0x8A00, 0xBE08, 0x08FA, 0x18FC, 0x8D01, 0xB604, 0x0CFB, 
    /*0x0400*/ 0x652D, 0x0000, 0x0CFA, 0x01BC, 0x0E1F, 0x0516, 0x0A1D, 0xFD47, 0x8E03, 0x0A1D, 0x3612, 0xA602, 0xFD47, 0x8E02, 0x1175, 0x1E1E, 
    /*0x0420*/ 0x8DB7, 0x0A1F, 0xFD47, 0x8E03, 0x0A1F, 0x3612, 0xA602, 0xFD47, 0x8E02, 0x1175, 0x1E1E, 0x8DB7, 0x0000, 0x0000, 0x0000, 0x0000, 
    /*0x0440*/ 0x0000, 0x0000
};


/// Look-up table that converts from AUX I/O index to MCU IOCFG offset
static const uint8_t pAuxIoIndexToMcuIocfgOffsetLut[] = {
    120, 116, 112, 108, 104, 100, 96, 92, 28, 24, 20, 16, 12, 8, 4, 0
};


/** \brief Look-up table of data structure information for each task
  *
  * There is one entry per data structure (\c cfg, \c input, \c output and \c state) per task:
  * - [31:20] Data structure size (number of 16-bit words)
  * - [19:12] Buffer count (when 2+, first data structure is preceded by buffering control variables)
  * - [11:0] Address of the first data structure
  */
static const uint32_t pScifTaskDataStructInfoLut[] = {
//  cfg         input       output      state       
    0x005010EA, 0x040010F4, 0x04001174, 0x00A011F4  // UART Emulator
};




// No run-time logging task data structure signatures needed in this project




// No task-specific initialization functions




// No task-specific uninitialization functions




/** \brief Initilializes task resource hardware dependencies
  *
  * This function is called by the internal driver initialization function, \ref scifInit().
  */
static void scifTaskResourceInit(void) {
    scifInitIo(5, AUXIOMODE_OUTPUT, 1, 1);
    scifInitIo(6, AUXIOMODE_INPUT,  1, 0);
} // scifTaskResourceInit




/** \brief Uninitilializes task resource hardware dependencies
  *
  * This function is called by the internal driver uninitialization function, \ref scifUninit().
  */
static void scifTaskResourceUninit(void) {
    scifUninitIo(5, 1);
    scifUninitIo(6, 1);
} // scifTaskResourceUninit




/** \brief Re-initializes I/O pins used by the specified tasks
  *
  * It is possible to stop a Sensor Controller task and let the System CPU borrow and operate its I/O
  * pins. For example, the Sensor Controller can operate an SPI interface in one application state while
  * the System CPU with SSI operates the SPI interface in another application state.
  *
  * This function must be called before \ref scifExecuteTasksOnceNbl() or \ref scifStartTasksNbl() if
  * I/O pins belonging to Sensor Controller tasks have been borrowed System CPU peripherals.
  *
  * \param[in]      bvTaskIds
  *     Bit-vector of task IDs for the task I/Os to be re-initialized
  */
void scifReinitTaskIo(uint32_t bvTaskIds) {
    if (bvTaskIds & (1 << SCIF_UART_EMULATOR_TASK_ID)) {
        scifReinitIo(5, 1, 0);
        scifReinitIo(6, 1, 0);
    }
} // scifReinitTaskIo




/// Driver setup data, to be used in the call to \ref scifInit()
const SCIF_DATA_T scifDriverSetup = {
    (volatile SCIF_INT_DATA_T*) 0x400E00D6,
    (volatile SCIF_TASK_CTRL_T*) 0x400E00E0,
    (volatile uint16_t*) 0x400E00CE,
    0x0000,
    sizeof(pAuxRamImage),
    pAuxRamImage,
    pScifTaskDataStructInfoLut,
    pAuxIoIndexToMcuIocfgOffsetLut,
    scifTaskResourceInit,
    scifTaskResourceUninit,
    (volatile uint16_t*) NULL,
    (volatile uint16_t*) NULL,
    NULL
};




// Register aliases (CC13x2/CC26x2 names are used in the source code)
#ifndef AUX_TIMER01_BASE
    #define AUX_TIMER01_BASE            AUX_TIMER_BASE
#endif
#ifndef AUX_TIMER01_O_T0CTL
    #define AUX_TIMER01_O_T0CTL         AUX_TIMER_O_T0CTL
#endif
#ifndef AUX_TIMER01_O_T0CFG
    #define AUX_TIMER01_O_T0CFG         AUX_TIMER_O_T0CFG
#endif
#ifndef AUX_TIMER01_O_T0TARGET
    #define AUX_TIMER01_O_T0TARGET      AUX_TIMER_O_T0TARGET
#endif
#ifndef AUX_TIMER01_T0CFG_PRE_S
    #define AUX_TIMER01_T0CFG_PRE_S     AUX_TIMER_T0CFG_PRE_S
#endif
#ifndef AUX_TIMER01_T0CFG_RELOAD_M
    #define AUX_TIMER01_T0CFG_RELOAD_M  AUX_TIMER_T0CFG_RELOAD_M
#endif




/** \brief Sets the RX byte timeout
  *
  * The RX timeout starts once a byte has been received. The last received byte is flagged (see
  * \ref scifUartRxGetChar()) when the UART RX line has been idle for the specified period of time.
  *
  * \note This function must only be called while the UART receiver is disabled!
  *
  * \param[in]      rxTimeout
  *     Timeout in half bit periods
  */
void scifUartSetRxTimeout(uint16_t rxTimeout) {
    scifTaskData.uartEmulator.cfg.rxByteTimeout = rxTimeout;
} // scifUartSetRxTimeout




/** \brief Sets the number of required idle bit periods when enabling the UART receiver
  *
  * To prevent enabling RX in the middle of a byte and receiving garbage data, the required idle count
  * should be set to 20 (or more). After calling \c scifUartRxEnable(1), start bit detection is enabled
  * once the RX line has been idle (high) for the specified period of time
  *
  * \note This function must only be called while the UART receiver is disabled!
  *
  * \param[in]      rxEnableReqIdleCount
  *     Required idle time in number of half bit periods
  */
void scifUartSetRxEnableReqIdleCount(uint16_t rxEnableReqIdleCount) {
    scifTaskData.uartEmulator.cfg.rxEnableReqIdleCount = rxEnableReqIdleCount;
} // scifUartSetRxEnableReqIdleCount




/** \brief Enables or disables the UART receiver
  *
  * When enabling, the UART begins detecting start bit after the configurable required idle period (see
  * \ref scifUartSetRxEnableReqIdleCount()).
  *
  * When disabling, the UART receiver will be deactivated within one bit-period. Any ongoing byte
  * reception is aborted. Note that this also prevents RX timeout for the last received byte.
  *
  * \param[in]      rxEnable
  *     True to enable RX, false to disable RX
  */
void scifUartRxEnable(uint16_t rxEnable) {
    scifTaskData.uartEmulator.state.rxEnable = rxEnable;
} // scifUartRxEnable




/** \brief Stops the UART emulator
  *
  * When running the UART emulation task, the execution code runs continuously, and can only be stopped
  * by calling this function. This stops the UART emulator within one bit period.
  *
  * \note This function must only be called while the UART baud generator is active, and baud rate
  *       generation must not be disabled until the stop operation has taken effect.
  */
void scifUartStopEmulator(void) {
    scifTaskData.uartEmulator.state.exit = 1;
} // scifUartStopEmulator




/** \brief Sets the UART baud rate
  *
  * This function must be called to start baud rate generation before or after starting the UART
  * emulation task. This function can be called during operation to change the baud rate on-the-fly.
  *
  * \param[in]      baudRate
  *     The new baud rate (for example 115200 or 9600)
  */
void scifUartSetBaudRate(uint32_t baudRate) {
    uint32_t t0Period;
    uint32_t t0PrescalerExp;

    // Start baud rate generation?
    if (baudRate) {

        // Calculate the AUX timer 0 period
        t0Period = 24000000 / (4 * baudRate);

        // The period must be 65535 clock cycles or less, so adjust the prescaler until it is
        t0PrescalerExp = 0;
        while (t0Period > 65535) {
            t0PrescalerExp += 1;
            t0Period >>= 1;
        }

        // Stop baud rate generation while reconfiguring
        HWREG(AUX_TIMER01_BASE + AUX_TIMER01_O_T0CTL) = 0;

        // Set period and prescaler, and select reload mode
        HWREG(AUX_TIMER01_BASE + AUX_TIMER01_O_T0CFG) = (t0PrescalerExp << AUX_TIMER01_T0CFG_PRE_S) | AUX_TIMER01_T0CFG_RELOAD_M;
        HWREG(AUX_TIMER01_BASE + AUX_TIMER01_O_T0TARGET) = t0Period;

        // Start baud rate generation
        HWREG(AUX_TIMER01_BASE + AUX_TIMER01_O_T0CTL) = 1;

    // Baud rate 0 -> stop baud rate generation
    } else {
        HWREG(AUX_TIMER01_BASE + AUX_TIMER01_O_T0CTL) = 0;
    }

} // scifUartSetBaudRate




/** \brief Sets ALERT event threshold for the RX FIFO
  *
  * \param[in]      threshold
  *     Number of bytes (or higher) in the RX FIFO that triggers ALERT interrupt
  */
void scifUartSetRxFifoThr(uint16_t threshold) {
    scifTaskData.uartEmulator.cfg.alertRxFifoThr = threshold;
} // scifUartSetRxFifoThr




/** \brief Calculates the number of bytes currently stored in the RX FIFO
  *
  * Note that the RX FIFO can only store the configured number of characters minus one. This number is
  * defined by \ref SCIF_UART_RX_FIFO_MAX_COUNT.
  *
  * \return
  *     The number of characters in the RX FIFO, ready to be read
  */
uint32_t scifUartGetRxFifoCount(void) {
    uint16_t rxTail = scifTaskData.uartEmulator.state.rxTail;
    uint16_t rxHead = scifTaskData.uartEmulator.state.rxHead;
    if (rxHead < rxTail) {
        rxHead += sizeof(scifTaskData.uartEmulator.output.pRxBuffer) / sizeof(uint16_t);
    }
    return rxHead - rxTail;
} // scifUartGetRxFifoCount




/** \brief Receives a single character, with flags
  *
  * This function must only be called when there is at least one character in the RX FIFO. Calling this
  * function when the FIFO is empty will cause underflow, without warning.
  *
  * \return
  *     The received character, with flags:
  *     - [15:12] - Reserved
  *     - [11] - RX timeout occurred after receiving this character
  *     - [10] - RX FIFO ran full when receiving this character (this character may be invalid)
  *     - [9] - Break occurred when receiving this character
  *     - [8] - Framing error occurred when receiving this character
  *     - [7:0] - The character value
  */
uint16_t scifUartRxGetChar(void) {

    // Get the character
    uint32_t rxTail = scifTaskData.uartEmulator.state.rxTail;
    uint16_t c = scifTaskData.uartEmulator.output.pRxBuffer[rxTail];

    // Update the TX FIFO head index
    if (++rxTail == (sizeof(scifTaskData.uartEmulator.output.pRxBuffer) / sizeof(uint16_t))) {
        rxTail = 0;
    }
    scifTaskData.uartEmulator.state.rxTail = (uint16_t) rxTail;

    return c;
} // scifUartRxGetChar




/** \brief Receives the specified number of character, without flags
  *
  * This function must only be called with count equal to or less than the current RX FIFO count. Calling
  * this function with too high count will cause underflow, without warning.
  *
  * \param[in,out]  *pBuffer
  *     Pointer to the character destination buffer
  * \param[in]      count
  *     Number of characters to get
  */
void scifUartRxGetChars(char* pBuffer, uint32_t count) {
    int n;

    // For each character ...
    uint32_t rxTail = scifTaskData.uartEmulator.state.rxTail;
    for (n = 0; n < count; n++) {

        // Get it
        pBuffer[n] = (char) scifTaskData.uartEmulator.output.pRxBuffer[rxTail];

        // Update the TX FIFO head index
        if (++rxTail == (sizeof(scifTaskData.uartEmulator.output.pRxBuffer) / sizeof(uint16_t))) {
            rxTail = 0;
        }
    }
    scifTaskData.uartEmulator.state.rxTail = (uint16_t) rxTail;

} // scifUartRxGetChars




/** \brief Receives the specified number of character, with flags
  *
  * This function must only be called with count equal to or less than the current RX FIFO count. Calling
  * this function with too high count will cause underflow, without warning.
  *
  * \param[in,out]  *pBuffer
  *     Pointer to the character destination buffer. Each entry contains the following
  *     - [15:12] - Reserved
  *     - [11] - RX timeout occurred after receiving this character
  *     - [10] - RX FIFO ran full when receiving this character (this character may be invalid)
  *     - [9] - Break occurred when receiving this character
  *     - [8] - Framing error occurred when receiving this character
  *     - [7:0] - The character value
  * \param[in]      count
  *     Number of characters to get
  */
void scifUartRxGetCharsWithFlags(uint16_t* pBuffer, uint32_t count) {
    int n;

    // For each character ...
    uint32_t rxTail = scifTaskData.uartEmulator.state.rxTail;
    for (n = 0; n < count; n++) {

        // Get it
        pBuffer[n] = scifTaskData.uartEmulator.output.pRxBuffer[rxTail];

        // Update the TX FIFO head index
        if (++rxTail == (sizeof(scifTaskData.uartEmulator.output.pRxBuffer) / sizeof(uint16_t))) {
            rxTail = 0;
        }
    }
    scifTaskData.uartEmulator.state.rxTail = (uint16_t) rxTail;

} // scifUartRxGetCharsWithFlags




/** \brief Sets ALERT event threshold for the TX FIFO
  *
  * \param[in]      threshold
  *     Number of bytes (or lower) in the TX FIFO that triggers ALERT interrupt
  */
void scifUartSetTxFifoThr(uint16_t threshold) {
    scifTaskData.uartEmulator.cfg.alertTxFifoThr = threshold;
} // scifUartSetTxFifoThr




/** \brief Calculates the number of bytes currently stored in the TX FIFO
  *
  * The count is decremented when a character's stop bits are transmitted.
  *
  * Note that the TX FIFO can only store the configured number of characters minus one. This number is
  * defined by \ref SCIF_UART_TX_FIFO_MAX_COUNT.
  *
  * \return
  *     The number of characters in the TX FIFO, waiting to be transmitted
  */
uint32_t scifUartGetTxFifoCount(void) {
    uint16_t txTail = scifTaskData.uartEmulator.state.txTail;
    uint16_t txHead = scifTaskData.uartEmulator.state.txHead;
    if (txHead < txTail) {
        txHead += sizeof(scifTaskData.uartEmulator.input.pTxBuffer) / sizeof(uint16_t);
    }
    return txHead - txTail;
} // scifUartGetTxFifoCount




/** \brief Transmits a single character, without delay
  *
  * This function must not be called when the TX FIFO is full. Calling this function when the FIFO is
  * full will cause overflow, without warning. The number of free entries in the FIFO is:
  * \code
  * SCIF_UART_TX_FIFO_MAX_COUNT - scifUartGetTxFifoCount()
  * \endcode
  *
  * \param[in]      c
  *     The character to transmit
  */
void scifUartTxPutChar(char c) {

    // Put the character
    uint32_t txHead = scifTaskData.uartEmulator.state.txHead;
    scifTaskData.uartEmulator.input.pTxBuffer[txHead] = (uint8_t) c;

    // Update the TX FIFO head index
    if (++txHead == (sizeof(scifTaskData.uartEmulator.input.pTxBuffer) / sizeof(uint16_t))) {
        txHead = 0;
    }
    scifTaskData.uartEmulator.state.txHead = (uint16_t) txHead;

} // scifUartTxPutChar




/** \brief Transmits a single character, after the specified delay
  *
  * This function must not be called when the TX FIFO is full. Calling this function when the FIFO is
  * full will cause overflow, without warning. The number of free entries in the FIFO is:
  * \code
  * SCIF_UART_TX_FIFO_MAX_COUNT - scifUartGetTxFifoCount()
  * \endcode
  *
  * \param[in]      c
  *     The character to transmit
  * \param[in]      delay
  *     Delay inserted before this character is transmitted, given in number of bits
  */
void scifUartTxPutCharDelayed(char c, uint8_t delay) {

    // Put the character, with delay
    uint32_t txHead = scifTaskData.uartEmulator.state.txHead;
    scifTaskData.uartEmulator.input.pTxBuffer[txHead] = (delay << 8) | (uint8_t) c;

    // Update the TX FIFO head index
    if (++txHead == (sizeof(scifTaskData.uartEmulator.input.pTxBuffer) / sizeof(uint16_t))) {
        txHead = 0;
    }
    scifTaskData.uartEmulator.state.txHead = (uint16_t) txHead;

} // scifUartTxPutCharDelayed




/** \brief Transmits the specified number of character
  *
  * This function must not be called with count higher than the number of free entries in the TX FIFO.
  * Calling this function with too high count will cause overflow, without warning. The number of free
  * entries in the FIFO is:
  * \code
  * SCIF_UART_TX_FIFO_MAX_COUNT - scifUartGetTxFifoCount()
  * \endcode
  *
  * \param[in,out]  *pBuffer
  *     Pointer to the character source buffer
  * \param[in]      count
  *     Number of characters to put
  */
void scifUartTxPutChars(char* pBuffer, uint32_t count) {
    int n;

    // For each character ...
    uint32_t txHead = scifTaskData.uartEmulator.state.txHead;
    for (n = 0; n < count; n++) {

        // Get it
        scifTaskData.uartEmulator.input.pTxBuffer[txHead] = (uint8_t) pBuffer[n];

        // Update the TX FIFO head index
        if (++txHead == (sizeof(scifTaskData.uartEmulator.input.pTxBuffer) / sizeof(uint16_t))) {
            txHead = 0;
        }
    }
    scifTaskData.uartEmulator.state.txHead = (uint16_t) txHead;

} // scifUartTxPutChars




/** \brief Returns the SCIF UART events that had occurred when the last ALERT interrupt was generated
  *
  * This function does not return any new events until \ref scifUartClearEvents() has been called. Events
  * that have occurred in the meantime are stored in a backlog.
  *
  * The reported events exclude masked-out events (see \ref scifUartSetEventMask()).
  *
  * \return
  *     Bit-vector of the events that have occurred (one or more of the following):
  *     - \ref BV_SCIF_UART_ALERT_RX_FIFO_ABOVE_THR
  *     - \ref BV_SCIF_UART_ALERT_RX_BYTE_TIMEOUT
  *     - \ref BV_SCIF_UART_ALERT_RX_BREAK_OR_FRAMING_ERROR
  *     - \ref BV_SCIF_UART_ALERT_TX_FIFO_BELOW_THR
  */
uint16_t scifUartGetEvents(void) {
    return scifTaskData.uartEmulator.state.alertEvents;
} // scifUartGetEvents




/** \brief Clears the SCIF UART events reported by the last call to \ref scifUartGetEvents()
  *
  * This function must be called to get further event reports through ALERT interrupts.
  */
void scifUartClearEvents(void) {
    scifTaskData.uartEmulator.state.alertEvents = 0x0000;
} // scifUartClearEvents




/** \brief Select which SCIF UART events that shall trigger the ALERT interrupt
  *
  * The ALERT interrupt is generated when one or more of the events included in the mask occur.
  *
  * \param[in]      mask
  *     Bit-vector of the events that shall generate the ALERT interrupt (zero or more of the following):
  *     - \ref BV_SCIF_UART_ALERT_RX_FIFO_ABOVE_THR
  *     - \ref BV_SCIF_UART_ALERT_RX_BYTE_TIMEOUT
  *     - \ref BV_SCIF_UART_ALERT_RX_BREAK_OR_FRAMING_ERROR
  *     - \ref BV_SCIF_UART_ALERT_TX_FIFO_BELOW_THR
  */
void scifUartSetEventMask(uint16_t mask) {

    // Disable all events temporarily to avoid unwanted FIFO events
    scifTaskData.uartEmulator.cfg.alertMask           = mask;
    scifTaskData.uartEmulator.state.alertMask         = 0x0000;

} // scifUartSetEventMask


//@}


// Generated by TECLAST at 2020-05-22 08:30:00.516
