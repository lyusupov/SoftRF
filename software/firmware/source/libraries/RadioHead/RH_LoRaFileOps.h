// RH_LoRaFileOps.h
//
// Definitions for RadioHead driver on RPi+Linux
// and using LoRa-file-ops Linux driver ioctls to
// transmit and receive RadioHead compatible messages via SX1276/77/78/79
// and compatible radios.
// Requires a modified version of LoRa-file-ops driver to be installed,
// and a compatible radio to be connected
// appropriately
// https://github.com/starnight/LoRa/tree/file-ops
//
// Tested with:
// RPi 2 + Debian 2021-03-04 (kernel 5.10.17-v7+ #1403)
// Dragino LoRa/GPS HAT, https://wiki.dragino.com/index.php?title=Lora/GPS_HAT
// modified so RFM95 pin 5 NSS was no longer connected to RPi
// pin P1-22 (GPIO6), but is instead connected to RPi Pin P1-24 (CE0) which
// is the one used by /dev/loraSPI0.0
//
// Author: Mike McCauley (mikem@airspayce.com)
// Copyright (C) 2021 Mike McCauley
//

#ifndef RH_LORAFILEOPS_h
#define RH_LORAFILEOPS_h

#include <RHGenericDriver.h>
#warning RH_LoRaFileOps unfinished

// This can only build on Linux and compatible systems
// Caution also requires Lora-file-ops driver to be installed
// See https://github.com/starnight/LoRa/tree/file-ops
#if (RH_PLATFORM == RH_PLATFORM_UNIX) || defined(DOXYGEN)

// Driver constant definitions
// These are copied from LoRa file-ops branch pull request #16
// See the instructions in the RH_LoRaFileOps documentation for getting that version from github
// which is absolutely necessary if you want to support RadioHead messages with CRC enabled,
// which we strongly recommend.
// since they are not necessarily available in the compile host file
// system.
// CAUTION: these must be kept in sync with LoRa-file-ops if it changes
/* I/O control by each command. */
#include <sys/ioctl.h>
#define LORA_IOC_MAGIC '\x74'

#define LORA_SET_STATE		(_IOW(LORA_IOC_MAGIC,  0, int))
#define LORA_GET_STATE		(_IOR(LORA_IOC_MAGIC,  1, int))
#define LORA_SET_FREQUENCY	(_IOW(LORA_IOC_MAGIC,  2, int))
#define LORA_GET_FREQUENCY	(_IOR(LORA_IOC_MAGIC,  3, int))
#define LORA_SET_POWER		(_IOW(LORA_IOC_MAGIC,  4, int))
#define LORA_GET_POWER		(_IOR(LORA_IOC_MAGIC,  5, int))
#define LORA_SET_LNA		(_IOW(LORA_IOC_MAGIC,  6, int))
#define LORA_GET_LNA		(_IOR(LORA_IOC_MAGIC,  7, int))
#define LORA_SET_LNAAGC		(_IOR(LORA_IOC_MAGIC,  8, int))
#define LORA_SET_SPRFACTOR	(_IOW(LORA_IOC_MAGIC,  9, int))
#define LORA_GET_SPRFACTOR	(_IOR(LORA_IOC_MAGIC, 10, int))
#define LORA_SET_BANDWIDTH	(_IOW(LORA_IOC_MAGIC, 11, int))
#define LORA_GET_BANDWIDTH	(_IOR(LORA_IOC_MAGIC, 12, int))
#define LORA_GET_RSSI		(_IOR(LORA_IOC_MAGIC, 13, int))
#define LORA_GET_SNR		(_IOR(LORA_IOC_MAGIC, 14, int))
/* Mikem added 2021-04-19 fro pull request 16: */
#define LORA_SET_CRC		(_IOW(LORA_IOC_MAGIC, 15, int))
#define LORA_SET_CODINGRATE	(_IOW(LORA_IOC_MAGIC, 16, int))
#define LORA_GET_CODINGRATE	(_IOR(LORA_IOC_MAGIC, 17, int))
#define LORA_SET_IMPLICIT	(_IOW(LORA_IOC_MAGIC, 18, int))
#define LORA_SET_LDRO		(_IOW(LORA_IOC_MAGIC, 19, int))
#define LORA_SET_PREAMBLE	(_IOW(LORA_IOC_MAGIC, 20, int))
#define LORA_GET_PREAMBLE	(_IOR(LORA_IOC_MAGIC, 21, int))
#define LORA_SET_PARAMP		(_IOW(LORA_IOC_MAGIC, 22, int))
#define LORA_GET_PARAMP		(_IOR(LORA_IOC_MAGIC, 23, int))
#define LORA_SET_OCPIMAX	(_IOW(LORA_IOC_MAGIC, 24, int))
#define LORA_GET_OCPIMAX	(_IOR(LORA_IOC_MAGIC, 25, int))
#define LORA_SET_LNABOOSTHF	(_IOW(LORA_IOC_MAGIC, 26, int))
#define LORA_SET_PMAX20DBM	(_IOW(LORA_IOC_MAGIC, 27, int))

  
/* List the state of the LoRa device. */
#define LORA_STATE_SLEEP	0
#define LORA_STATE_STANDBY	1
#define LORA_STATE_TX		2
#define LORA_STATE_RX		3
#define LORA_STATE_CAD		4

// Max number of octets the SX1278 LORA Rx/Tx FIFO can hold
#define RH_LORAFILEOPS_FIFO_SIZE 255

// This is the maximum number of bytes that can be carried by the LORA.
// We use some for headers, keeping fewer for RadioHead messages
#define RH_LORAFILEOPS_MAX_PAYLOAD_LEN RH_LORAFILEOPS_FIFO_SIZE

// The length of the headers we add.
// The headers are inside the LORA's payload
#define RH_LORAFILEOPS_HEADER_LEN 4

// This is the maximum message length that can be supported by this driver. 
// Can be pre-defined to a smaller size (to save SRAM) prior to including this header
// Here we allow for 1 byte message length, 4 bytes headers, user data and 2 bytes of FCS
#ifndef RH_LORAFILEOPS_MAX_MESSAGE_LEN
#define RH_LORAFILEOPS_MAX_MESSAGE_LEN (RH_LORAFILEOPS_MAX_PAYLOAD_LEN - RH_LORAFILEOPS_HEADER_LEN)
#endif

/////////////////////////////////////////////////////////////////////
/// \class RH_LoRaFileOps RH_LoRaFileOps.h <RH_LoRaFileOps.h>
/// \brief Driver to send and receive unaddressed, unreliable datagrams via a LoRa 
/// capable radio transceiver on a Linux platform (possibly Raspberry Pi), using the
/// lora-file-ops driver by Jian-Hong Pan (starnight):
/// https://github.com/starnight/LoRa/tree/file-ops
///
/// This RadioHead driver is only available to Commercial licensees. Apply to info@airspayce.com.
///
/// For an excellent discussion of LoRa range and modulations, see
/// https://medium.com/home-wireless/testing-lora-radios-with-the-limesdr-mini-part-2-37fa481217ff
/// Works with Dragino LoRa/GPS HAT, https://wiki.dragino.com/index.php?title=Lora/GPS_HAT
/// modified so RFM95 pin 5 NSS was no longer connected to RPi
/// pin P1-22 (GPIO6), but is instead connected to RPi Pin P1-24 (CE0) which
/// is the one used by /dev/loraSPI0.0. Interoperates with RH_RF95
/// with modem config RH_RF95::Bw125Cr45Sf2048
///
/// \par Overview
///
/// This class provides basic functions for sending and receiving unaddressed, 
/// unreliable datagrams of arbitrary length up to 251 octets per packet.
///
/// Manager classes may use this class to implement reliable, addressed datagrams and streams, 
/// mesh routers, repeaters, translators etc.
///
/// Naturally, for any 2 radios to communicate that must be configured to use the same frequency and 
/// modulation scheme.
///
/// This RadioHead Driver provides an object-oriented interface for sending and receiving
/// data messages with Semtech SX1276/77/78/79
/// and compatible radio modules in LoRa mode, using the lora-file-ops Linux driver. It only runs on Linux
/// such as Raspberry Pi Debian etc.is a low-cost ISM transceiver
/// chip. It supports FSK, GFSK, OOK over a wide range of frequencies and
/// programmable data rates, and it also supports the proprietary LoRA (Long Range) mode, which
/// is the only mode supported in this RadioHead driver (because that is the only mode supported by the
/// underlying lora-file-ops Linux driver.
///
/// This Driver provides functions for sending and receiving messages of up
/// to 251 octets on any frequency supported by the radio, in a range of
/// predefined Bandwidths, Spreading Factors and Coding Rates.  Frequency can be set with
/// 61Hz precision to any frequency from 240.0MHz to 960.0MHz. Caution: most modules only support a more limited
/// range of frequencies due to antenna tuning.
///
/// Up to 2 modules are supported by lora-file-ops
/// permitting the construction of translators and frequency changers, etc.
///
/// Support for other features such as transmitter power control etc is
/// also provided.
///
/// Tested with:
/// RPi 2 + Debian 2021-03-04 (kernel 5.10.17-v7+ #1403)
/// Dragino LoRa/GPS HAT, https://wiki.dragino.com/index.php?title=Lora/GPS_HAT
/// modified so RFM95 pin 5 NSS was no longer connected to RPi
/// pin P1-22 (GPIO6), but is instead connected to RPi Pin P1-24 (CE0) which
/// is the one used by /dev/loraSPI0.0
/// \par Packet Format
///
/// All messages sent and received by this RH_RF95 Driver conform to this packet format:
///
/// - LoRa mode:
/// - 8 symbol PREAMBLE
/// - Explicit header with header CRC (default CCITT, handled internally by the radio)
/// - 4 octets HEADER: (TO, FROM, ID, FLAGS)
/// - 0 to 251 octets DATA 
/// - CRC (default CCITT, handled internally by the radio)
///
/// This format is compatible with the one used by RH_RF95 by default.
///
/// \par Modulation
///
/// The default modulation scheme implemented by this driver is:
/// 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
/// which is compatible withthe RH_RF95 modem config RH_RF95::Bw125Cr45Sf2048 and so this RadioHead driver will
/// interoperate with RH_RF95.
///
/// \par Installing lora-file-ops
/// For this driver to work on a Linux platform such as Raspberry Pi, it is absolutely necessary to install the
/// LoRa-file-ops Linux driver written by starnight, and which is available from github.
/// The version currently available (2021-04-19) does not support enabling CRCs in the radio, which is
/// strongly recommended, and necessary to work with any other RadioHead lora driver/
/// At this date, code to add CRC suport to the driver is avalable as a pull request on github
/// as a Git pull request #16 from flyskywhy. We strongly recommend using it as described below.
///
/// To get LoRa-file-ops from starnight, plus the necessary patches and fixes from pull request #16 from flyskywhy
/// and to build it and install it and load it into the kernel for testing:
/// \code
/// # ON a recent Debian kernel:
/// sudo apt-get install linux-headers-rpi raspberrypi-kernel-headers
/// # Enable the SPI interface in the kernel
/// sudo raspi-config:
///    -> 3 Interface Options
///    -> P4 SPI
///    -> Would you like the SPI interface to be enabled? select Yes, press Return, Return, Select Finish
/// # in a working directory, not as root:
/// git clone https://github.com/starnight/LoRa.git
/// cd LoRa/
/// git checkout file-ops
/// git fetch origin pull/16/head:file-ops-patched  # only until pull #16 is not merged into master
/// git checkout file-ops-patched                   # only until pull #16 is not merged into master
/// cd LoRa/
/// make
/// make install
/// cd ../dts-overlay
/// make
/// cd ../
/// # and after every reboot:
/// sudo dtoverlay rpi-lora-spi
/// sudo modprobe sx1278
/// \endcode
///
/// If you want to permanently add the LoRa-file-ops Linux driver so it loads automatically
/// on every boot, add this to /boot/config.txt
/// \code
/// dtparam=rpi-lora-spi=on
/// \endcode
///
/// Note: it may be the case in the future that pull request 16 is merged into the master of LoRa-File-Ops
/// in which case 2 steps are not needed above
class RH_LoRaFileOps : public RHGenericDriver
{
public:
    /// Constructor. You can have multiple instances each connected to a different LoRa port
    /// \param[in] port Name of the lora-file-ops port, typically something like /dev/loraSPI0.0
    RH_LoRaFileOps(const char* port);
  
    /// Initialise the Driver transport hardware and software.
    /// Opens the LorFileOps driver port and initalises the radio to default settings
    /// Leaves the radio in receive mode,
    /// with default configuration of: 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 2048chips/symbol, CRC on
    /// which is compatible with RH_RF95::Bw125Cr45Sf2048
    /// \return true if initialisation succeeded.
    virtual bool    init();
  
    /// Tests whether a new message is available from the lora-file-ops Linux driver. 
    /// This can be called multiple times in a timeout loop
    /// \return true if a new, complete, error-free uncollected message is available to be retreived by recv()
    virtual bool    available();

    // Sigh, its not possible to implement waitAvailable and waitAvailableTimout in terms
    // of select(), since the LoRa-file-ops driver does not detect any interrupts, and
    // therefore select will not return when a packet is received by the radio.
    // So we have to live with the RHGenericDriver implementations that call available() to poll the port.
    // waitAvailableTimeout() supports an optional delay between each poll
    // of available() so that on Linux at least another process can get the CPU.
  
    /// If there is a valid message available and it is for this node, copy it to buf and return true
    /// else return false.
    /// If a message is copied, *len is set to the length (Caution, 0 length messages are permitted).
    /// You should be sure to call this function frequently enough to not miss any messages
    /// or call it aafter available(), waitAvailable() or waitAvailableTimeout()
    /// indicate that a message is avalable
    /// It is recommended that you call it in your main loop.
    /// \param[in] buf Location to copy the received message
    /// \param[in,out] len Pointer to available space in buf. Set to the actual number of octets copied.
    /// \return true if a valid message addressed to this node was copied to buf
    virtual bool    recv(uint8_t* buf, uint8_t* len);

    /// Loads a message into the transmitter and starts the transmitter. Note that a message length
    /// of 0 is permitted. CAD is not supported yet.
    /// The lora-file-ops driver waits for the entire message to be transmitted before resuming operations.
    /// \param[in] data Array of data to be sent
    /// \param[in] len Number of bytes of data to send
    /// \return true if the message length was valid and it was correctly transmitted.
    virtual bool    send(const uint8_t* data, uint8_t len);

    /// Returns the maximum message length 
    /// available in this Driver.
    /// \return The maximum legal message length
    virtual uint8_t maxMessageLength();
    
    /// Sets the transmitter and receiver 
    /// centre (carrier) frequency.
    /// \param[in] centre Frequency in Hz. 137000000 to 1020000000. Caution: SX1276/77/78/79 comes in several
    /// different frequency ranges, and setting a frequency outside that range of your radio will probably not work
    /// correctly becasue the antenna coupling or antenna wont work outside their designed frequency range
    /// \return true if the selected frequency centre is within range
    bool           setFrequency(uint32_t centre);

    /// Returns the current transmitter and receiver 
    /// centre frequency.
    /// \return Centre frequency in Hz.
    uint32_t       getFrequency();
    
    /// Returns the Signal-to-noise ratio (SNR) of the last received message, as measured
    /// by the receiver.
    /// \return SNR of the last received message in dB
    int lastSNR();

    /// Sets the transmitter power output level
    /// Be a good neighbour and set the lowest power level you need.
    /// Caution: legal power limits may apply in certain countries.
    /// After init(), the power will be set to 13dBm
    /// \param[in] power Transmitter power level in dBm. Max 20dBm.
    void           setTxPower(int32_t power);

    /// Gets the currently set transmitter power output level
    /// \return Current poer level in dbM
    int32_t        getTxPower();

    /// Set the LoRa Spreading Factor
    /// \param[in] sf The spreading factor. Valid values are 64, 128, 256, 512, 1024, 2048, 4096.
    void           setSpreadingFactor(int32_t sf);

    /// Get the LoRa Spreading Factor
    /// \return The current Spreading Factor
    int32_t        getSpreadingFactor();

    /// Gets the RSSI of the last received packet
    /// \return RSSI of the last received packet
    int32_t        getRSSI();

    /// Gets the Signal To Noise (SNR) of the last received packet
    /// \return SNR of the last received packet
    int32_t        getSNR();

    /// Set the receiver Low Noise Amplifier (LNA) gain
    /// \param[in] lna LNA gain in dBm
    void           setLNA(int32_t lna);

    /// Get the current LNA gain
    /// \return The current LNA gain in dBm
    int32_t        getLNA();

    /// Set the LNA Automatic Gain Control (AGC) enabled
    /// \param[in] lnaagc 1 to enable LNA AGC, 0 to disable it
    void           setLNAAGC(int32_t lnaagc);

    /// Set the transmitter and receiver modulation bandwidth
    /// \param[in] bw Modulation bandwidth in Hz. Valid values are 7800, 10400, 15600, 20800,
    /// 312500, 41700, 62500, 125000, 250000, 500000.
    void           setBW(int32_t bw);

    /// Get the transmitter and receiver modulation bandwidth
    /// \return Modulation bandwidth in Hz
    int32_t        getBW();

    /// Enable Cyclic Redundancy Check (CRC) in the transmitter and receiver. If enabled,
    /// the transmitter will always appenda CRC to every packet, and the receiver will
    /// always check the CRC on received packets, ignoring packets with incorrect CRC
    /// \param[in] crc 1 to enable CRC generation and detection, 0 to disable it
    void           setCRC(uint32_t crc);


protected:
    /// Set the current radio state, one of LORA_STATE_*
    void           setState(uint32_t state);

    /// Get the current radio state
    uint32_t       getState();
    
private:
    // The name of the Unix filesystm port for the Lora SX1278 compatible radio
    // typically /dev/loraSPI0.0 or similar
    const char*   _port;

    /// Unix file system device number of the LoRa device port. -1 if not open
    int            _fd;

    /// Last measured SNR, dB
    int8_t         _lastSNR;
};

/// @example lorafileops_client.cpp
/// @example lorafileops_server.cpp

#endif
#endif
