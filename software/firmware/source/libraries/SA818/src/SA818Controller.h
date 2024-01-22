#ifndef SA818_CONTROLLER_H
#define SA818_CONTROLLER_H

#include <SA818.h>

class SA818Controller {
private:
    SA818* sa = 0;
    byte type_ = Model::SA_818;
    byte band_ = Band::VHF;

    byte bw_;
    float tx_f_;
    float rx_f_;
    int tx_sub_;
    byte sq_;
    int rx_sub_;

    uint8_t ptt_ = 0;
    uint8_t pd_  = 0;
    uint8_t hl_  = 0;
    bool _transmitStatus = false;

    bool scanning = false;

    String ctcss(int);
    float loopScan(float, bool, float = 0, unsigned long = 0);

public:
    SA818Controller(SA818*);
    ~SA818Controller();
    void setModel(Model = Model::SA_818);
    byte getModel();
    void setBand(Band = Band::VHF);
    byte getBand();
    void setPins(uint8_t ptt, uint8_t pd, uint8_t hl);
    String response();
    String result();

    void setBW(byte);
    void setTXF(float);
    float getTXF();
    void setRXF(float);
    void setTXSub(int);
    void setSQ(byte);
    void setRXSub(int);
    bool update();

    bool connect();
    bool setGroup(byte, float, float, int, byte, int);
    bool scan(float);
    bool setVolume(byte);
    bool setFilter(byte, byte, byte);
    bool openTail();
    bool closeTail();
    String rssi();
    String version();
    String model();

    float next(float, float = 0, unsigned long = 0);
    float previous(float, float = 0, unsigned long = 0);
    int scanlist(float[], int, float[]);
    void stopScan();

    void sleep();
    void wake();
    void highPower();
    void lowPower();
    void receive();
    void transmit();
    bool getTxStatus();
};

/*
 * TODO
 *
 * Source: https://github.com/nakhonthai/ESP32APRS_T-TWR/pull/17
 */

enum class OpenEdition_Mode: uint8_t
{
    OFF = 0,
    RX,
    TX
};

typedef struct OpenEdition_Config_Struct
{
	uint32_t freq_rx;
	uint32_t freq_tx;
	int tone_rx;
	int tone_tx;
	uint8_t band;
	uint8_t sql_level;
	bool rf_power;
	uint8_t volume;
	uint8_t mic;
    OpenEdition_Mode mode;
} OpenEdition_Configuration;

typedef struct OpenEdition_Version_Struct
{
    uint8_t major;
    uint8_t minor;
    uint8_t patch;
    uint8_t revision;
} OpenEdition_Version;

class OpenEdition
{
    public:
        OpenEdition(HardwareSerial * SerialRF, uint8_t RX_PIN, uint8_t TX_PIN);
        void init();
        OpenEdition_Version Version();
        int16_t getRSSI();
        void RxOn();
        void TxOn();
        void TxOff();
        void setAudio(bool value);
        void setTxFrequency(uint32_t freq);
        void setRxFrequency(uint32_t freq);
        void setTxTone(uint32_t tone);
        void setRxTone(uint32_t tone);
        void setVolume(uint8_t value);
        void setBandwidth(uint8_t value);
        void setSqlThresh(uint8_t value);
        OpenEdition_Configuration settings();
        bool isHighPower();
        void setHighPower();
        void setLowPower();
    private:
        HardwareSerial * _SerialRF;
        OpenEdition_Configuration _config;
        uint8_t _RX_PIN;
        uint8_t _TX_PIN;
        void setFrequency(uint32_t freq);
        void updateBandwidth();
        void setSqlThresh();
        void setPower();
};

#endif /* SA818_CONTROLLER_H */
