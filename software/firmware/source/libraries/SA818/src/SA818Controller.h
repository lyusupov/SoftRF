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

#endif
