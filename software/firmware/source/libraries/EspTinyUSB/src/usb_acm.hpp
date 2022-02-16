#pragma once
#include "usb_device.hpp"
#include "usb_requests.hpp"

#define CDC_DATA_IN                             1
#define CDC_DATA_OUT                            2
#define CDC_CTRL_SET_CONTROL_LINE_STATE         3
#define CDC_CTRL_SET_LINE_CODING                4
#define CDC_CTRL_GET_LINE_CODING                5

class USBacmDevice : public USBhostDevice
{
    friend void IRAM_ATTR usb_ctrl_cb(usb_transfer_t *transfer);
    friend void IRAM_ATTR usb_read_cb(usb_transfer_t *transfer);
    friend void IRAM_ATTR usb_write_cb(usb_transfer_t *transfer);

private:
    const usb_ep_desc_t * ep_int;
    const usb_ep_desc_t * ep_in;
    const usb_ep_desc_t * ep_out;
    bool connected;

public:
    USBacmDevice(const usb_config_desc_t* config_desc, USBhost*);
    ~USBacmDevice();

    bool init();
    void setControlLine(bool dtr, bool rts);
    void setLineCoding(uint32_t bitrate, uint8_t cf, uint8_t parity, uint8_t bits);
    void getLineCoding();
    void INDATA(size_t len = 64);
    void OUTDATA(uint8_t*, size_t);
    bool isConnected();

private:
    void _callback(int event, usb_transfer_t* data);
};

