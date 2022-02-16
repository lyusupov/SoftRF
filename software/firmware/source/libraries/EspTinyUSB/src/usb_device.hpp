#pragma once
#include "esp_err.h"
#include "usb/usb_host.h"
#include "usb_host.hpp"

typedef void (*usb_host_event_cb_t)(int, void* data, size_t len);

class USBhostDevice
{
protected:
    const usb_config_desc_t *config_desc;
    uint8_t itf_num;
    usb_host_event_cb_t event_cb = nullptr;

    usb_transfer_t *xfer_ctrl = NULL;   // every device have EP0

public:
    USBhostDevice();
    ~USBhostDevice();

    esp_err_t init(size_t len = 64);
    usb_transfer_t * allocate(size_t);
    esp_err_t deallocate(usb_transfer_t *);    
    void onEvent(usb_host_event_cb_t _cb);
    USBhost* _host;
    bool deinit();
};

