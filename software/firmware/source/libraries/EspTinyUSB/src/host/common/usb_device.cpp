#include "esp_log.h"
#include "usb_device.hpp"

USBhostDevice::USBhostDevice()
{
}

USBhostDevice::~USBhostDevice()
{
}

esp_err_t USBhostDevice::init(size_t len)
{
    esp_err_t err = usb_host_transfer_alloc(len, 0, &xfer_ctrl);
    xfer_ctrl->device_handle = _host->deviceHandle();
    xfer_ctrl->context = this;
    xfer_ctrl->bEndpointAddress = 0;

    return err;
}

IRAM_ATTR usb_transfer_t *USBhostDevice::allocate(size_t _size)
{
    usb_transfer_t *transfer = NULL;

    esp_err_t err = usb_host_transfer_alloc(_size, 0, &transfer);
    if (!err)
    {
        transfer->device_handle = _host->deviceHandle();
        transfer->context = this;
    }
    return transfer;
}

IRAM_ATTR esp_err_t USBhostDevice::deallocate(usb_transfer_t *transfer)
{
    esp_err_t err = usb_host_transfer_free(transfer);
    if (ESP_OK != err)
    {
        ESP_LOGE("", "deallocate free transfer : %d", err);
    }

    return err;
}

void USBhostDevice::onEvent(usb_host_event_cb_t _cb)
{
    event_cb = _cb;
}

bool USBhostDevice::deinit()
{
    for (size_t n = 0; n < config_desc->bNumInterfaces; n++)
    {
        usb_host_interface_release(_host->clientHandle(), _host->deviceHandle(), n);
    }

    return true;
}
