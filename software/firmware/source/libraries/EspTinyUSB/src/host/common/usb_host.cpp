#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "usb/usb_host.h"

#include "usb_host.hpp"

void _client_event_callback(const usb_host_client_event_msg_t *event_msg, void *arg)
{
    USBhost *host = (USBhost *)arg;
    if (event_msg->event == USB_HOST_CLIENT_EVENT_NEW_DEV)
    {
        host->open(event_msg);
        ESP_LOGI("USB_HOST_CLIENT_EVENT_NEW_DEV", "client event: %d, address: %d", event_msg->event, event_msg->new_dev.address);
        if (host->_client_event_cb)
        {
            host->_client_event_cb(event_msg, arg);
        } else {
            
        }
    } else {
        ESP_LOGI("USB_HOST_CLIENT_EVENT_DEV_GONE", "client event: %d", event_msg->event);
        if (host->_client_event_cb)
        {
            host->_client_event_cb(event_msg, arg);
        }
        host->close();
    }
}

static void client_async_seq_task(void *param)
{
    USBhost* host = (USBhost *)param;
    printf("create async task\n");
    while (1)
    {
        usb_host_client_handle_t client_hdl = host->client_hdl;
        uint32_t event_flags;
        if(client_hdl)usb_host_client_handle_events(client_hdl, 1);
        if (ESP_OK == usb_host_lib_handle_events(1, &event_flags))
        {
            if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS)
            {
                ESP_LOGD("", "No more clients\n");
                do{
                    if(usb_host_device_free_all() != ESP_ERR_NOT_FINISHED) break;
                }while(1);
                usb_host_uninstall();
                host->init(false);
            }
            if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE)
            {
                ESP_LOGD("", "USB_HOST_LIB_EVENT_FLAGS_ALL_FREE\n");
                usb_host_client_deregister(client_hdl);
                host->client_hdl = NULL;
            }
        }
    }
    printf("delete task\n");
    vTaskDelete(NULL);
}

USBhost::USBhost()
{
}

USBhost::~USBhost()
{
}

bool USBhost::init(bool create_tasks)
{
    const usb_host_config_t config = {
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    esp_err_t err = usb_host_install(&config);
    ESP_LOGI("", "install status: %d", err);

    const usb_host_client_config_t client_config = {
        .is_synchronous = false,
        .max_num_event_msg = 5,
        .async = {
            .client_event_callback = _client_event_callback,
            .callback_arg = this,
        }
    };

    err = usb_host_client_register(&client_config, &client_hdl);
    ESP_LOGI("", "client register status: %d", err);

    if (create_tasks)
    {
        xTaskCreate(client_async_seq_task, "async", 6 * 512, this, 20, NULL);
    }

    return true;
}

bool USBhost::open(const usb_host_client_event_msg_t *event_msg)
{
    esp_err_t err = usb_host_device_open(client_hdl, event_msg->new_dev.address, &dev_hdl);
    parseConfig();

    return true;
}

void USBhost::close()
{
    usb_host_device_close(client_hdl, dev_hdl);
}

void USBhost::parseConfig()
{
    const usb_device_desc_t *device_desc;
    usb_host_get_device_descriptor(dev_hdl, &device_desc);
    // ESP_LOG_BUFFER_HEX("", device_desc->val, USB_DEVICE_DESC_SIZE);
    const usb_config_desc_t *config_desc;
    usb_host_get_active_config_descriptor(dev_hdl, &config_desc);
}

usb_device_info_t USBhost::getDeviceInfo()
{
    usb_host_device_info(dev_hdl, &dev_info);

    return dev_info;
}

const usb_device_desc_t* USBhost::getDeviceDescriptor()
{
    const usb_device_desc_t *device_desc;
    usb_host_get_device_descriptor(dev_hdl, &device_desc);

    return device_desc;
}

const usb_config_desc_t* USBhost::getConfigurationDescriptor()
{
    const usb_config_desc_t *config_desc;
    usb_host_get_active_config_descriptor(dev_hdl, &config_desc);
    return config_desc;
}

uint8_t USBhost::getConfiguration()
{
    return getDeviceInfo().bConfigurationValue;
}

usb_host_client_handle_t USBhost::clientHandle()
{
    return client_hdl;
}

usb_device_handle_t USBhost::deviceHandle()
{
    return dev_hdl;
}

// bool USBhost::setConfiguration(uint8_t);
