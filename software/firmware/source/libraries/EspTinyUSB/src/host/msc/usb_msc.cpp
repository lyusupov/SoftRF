#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include <string.h>

#include "usb_msc.hpp"
#include "diskio_rawmsc.hpp"
#include "usb_requests.hpp"

void IRAM_ATTR usb_transfer_cb(usb_transfer_t *transfer)
{
    ESP_LOGW("STATUS", "status: %d, bytes: %d", transfer->status, transfer->actual_num_bytes);
    ESP_LOG_BUFFER_HEX("HEX_DATA", transfer->data_buffer, transfer->actual_num_bytes);

    USBmscDevice *dev = (USBmscDevice *)transfer->context;

    if (dev && transfer->actual_num_bytes == 9 && *(uint16_t *)transfer->data_buffer == 0xfea1) // max luns
    {
        dev->onCSW(transfer);
    }
    else if (dev && transfer->actual_num_bytes == 31 && *(uint32_t *)transfer->data_buffer == 0x43425355) // CBW
    {
        dev->onCBW(transfer);
    }
    else if (dev && transfer->actual_num_bytes == 13 && *(uint32_t *)transfer->data_buffer == 0x53425355) // CSW
    {
        dev->onCSW(transfer);
    }
    else
    {
        dev->onData(transfer);
        ESP_LOG_BUFFER_HEX("DATA", transfer->data_buffer, transfer->actual_num_bytes);
    }
    dev->deallocate(transfer);
}

USBmscDevice *USBmscDevice::instance = nullptr;

USBmscDevice *USBmscDevice::getInstance()
{
    return instance;
}

USBmscDevice::USBmscDevice(const usb_config_desc_t *_config_desc, USBhost *host)
{
    esp_log_level_set("STATUS", ESP_LOG_ERROR);
    esp_log_level_set("EMIT", ESP_LOG_ERROR);
    esp_log_level_set("HEX", ESP_LOG_ERROR);
    esp_log_level_set("HEX_DATA", ESP_LOG_ERROR);
    esp_log_level_set("DATA", ESP_LOG_ERROR);

    _host = host;
    config_desc = _config_desc;
    int offset = 0;
    for (size_t n = 0; n < config_desc->bNumInterfaces; n++)
    {
        const usb_intf_desc_t *intf = usb_parse_interface_descriptor(config_desc, n, 0, &offset);
        const usb_ep_desc_t *ep = nullptr;

        if (intf->bInterfaceClass == 0x08)
        {
            if (intf->bNumEndpoints != 2)
                return;
            // ESP_LOGI("", "EP MSC.");
            for (size_t i = 0; i < intf->bNumEndpoints; i++)
            {
                int _offset = 0;
                ep = usb_parse_endpoint_descriptor_by_index(intf, i, config_desc->wTotalLength, &_offset);
                if (ep->bEndpointAddress & 0x80)
                {
                    ep_in = ep;
                }
                else
                {
                    ep_out = ep;
                }

                if (ep)
                {
                    printf("EP num: %d/%d, len: %d, ", i + 1, intf->bNumEndpoints, config_desc->wTotalLength);
                    printf("address: 0x%02x, EP max size: %d, dir: %s\n", ep->bEndpointAddress, ep->wMaxPacketSize, (ep->bEndpointAddress & 0x80) ? "IN" : "OUT");
                }
                else
                    ESP_LOGW("", "error to parse endpoint by index; EP num: %d/%d, len: %d", i + 1, intf->bNumEndpoints, config_desc->wTotalLength);
            }
            itf_num = n;
            esp_err_t err = usb_host_interface_claim(_host->clientHandle(), _host->deviceHandle(), itf_num, 0);
            ESP_LOGI("", "interface %d claim status: %d", itf_num, err);
        }
    }
    instance = this;
}

USBmscDevice::~USBmscDevice()
{
    // TODO
    // deallocate(xfer_ctrl);
}

bool USBmscDevice::init()
{
    usb_device_info_t info = _host->getDeviceInfo();
    USBhostDevice::init(info.bMaxPacketSize0);
    xfer_ctrl->callback = usb_transfer_cb;

    _getMaxLUN();
    return true;
}

bool USBmscDevice::mount(char *path, uint8_t lun)
{
    if(lun > _luns) return false;
    const esp_vfs_fat_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 1,
        .allocation_unit_size = block_size[lun]};
    esp_err_t err = vfs_fat_rawmsc_mount(path, &mount_config, lun);
    ESP_LOGI("", "VFS mount status: %d", err);

    return err == ESP_OK;
}

void USBmscDevice::unmount(char *path, uint8_t lun)
{
    vfs_fat_rawmsc_unmount(path, lun);
}

void USBmscDevice::getDrivePath(char* path, uint8_t lun)
{
    int8_t vol = ff_msc_get_pdrv_raw(lun);
    if(vol >= 0)
        sprintf(path, "%d:", vol);
}

uint8_t USBmscDevice::getMaxLUN()
{
    return _luns;
}

uint32_t USBmscDevice::getBlockCount(uint8_t lun)
{
    return block_count[lun];
}

uint16_t USBmscDevice::getBlockSize(uint8_t lun)
{
    return block_size[lun];
}

void USBmscDevice::registerCallbacks(msc_transfer_cb_t cb)
{
    callbacks = cb;
}

void USBmscDevice::reset()
{
    event = 2;  // our own event value
    MSC_SCSI_REQ_INIT_RESET((usb_setup_packet_t *)xfer_ctrl->data_buffer, itf_num);
    xfer_ctrl->num_bytes = sizeof(usb_setup_packet_t);
    esp_err_t err = usb_host_transfer_submit_control(_host->clientHandle(), xfer_ctrl);
}

// TODO
void USBmscDevice::format()
{
    /*
    scsi_cmd10_t cbw = {};
    cbw.opcode = SCSI_CMD_FORMAT_UNIT;
    // cbw.flags = _lun | 0xE8;
    // cbw.lba_3 = (offset >> 24);
    // cbw.lba_2 = (offset >> 16);
    // cbw.lba_1 = (offset >> 8);
    // cbw.lba_0 = (offset >> 0);
    // cbw.group = 0;
    // cbw.len_1 = (num_sectors >> 8);
    // cbw.len_0 = (num_sectors >> 0);
    // cbw.control = 0;
    event = cbw.opcode;

    MSC_SCSI_INIT_CBW((msc_bulk_cbw_t *)xfer_out->data_buffer, true, _lun, 0x1234, cbw, 8);

    xfer_out->num_bytes = sizeof(msc_bulk_cbw_t);

    esp_err_t err = usb_host_transfer_submit(xfer_out);
    printf("test capacity: 0x%02x => 0x%02x\n", err, xfer_out->bEndpointAddress);
    _csw();
    */
}

//------------------------------------- ff_diskio ---------------------------------//

esp_err_t USBmscDevice::_read10(uint8_t lun, int offset, int num_sectors, uint8_t *buff)
{
    temp_buffer = buff;
    scsi_cmd10_t cbw = {};
    cbw.opcode = SCSI_CMD_READ_10;
    cbw.flags = 0;
    cbw.lba_3 = (offset >> 24);
    cbw.lba_2 = (offset >> 16);
    cbw.lba_1 = (offset >> 8);
    cbw.lba_0 = (offset >> 0);
    cbw.group = 0;
    cbw.len_1 = (num_sectors >> 8);
    cbw.len_0 = (num_sectors >> 0);
    cbw.control = 0;

    event = cbw.opcode;

    usb_transfer_t *xfer_read = allocate(num_sectors * block_size[lun]);
    if (!xfer_read)
    {
        return ESP_ERR_NO_MEM;
    }
    
    xfer_read->callback = usb_transfer_cb;
    xfer_read->bEndpointAddress = ep_out->bEndpointAddress;
    xfer_read->num_bytes = sizeof(msc_bulk_cbw_t);
    MSC_SCSI_INIT_CBW((msc_bulk_cbw_t *)xfer_read->data_buffer, true /*is_read*/, lun, 0x1234, cbw, num_sectors * block_size[lun]);

    esp_err_t err = usb_host_transfer_submit(xfer_read);
    _csw();
    // TODO fix this workaround
    if (err == 0x103){
        ESP_LOGW("", "test read10: 0x%02x => 0x%02x, sector: %d [%d][%d]\n", err, xfer_read->bEndpointAddress, offset, *(uint32_t *)&cbw.lba_3, __builtin_bswap32(*(uint32_t *)&cbw.lba_3));
        vTaskDelay(1);
        deallocate(xfer_read);
        err = _read10(lun, offset, num_sectors, buff);
    }
    else
    {
        xTaskToNotify = xTaskGetCurrentTaskHandle();
        uint32_t pulNotificationValue;
        if (xTaskNotifyWait(0, 0, &pulNotificationValue, 20 / portTICK_PERIOD_MS) == pdTRUE)
        {
            return ESP_OK;
        } else

        err = ESP_ERR_TIMEOUT;
        ESP_LOGE("", "test write10 data: 0x%02x", err);
    }

    return err;
}

esp_err_t USBmscDevice::_write10(uint8_t lun, int offset, int num_sectors, uint8_t *buff)
{

    scsi_cmd10_t cbw;
    cbw.opcode = SCSI_CMD_WRITE_10;
    cbw.flags = 0;
    cbw.lba_3 = (offset >> 24);
    cbw.lba_2 = (offset >> 16);
    cbw.lba_1 = (offset >> 8);
    cbw.lba_0 = (offset >> 0);
    cbw.group = 0;
    cbw.len_1 = (num_sectors >> 8);
    cbw.len_0 = (num_sectors >> 0);
    cbw.control = 0;

    event = cbw.opcode;

    usb_transfer_t *xfer_write = allocate(sizeof(msc_bulk_cbw_t));
    if (!xfer_write)
    {
        return ESP_ERR_NO_MEM;
    }
    
    xfer_write->callback = usb_transfer_cb;
    xfer_write->bEndpointAddress = ep_out->bEndpointAddress;

    MSC_SCSI_INIT_CBW((msc_bulk_cbw_t *)xfer_write->data_buffer, false /*is_read*/, lun, 0x1234, cbw, block_size[_lun] * num_sectors);

    xfer_write->num_bytes = sizeof(msc_bulk_cbw_t);

    esp_err_t err = usb_host_transfer_submit(xfer_write);
    if (err){
        ESP_LOGW("", "test write10: 0x%02x => 0x%02x, sector: %d [%d][%d]", err, xfer_write->bEndpointAddress, offset, *(uint32_t *)&cbw.lba_3, __builtin_bswap32(*(uint32_t *)&cbw.lba_3));
        deallocate(xfer_write);
        return -1;
    }

    usb_transfer_t *xfer_read = allocate(num_sectors * block_size[lun]);
    if (!xfer_read)
    {
        return ESP_ERR_NO_MEM;
    }

    xfer_read->callback = usb_transfer_cb;
    xfer_read->bEndpointAddress = ep_out->bEndpointAddress;
    xfer_read->num_bytes = block_size[_lun] * num_sectors;
    memcpy(xfer_read->data_buffer, buff, xfer_read->num_bytes);

    err = usb_host_transfer_submit(xfer_read);
    if (err)
    {
        deallocate(xfer_read);
        ESP_LOGW("", "test write10 data: 0x%02x", err);
        _csw();
    }
    else
    {
        xTaskToNotify = xTaskGetCurrentTaskHandle();
        uint32_t pulNotificationValue;
        _csw();
        if (xTaskNotifyWait(0, 0, &pulNotificationValue, 50 / portTICK_PERIOD_MS) == pdTRUE)
        {
            return ESP_OK;
        }

        err = ESP_ERR_TIMEOUT;
        ESP_LOGE("", "test write10 data: 0x%02x", err);
    }
    return err;
}

//-------------------------------------- Private ----------------------------------//

void USBmscDevice::_emitEvent(host_event_t _event, usb_transfer_t *transfer)
{
    switch (_event)
    {
    case SCSI_CMD_MAX_LUN:
        ESP_LOGI("EMIT", "%s", "SCSI_CMD_MAX_LUN");
        _luns = transfer->data_buffer[8];
        _lun = 0;
        _getCapacity(_lun);

        if (callbacks.max_luns_cb)
        {
            callbacks.max_luns_cb(transfer);
        }
        if (event_cb)
            event_cb(event, transfer->data_buffer, transfer->actual_num_bytes);
        break;
    case SCSI_CMD_TEST_UNIT_READY:
        ESP_LOGI("EMIT", "%s", "SCSI_CMD_TEST_UNIT_READY");
        // ets_delay_us(500);
        if (xTaskToNotify)
            xTaskNotify(xTaskToNotify, 0, eNoAction);
        break;
    case SCSI_CMD_INQUIRY:
        ESP_LOG_BUFFER_HEX("EMIT", transfer->data_buffer, transfer->actual_num_bytes);

        if (callbacks.inquiry_cb)
        {
            callbacks.inquiry_cb(transfer);
        }
        break;
    case SCSI_CMD_READ_CAPACITY_10:
    {
        ESP_LOGI("EMIT", "%s", "SCSI_CMD_READ_CAPACITY_10");
        if (_lun < _luns) // all LUNS get capacity at first
        {
            _getCapacity(++_lun);
        }
        else  // then INQUIRY
        {
            _inquiry();
            if (callbacks.capacity_cb)
            {
                callbacks.capacity_cb(transfer);
            }
            if (event_cb)
                event_cb(event, transfer->data_buffer, transfer->actual_num_bytes);
        }
    }
    break;
    case SCSI_CMD_WRITE_10:
        ESP_LOGI("EMIT", "%s", "SCSI_CMD_WRITE_10");
        // ets_delay_us(500);
        if (xTaskToNotify)
            xTaskNotify(xTaskToNotify, 0, eNoAction);
        break;
    case SCSI_CMD_READ_10:
        ESP_LOGI("EMIT", "%s", "SCSI_CMD_READ_10");
        if (xTaskToNotify)
            xTaskNotify(xTaskToNotify, 0, eNoAction);
        break;

    default:
        ESP_LOGW("EMIT", "%d", _event);
        break;
    }
}

void USBmscDevice::_getMaxLUN()
{
    event = SCSI_CMD_MAX_LUN;
    MSC_SCSI_REQ_MAX_LUN((usb_setup_packet_t *)xfer_ctrl->data_buffer, itf_num);
    xfer_ctrl->num_bytes = sizeof(usb_setup_packet_t) + ((usb_setup_packet_t *)xfer_ctrl->data_buffer)->wLength;
    esp_err_t err = usb_host_transfer_submit_control(_host->clientHandle(), xfer_ctrl);
    (void) err;
}

void USBmscDevice::_getCapacity(uint8_t lun)
{
    _lun = lun;
    scsi_cmd10_t cbw = {};
    cbw.opcode = SCSI_CMD_READ_CAPACITY_10;
    event = cbw.opcode;
    usb_transfer_t* xfer_out = allocate(64);

    xfer_out->callback = usb_transfer_cb;
    xfer_out->bEndpointAddress = ep_out->bEndpointAddress;
    xfer_out->num_bytes = sizeof(msc_bulk_cbw_t);

    MSC_SCSI_INIT_CBW((msc_bulk_cbw_t *)xfer_out->data_buffer, true /*is_read*/, lun, 0x1234, cbw, 8);
    esp_err_t err = usb_host_transfer_submit(xfer_out);
    if (err)
        ESP_LOGW("", "test capacity: 0x%02x => 0x%02x\n", err, xfer_out->bEndpointAddress);

    _csw();
}

void USBmscDevice::_setCapacity(uint32_t count, uint32_t _size)
{
    block_count[_lun] = count;
    block_size[_lun] = _size;
    ESP_LOGI("", "capacity => lun: %d count %d, size: %d", _lun, block_count[_lun], block_size[_lun]);
    _csw();
}

void USBmscDevice::_csw()
{
    usb_transfer_t *transfer = allocate(block_size[0]);
    if(transfer){
        transfer->callback = usb_transfer_cb;
        transfer->bEndpointAddress = ep_in->bEndpointAddress;
        transfer->num_bytes = block_size[0];

        esp_err_t err = usb_host_transfer_submit(transfer);
        if(err)usb_host_transfer_free(transfer);
    } else {
        ESP_LOGE("", "faile to allocate CSW transfer");
    }
}

void USBmscDevice::onCSW(usb_transfer_t *transfer)
{
    _emitEvent(event, transfer);

    if (callbacks.csw_cb)
    {
        callbacks.csw_cb(transfer);
    }
    // deallocate(transfer);
}

void USBmscDevice::onCBW(usb_transfer_t *transfer)
{
    ESP_LOG_BUFFER_HEX("HEX", transfer->data_buffer, transfer->actual_num_bytes);
    if (callbacks.cbw_cb)
    {
        callbacks.cbw_cb(transfer);
    }
    // deallocate(transfer);
}

void USBmscDevice::onData(usb_transfer_t *transfer)
{
    switch (event)
    {
    case SCSI_CMD_INQUIRY:
    {
        ESP_LOG_BUFFER_HEX("HEX", transfer->data_buffer, transfer->actual_num_bytes);
        _csw();
    }
    break;
    case SCSI_CMD_READ_10:
    {
        memcpy(temp_buffer, transfer->data_buffer, block_size[_lun]);
        _csw();
    }
    break;
    case SCSI_CMD_WRITE_10:
        break;
    case SCSI_CMD_READ_CAPACITY_10:
    {
        uint32_t block_count = __builtin_bswap32(*(uint32_t *)&transfer->data_buffer[0]);
        uint32_t block_size = __builtin_bswap32(*(uint32_t *)&transfer->data_buffer[4]);

        _setCapacity(block_count, block_size);
    }
    break;

    default:
    {
        ESP_LOGE("", "event: %d", event);
        ESP_LOG_BUFFER_HEX("HEX", transfer->data_buffer, 16);
    }
    break;
    }

    if (callbacks.data_cb)
    {
        callbacks.data_cb(transfer);
    }

    if (event_cb)
        event_cb(event, transfer->data_buffer, transfer->actual_num_bytes);
    // deallocate(transfer);
}

// FIXME the same as a write10
esp_err_t USBmscDevice::_unitReady()
{
    /*
    scsi_cmd10_t cbw = {};
    cbw.opcode = SCSI_CMD_TEST_UNIT_READY;
    cbw.flags = 0;
    event = cbw.opcode;

    usb_transfer_t *xfer_out = allocate(64);
    xfer_out->callback = usb_transfer_cb;
    xfer_out->bEndpointAddress = ep_out->bEndpointAddress;
    xfer_out->num_bytes = sizeof(msc_bulk_cbw_t);

    MSC_SCSI_INIT_CBW((msc_bulk_cbw_t *)xfer_out->data_buffer, false, _lun, 0x1234, cbw, 0);

    esp_err_t err = usb_host_transfer_submit(xfer_out);

    err = usb_host_transfer_submit(xfer_out);
    ESP_LOGW("", "test unit ready data: 0x%02x", err);

    if (err)
    {
        ESP_LOGW("", "test unit ready data: 0x%02x", err);
        _csw();
    }
    else
    {
        xTaskToNotify = xTaskGetCurrentTaskHandle();
        uint32_t pulNotificationValue;
        _csw();

        if (xTaskNotifyWait(0, 0, &pulNotificationValue, 100) == pdTRUE)
        {
            return ESP_OK;
        }

        ESP_LOGE("", "test unit ready data: 0x%02x", ESP_FAIL);
    }
    */
    return ESP_FAIL;
}

void USBmscDevice::_inquiry()
{
    scsi_cmd10_t cbw = {};
    cbw.opcode = SCSI_CMD_INQUIRY;
    cbw.lba_1 = 36;

    event = cbw.opcode;
    usb_transfer_t *xfer_out = allocate(64);
    xfer_out->callback = usb_transfer_cb;
    xfer_out->bEndpointAddress = ep_out->bEndpointAddress;
    xfer_out->num_bytes = sizeof(msc_bulk_cbw_t);
    MSC_SCSI_INIT_CBW((msc_bulk_cbw_t *)xfer_out->data_buffer, true /*is_read*/, 0, 0x1234, cbw, 36);

    esp_err_t err = usb_host_transfer_submit(xfer_out);
    if (err)
        ESP_LOGW("", "inquiry: 0x%02x => 0x%02x\n", err, xfer_out->bEndpointAddress);

    // xfer_in->callback = usb_transfer_cb; // TODO separate cb
    _csw();
}



