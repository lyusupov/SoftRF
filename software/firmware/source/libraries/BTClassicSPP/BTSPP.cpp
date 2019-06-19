// Copyright 2018 Evandro Luis Copercini
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "sdkconfig.h"
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#if defined(CONFIG_BT_ENABLED) && defined(CONFIG_BLUEDROID_ENABLED)

#ifdef ARDUINO_ARCH_ESP32
#include "esp32-hal-log.h"
#endif

#include "BTSPP.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"
#include <esp_log.h>

#ifdef ARDUINO_ARCH_ESP32
#include "esp32-hal-log.h"
#endif

const char* _spp_server_name = "ESP32SPP";

#define RX_QUEUE_SIZE 512
#define TX_QUEUE_SIZE 32
static uint32_t _spp_client = 0;
static xQueueHandle _spp_rx_queue = NULL;
static xQueueHandle _spp_tx_queue = NULL;
static SemaphoreHandle_t _spp_tx_done = NULL;
static TaskHandle_t _spp_task_handle = NULL;
static EventGroupHandle_t _spp_event_group = NULL;
static boolean secondConnectionAttempt;
static esp_spp_cb_t* custom_spp_callback = NULL;

static esp_spp_role_t bt_role = ESP_SPP_ROLE_SLAVE;
char peer_bdname[ESP_BT_GAP_MAX_BDNAME_LEN + 1];
static uint8_t peer_bdname_len;
esp_bd_addr_t peer_bdaddress;
uint8_t       channel;

static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_NONE;

#define SPP_RUNNING		   0x01
#define SPP_CONNECTED	   0x02
#define SPP_CONGESTED	   0x04
#define SPP_DISCOVERED     0x08

typedef struct {
	size_t len;
	uint8_t data[];
} spp_packet_t;

static esp_err_t _spp_queue_packet (uint8_t* data, size_t len) {
	if (!data || !len) {
		log_w ("No data provided");
		return ESP_OK;
	}
	spp_packet_t* packet = (spp_packet_t*)malloc (sizeof (spp_packet_t) + len);
	if (!packet) {
		log_e ("SPP TX Packet Malloc Failed!");
		return ESP_FAIL;
	}
	packet->len = len;
	memcpy (packet->data, data, len);
	if (xQueueSend (_spp_tx_queue, &packet, portMAX_DELAY) != pdPASS) {
		log_e ("SPP TX Queue Send Failed!");
		free (packet);
		return ESP_FAIL;
	}
	return ESP_OK;
}

const uint16_t SPP_TX_MAX = 330;
static uint8_t _spp_tx_buffer[SPP_TX_MAX];
static uint16_t _spp_tx_buffer_len = 0;

static bool _spp_send_buffer () {
	if ((xEventGroupWaitBits (_spp_event_group, SPP_CONGESTED, pdFALSE, pdTRUE, portMAX_DELAY) & SPP_CONGESTED)) {
		esp_err_t err = esp_spp_write (_spp_client, _spp_tx_buffer_len, _spp_tx_buffer);
		if (err != ESP_OK) {
			log_e ("SPP Write Failed! [0x%X]", err);
			return false;
		}
		_spp_tx_buffer_len = 0;
		if (xSemaphoreTake (_spp_tx_done, portMAX_DELAY) != pdTRUE) {
			log_e ("SPP Ack Failed!");
			return false;
		}
		return true;
	}
	return false;
}

static void _spp_tx_task (void* arg) {
	spp_packet_t* packet = NULL;
	size_t len = 0, to_send = 0;
	uint8_t* data = NULL;
	for (;;) {
		if (_spp_tx_queue && xQueueReceive (_spp_tx_queue, &packet, portMAX_DELAY) == pdTRUE && packet) {
			if (packet->len <= (SPP_TX_MAX - _spp_tx_buffer_len)) {
				memcpy (_spp_tx_buffer + _spp_tx_buffer_len, packet->data, packet->len);
				_spp_tx_buffer_len += packet->len;
				free (packet);
				packet = NULL;
				if (SPP_TX_MAX == _spp_tx_buffer_len || uxQueueMessagesWaiting (_spp_tx_queue) == 0) {
					_spp_send_buffer ();
				}
			}
			else {
				len = packet->len;
				data = packet->data;
				to_send = SPP_TX_MAX - _spp_tx_buffer_len;
				memcpy (_spp_tx_buffer + _spp_tx_buffer_len, data, to_send);
				_spp_tx_buffer_len = SPP_TX_MAX;
				data += to_send;
				len -= to_send;
				_spp_send_buffer ();
				while (len >= SPP_TX_MAX) {
					memcpy (_spp_tx_buffer, data, SPP_TX_MAX);
					_spp_tx_buffer_len = SPP_TX_MAX;
					data += SPP_TX_MAX;
					len -= SPP_TX_MAX;
					_spp_send_buffer ();
				}
				if (len) {
					memcpy (_spp_tx_buffer, data, len);
					_spp_tx_buffer_len += len;
					free (packet);
					packet = NULL;
					if (uxQueueMessagesWaiting (_spp_tx_queue) == 0) {
						_spp_send_buffer ();
					}
				}
			}
		}
		else {
			log_e ("Something went horribly wrong");
		}
	}
	vTaskDelete (NULL);
	_spp_task_handle = NULL;
}


static void esp_spp_cb (esp_spp_cb_event_t event, esp_spp_cb_param_t* param)
{
	switch (event)
	{
	case ESP_SPP_INIT_EVT:
		log_i ("ESP_SPP_INIT_EVT");
		esp_bt_gap_set_scan_mode (ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
		esp_spp_start_srv (ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE, 0, _spp_server_name);
		xEventGroupSetBits (_spp_event_group, SPP_RUNNING);
		break;

	case ESP_SPP_SRV_OPEN_EVT://Server connection open
		if (!_spp_client) {
			_spp_client = param->open.handle;
		}
		else {
			secondConnectionAttempt = true;
			esp_spp_disconnect (param->open.handle);
		}
		xEventGroupSetBits (_spp_event_group, SPP_CONNECTED);
		log_i ("ESP_SPP_SRV_OPEN_EVT");
		break;

	case ESP_SPP_CLOSE_EVT://Client connection closed
		log_i ("ESP_SPP_CLOSE_EVT %d", param->close.status);
		if (secondConnectionAttempt) {
			secondConnectionAttempt = false;
		}
		else {
			_spp_client = 0;
		}
		xEventGroupClearBits (_spp_event_group, SPP_CONNECTED);
		if (!bt_role) { // If master
			if (esp_spp_disconnect (param->close.handle) == ESP_OK) {
				log_i ("ESP_SPP disconnected");
				if (esp_spp_start_discovery (peer_bdaddress) == ESP_OK) {
					log_i ("ESP_SPP discovery restarted");
				}
			}
			else {
				log_w ("Error disconnecting from %s", peer_bdname);
			}
		}
		break;

	case ESP_SPP_CONG_EVT://connection congestion status changed
		if (param->cong.cong) {
			xEventGroupClearBits (_spp_event_group, SPP_CONGESTED);
		}
		else {
			xEventGroupSetBits (_spp_event_group, SPP_CONGESTED);
		}
		log_v ("ESP_SPP_CONG_EVT: %s", param->cong.cong ? "CONGESTED" : "FREE");
		break;

	case ESP_SPP_WRITE_EVT://write operation completed
		if (param->write.cong) {
			xEventGroupClearBits (_spp_event_group, SPP_CONGESTED);
		}
		xSemaphoreGive (_spp_tx_done);//we can try to send another packet
		log_v ("ESP_SPP_WRITE_EVT: %u %s", param->write.len, param->write.cong ? "CONGESTED" : "FREE");
		break;

	case ESP_SPP_DATA_IND_EVT://connection received data
		log_v ("ESP_SPP_DATA_IND_EVT len=%d handle=%d", param->data_ind.len, param->data_ind.handle);
		//esp_log_buffer_hex("",param->data_ind.data,param->data_ind.len); //for low level debug
		//ets_printf("r:%u\n", param->data_ind.len);

		if (_spp_rx_queue != NULL) {
			for (int i = 0; i < param->data_ind.len; i++) {
				if (xQueueSend (_spp_rx_queue, param->data_ind.data + i, (TickType_t)0) != pdTRUE) {
					log_e ("RX Full! Discarding %u bytes", param->data_ind.len - i);
					break;
				}
			}
		}
		break;

		//should maybe delete those.
	case ESP_SPP_DISCOVERY_COMP_EVT://discovery complete
		log_i ("ESP_SPP_DISCOVERY_COMP_EVT %d Channel %d", param->disc_comp.status, param->disc_comp.scn[0]);
		if (param->disc_comp.status == ESP_SPP_SUCCESS) {
			channel = param->disc_comp.scn[0];
			esp_spp_connect (sec_mask, bt_role, channel, peer_bdaddress);
		}
		else {
			esp_spp_start_discovery (peer_bdaddress);
			log_i ("Started ESP_SPP discovery");
		}
		break;
	case ESP_SPP_OPEN_EVT://Client connection open
		if (!_spp_client) {
			_spp_client = param->open.handle;
		}
		else {
			secondConnectionAttempt = true;
			esp_spp_disconnect (param->open.handle);
		}
		xEventGroupSetBits (_spp_event_group, SPP_CONNECTED);
		log_i ("ESP_SPP_OPEN_EVT");
		break;
	case ESP_SPP_START_EVT://server started
		log_i ("ESP_SPP_START_EVT");
		break;
	case ESP_SPP_CL_INIT_EVT://client initiated a connection
		log_i ("ESP_SPP_CL_INIT_EVT %d", param->cl_init.status);
		if (!bt_role) { // If master
			if (param->cl_init.status == ESP_SPP_FAILURE) {
				if (esp_spp_connect (sec_mask, bt_role, channel, peer_bdaddress) == ESP_OK) {
					log_i ("Start new connection attempt");
				}
				else {
					log_w ("Error connecting to %s", peer_bdname);
				}
			}
		}
		break;
	default:
		break;
	}
	if (custom_spp_callback)(*custom_spp_callback)(event, param);
}

static bool get_name_from_eir (uint8_t* eir, char* bdname, uint8_t* bdname_len)
{
	uint8_t* rmt_bdname = NULL;
	uint8_t rmt_bdname_len = 0;

	if (!eir) {
		return false;
	}

	rmt_bdname = esp_bt_gap_resolve_eir_data (eir, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &rmt_bdname_len);
	if (!rmt_bdname) {
		rmt_bdname = esp_bt_gap_resolve_eir_data (eir, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &rmt_bdname_len);
	}

	if (rmt_bdname) {
		if (rmt_bdname_len > ESP_BT_GAP_MAX_BDNAME_LEN) {
			rmt_bdname_len = ESP_BT_GAP_MAX_BDNAME_LEN;
		}

		if (bdname) {
			memcpy (bdname, rmt_bdname, rmt_bdname_len);
			bdname[rmt_bdname_len] = '\0';
		}
		if (bdname_len) {
			*bdname_len = rmt_bdname_len;
		}
		return true;
	}

	return false;
}

static void esp_bt_gap_cb (esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t* param)
{
	static char temp_bdname[ESP_BT_GAP_MAX_BDNAME_LEN + 1];
	static uint8_t temp_bdname_len;

	switch (event) {
	case ESP_BT_GAP_DISC_RES_EVT:
		log_i ("ESP_BT_GAP_DISC_RES_EVT");
		esp_log_buffer_hex ("V", param->disc_res.bda, ESP_BD_ADDR_LEN);
		for (int i = 0; i < param->disc_res.num_prop; i++) {
			if (param->disc_res.prop[i].type == ESP_BT_GAP_DEV_PROP_BDNAME) {
				strcpy (temp_bdname, (char*)(param->disc_res.prop[i].val));
				log_i ("%d ESP_BT_GAP_DEV_PROP_BDNAME: %s", millis (), temp_bdname);
				log_i ("strlen (temp_bdname) = %d, peer_bdname_len = %d", strlen (temp_bdname), peer_bdname_len);
				if (strlen (temp_bdname) == peer_bdname_len
					&& strncmp (temp_bdname, peer_bdname, peer_bdname_len) == 0) {
					xEventGroupSetBits (_spp_event_group, SPP_DISCOVERED);
					log_i ("SPP_DISCOVERED = %d", (xEventGroupGetBits (_spp_event_group) & SPP_DISCOVERED) == SPP_DISCOVERED);
					memcpy (peer_bdaddress, param->disc_res.bda, ESP_BD_ADDR_LEN);
					esp_spp_start_discovery (peer_bdaddress);
					esp_bt_gap_cancel_discovery ();
					log_i ("Started ESP_SPP discovery");
				}
			}
			else if (param->disc_res.prop[i].type == ESP_BT_GAP_DEV_PROP_EIR
				&& get_name_from_eir ((uint8_t*)(param->disc_res.prop[i].val), temp_bdname, &temp_bdname_len)) {
				log_i ("ESP_BT_GAP_DEV_PROP_EIR %s", temp_bdname);
				esp_log_buffer_char ("V", temp_bdname, peer_bdname_len);
				if (strlen (temp_bdname) == peer_bdname_len
					&& strncmp (temp_bdname, peer_bdname, peer_bdname_len) == 0) {
					xEventGroupSetBits (_spp_event_group, SPP_DISCOVERED);
					memcpy (peer_bdaddress, param->disc_res.bda, ESP_BD_ADDR_LEN);
					esp_spp_start_discovery (peer_bdaddress);
					esp_bt_gap_cancel_discovery ();
					log_i ("Started GAP_SPP discovery");
				}
			}


		}
		break;
	case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
		log_i ("ESP_BT_GAP_DISC_STATE_CHANGED_EVT %d", param->disc_st_chg.state);
		log_i ("SPP_DISCOVERED = %d", (xEventGroupGetBits (_spp_event_group) & SPP_DISCOVERED) == SPP_DISCOVERED);
		if (!bt_role && ((xEventGroupGetBits (_spp_event_group) & SPP_DISCOVERED) != SPP_DISCOVERED)) {
			if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
				uint8_t period = 10 / 1.28;
				if (esp_bt_gap_start_discovery (ESP_BT_INQ_MODE_GENERAL_INQUIRY, period, 0) != ESP_OK) {
					log_e ("%s start discovery failed", __func__);
				}
				else {
					log_e ("GAP discovery started");
				}
			}
		}
		break;
	case ESP_BT_GAP_RMT_SRVCS_EVT:
		log_i ("ESP_BT_GAP_RMT_SRVCS_EVT");
		break;
	case ESP_BT_GAP_RMT_SRVC_REC_EVT:
		log_i ("ESP_BT_GAP_RMT_SRVC_REC_EVT");
		break;
	case ESP_BT_GAP_AUTH_CMPL_EVT: {
		if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
			log_i ("authentication success: %s", param->auth_cmpl.device_name);
			esp_log_buffer_hex (NULL, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
		}
		else {
			log_e ("authentication failed, status:%d", param->auth_cmpl.stat);
		}
		break;
	}
	case ESP_BT_GAP_PIN_REQ_EVT: {
		log_i ("ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
		if (param->pin_req.min_16_digit) {
			ESP_LOGI (SPP_TAG, "Input pin code: 0000 0000 0000 0000");
			esp_bt_pin_code_t pin_code = { 0 };
			esp_bt_gap_pin_reply (param->pin_req.bda, true, 16, pin_code);
		}
		else {
			ESP_LOGI (SPP_TAG, "Input pin code: 1234");
			esp_bt_pin_code_t pin_code;
			pin_code[0] = '1';
			pin_code[1] = '2';
			pin_code[2] = '3';
			pin_code[3] = '4';
			esp_bt_gap_pin_reply (param->pin_req.bda, true, 4, pin_code);
		}
		break;
	}
	}
}

bool check_null_address (esp_bd_addr_t address) {
	for (int i = 0; i < ESP_BD_ADDR_LEN; i++) {
		if (address[i] != 0) {
			return false;
		}
	}
	return true;
}

static bool _init_bt (const char* deviceName)
{
	if (!_spp_event_group) {
		_spp_event_group = xEventGroupCreate ();
		if (!_spp_event_group) {
			log_e ("SPP Event Group Create Failed!");
			return false;
		}
		xEventGroupClearBits (_spp_event_group, 0xFFFFFF);
		xEventGroupSetBits (_spp_event_group, SPP_CONGESTED);
	}
	if (_spp_rx_queue == NULL) {
		_spp_rx_queue = xQueueCreate (RX_QUEUE_SIZE, sizeof (uint8_t)); //initialize the queue
		if (_spp_rx_queue == NULL) {
			log_e ("RX Queue Create Failed");
			return false;
		}
	}
	if (_spp_tx_queue == NULL) {
		_spp_tx_queue = xQueueCreate (TX_QUEUE_SIZE, sizeof (spp_packet_t*)); //initialize the queue
		if (_spp_tx_queue == NULL) {
			log_e ("TX Queue Create Failed");
			return false;
		}
	}
	if (_spp_tx_done == NULL) {
		_spp_tx_done = xSemaphoreCreateBinary ();
		if (_spp_tx_done == NULL) {
			log_e ("TX Semaphore Create Failed");
			return false;
		}
		xSemaphoreTake (_spp_tx_done, 0);
	}

	if (!_spp_task_handle) {
		xTaskCreate (_spp_tx_task, "spp_tx", 4096, NULL, 2, &_spp_task_handle);
		if (!_spp_task_handle) {
			log_e ("Network Event Task Start Failed!");
			return false;
		}
	}

	if (!btStarted () && !btStart ()) {
		log_e ("initialize controller failed");
		return false;
	}

	esp_bluedroid_status_t bt_state = esp_bluedroid_get_status ();
	if (bt_state == ESP_BLUEDROID_STATUS_UNINITIALIZED) {
		if (esp_bluedroid_init ()) {
			log_e ("initialize bluedroid failed");
			return false;
		}
	}

	if (bt_state != ESP_BLUEDROID_STATUS_ENABLED) {
		if (esp_bluedroid_enable ()) {
			log_e ("enable bluedroid failed");
			return false;
		}
	}

	if (esp_spp_register_callback (esp_spp_cb) != ESP_OK) {
		log_e ("spp register failed");
		return false;
	}

	if (esp_spp_init (ESP_SPP_MODE_CB) != ESP_OK) {
		log_e ("spp init failed");
		return false;
	}

	if (!bt_role) { // If master
		if (check_null_address (peer_bdaddress)) {
			if ((esp_bt_gap_register_callback (esp_bt_gap_cb)) != ESP_OK) {
				log_e ("GAP register failed");
				return false;
			}
			log_i ("GAP callback registered");
			log_i ("peer_bdaddress: %02x:%02x:%02x:%02x:%02x:%02x",
				peer_bdaddress[0], peer_bdaddress[1], peer_bdaddress[2], peer_bdaddress[3], peer_bdaddress[4], peer_bdaddress[5]);

			uint8_t period = 10 / 1.28;

			if (esp_bt_gap_start_discovery (ESP_BT_INQ_MODE_GENERAL_INQUIRY, period, 0) != ESP_OK) {
				log_e ("Start discovery failed");
				return false;
			}
			log_d ("GAP discovery started");
		}
		else {
			if (esp_spp_start_discovery (peer_bdaddress) != ESP_OK) {
				log_e ("Start SPP discovery failed");
				return false;
			}
			else {
				log_i ("Started GAP_SPP discovery");
			}
		}

	}

	esp_bt_dev_set_device_name (deviceName);

	// the default BTA_DM_COD_LOUDSPEAKER does not work with the macOS BT stack
	esp_bt_cod_t cod;
	cod.major = 0b00001;
	cod.minor = 0b000100;
	cod.service = 0b00000010110;
	if (esp_bt_gap_set_cod (cod, ESP_BT_INIT_COD) != ESP_OK) {
		log_e ("set cod failed");
		return false;
	}

	return true;
}

static bool _stop_bt ()
{
	if (btStarted ()) {
		if (_spp_client)
			esp_spp_disconnect (_spp_client);
		esp_spp_deinit ();
		esp_bluedroid_disable ();
		esp_bluedroid_deinit ();
		btStop ();
	}
	_spp_client = 0;
	if (_spp_task_handle) {
		vTaskDelete (_spp_task_handle);
		_spp_task_handle = NULL;
	}
	if (_spp_event_group) {
		vEventGroupDelete (_spp_event_group);
		_spp_event_group = NULL;
	}
	if (_spp_rx_queue) {
		vQueueDelete (_spp_rx_queue);
		//ToDo: clear RX queue when in packet mode
		_spp_rx_queue = NULL;
	}
	if (_spp_tx_queue) {
		spp_packet_t* packet = NULL;
		while (xQueueReceive (_spp_tx_queue, &packet, 0) == pdTRUE) {
			free (packet);
		}
		vQueueDelete (_spp_tx_queue);
		_spp_tx_queue = NULL;
	}
	if (_spp_tx_done) {
		vSemaphoreDelete (_spp_tx_done);
		_spp_tx_done = NULL;
	}
	return true;
}

/*
 * Serial Bluetooth Arduino
 *
 * */

BluetoothSerial::BluetoothSerial ()
{
	local_name = "ESP32"; //default bluetooth name
}

BluetoothSerial::~BluetoothSerial (void)
{
	_stop_bt ();
}

bool BluetoothSerial::begin (String localName, esp_spp_role_t role, String bda_name, esp_bd_addr_t destAddress)
{
	if (localName.length ()) {
		local_name = localName;
	}
	bt_role = role;
	log_v ("Role set to %s", role ? "slave" : "master");
	if (!role) { // if master
		if (bda_name.length () == 0 && destAddress == 0) {
			log_e ("Slave address not set");
			return false;
		}
		//slave_name = bda_name;
		bda_name.getBytes ((uint8_t*)peer_bdname, ESP_BT_GAP_MAX_BDNAME_LEN);
		peer_bdname_len = bda_name.length ();
		if (destAddress) {
			log_i ("BDA address for slave set to %02x:%02x:%02x:%02x:%02x:%02x",
				destAddress[0], destAddress[1], destAddress[2], destAddress[3], destAddress[4], destAddress[5], destAddress[6]);
		}
		else if (peer_bdname_len > 0) {
			log_i ("BDA name for slave set to %s", peer_bdname);
		}

		if (destAddress) {
			for (int i = 0; i < ESP_BD_ADDR_LEN; i++) {
				slave_address[i] = destAddress[i];
				peer_bdaddress[i] = destAddress[i];
			}
			log_v ("Slave address set");
		}
	}

	return _init_bt (local_name.c_str ());
}

int BluetoothSerial::available (void)
{
	if (_spp_rx_queue == NULL) {
		return 0;
	}
	return uxQueueMessagesWaiting (_spp_rx_queue);
}

int BluetoothSerial::peek (void)
{
	uint8_t c;
	if (_spp_rx_queue && xQueuePeek (_spp_rx_queue, &c, 0)) {
		return c;
	}
	return -1;
}

bool BluetoothSerial::hasClient (void)
{
	return _spp_client > 0;
}

int BluetoothSerial::read (void)
{

	uint8_t c = 0;
	if (_spp_rx_queue && xQueueReceive (_spp_rx_queue, &c, 0)) {
		return c;
	}
	return -1;
}

size_t BluetoothSerial::write (uint8_t c)
{
	return write (&c, 1);
}

size_t BluetoothSerial::write (const uint8_t* buffer, size_t size)
{
	if (!_spp_client) {
		return 0;
	}
	return (_spp_queue_packet ((uint8_t*)buffer, size) == ESP_OK) ? size : 0;
}

void BluetoothSerial::flush ()
{
	while (read () >= 0) {}
}

void BluetoothSerial::end ()
{
	_stop_bt ();
}

esp_err_t BluetoothSerial::register_callback (esp_spp_cb_t* callback)
{
	custom_spp_callback = callback;
	return ESP_OK;
}

#endif
