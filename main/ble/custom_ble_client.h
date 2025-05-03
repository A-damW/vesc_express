/*  
	Copyright 2024  
  
	This file is part of the VESC firmware.  
  
	The VESC firmware is free software: you can redistribute it and/or modify  
	it under the terms of the GNU General Public License as published by  
	the Free Software Foundation, either version 3 of the License, or  
	(at your option) any later version.  
  
	The VESC firmware is distributed in the hope that it will be useful,  
	but WITHOUT ANY WARRANTY; without even the implied warranty of  
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the  
	GNU General Public License for more details.  
  
	You should have received a copy of the GNU General Public License  
	along with this program.  If not, see <http://www.gnu.org/licenses/>.  
*/  
  
#ifndef MAIN_BLE_BLE_CLIENT_H_  
#define MAIN_BLE_BLE_CLIENT_H_  
  
#include <stdint.h>  
#include <stdlib.h>  
#include <stdbool.h>  
  
#include "esp_bt_defs.h"  
#include "esp_gatt_defs.h"  
#include "esp_gattc_api.h"  
#include "esp_gap_ble_api.h"  
  
// Maximum number of discovered devices that can be stored  
#define BLE_CLIENT_MAX_SCAN_DEVICES 20  
// Maximum number of services that can be discovered per device  
#define BLE_CLIENT_MAX_SERVICES 20  
// Maximum number of characteristics that can be discovered per service  
#define BLE_CLIENT_MAX_CHARS 20  
// Maximum number of descriptors that can be discovered per characteristic  
#define BLE_CLIENT_MAX_DESCRIPTORS 5  
// Timeout for operations in milliseconds  
#define BLE_CLIENT_OPERATION_TIMEOUT 10000  
  
// Scan duration in seconds (0 = scan indefinitely until stop)  
#define BLE_CLIENT_DEFAULT_SCAN_DURATION 5  
// Default MTU size  
#define BLE_CLIENT_DEFAULT_MTU 23  
  
typedef enum {  
    BLE_CLIENT_OK = 0,  
    BLE_CLIENT_ERROR = 1,  
    BLE_CLIENT_ESP_ERROR = 2,  
    BLE_CLIENT_TIMEOUT = 3,  
    BLE_CLIENT_INVALID_STATE = 4,  
    BLE_CLIENT_NOT_FOUND = 5,  
    BLE_CLIENT_NOT_CONNECTED = 6,  
    BLE_CLIENT_ALREADY_CONNECTED = 7,  
    BLE_CLIENT_SCAN_BUSY = 8,  
    BLE_CLIENT_DISCOVERY_BUSY = 9,  
    BLE_CLIENT_INVALID_HANDLE = 10,  
    BLE_CLIENT_MEMORY_ERROR = 11,  
} ble_client_result_t;  
  
typedef struct {  
    esp_bd_addr_t bda;  
    char name[32];  
    int8_t rssi;  
    esp_ble_addr_type_t addr_type;  
    bool has_name;  
} ble_client_scan_device_t;  
  
typedef struct {  
    uint16_t handle;  
    esp_bt_uuid_t uuid;  
    uint16_t start_handle;  
    uint16_t end_handle;  
} ble_client_service_t;  
  
typedef struct {  
    uint16_t handle;  
    esp_bt_uuid_t uuid;  
    esp_gatt_char_prop_t properties;  
} ble_client_char_t;  
  
typedef struct {  
    uint16_t handle;  
    esp_bt_uuid_t uuid;  
} ble_client_descr_t;  
  
// Callback for scan results  
typedef void (*ble_client_scan_cb_t)(ble_client_scan_device_t *device, void *user_data);  
  
// Callback for connection events  
typedef void (*ble_client_connect_cb_t)(bool connected, void *user_data);  
  
// Callback for characteristic notifications/indications  
typedef void (*ble_client_notify_cb_t)(  
    uint16_t char_handle, const uint8_t *data, size_t data_len, bool is_notify, void *user_data  
);  
  
// Initialize the BLE client  
void ble_client_init(void);  
  
// Start scanning for BLE devices  
ble_client_result_t ble_client_scan_start(uint32_t duration_s, ble_client_scan_cb_t callback, void *user_data);  
  
// Stop scanning for BLE devices  
ble_client_result_t ble_client_scan_stop(void);  
  
// Get the list of discovered devices  
ble_client_result_t ble_client_get_scan_devices(ble_client_scan_device_t *devices, uint16_t *count);  
  
// Connect to a BLE device  
ble_client_result_t ble_client_connect(esp_bd_addr_t bda, esp_ble_addr_type_t addr_type, ble_client_connect_cb_t callback, void *user_data);  
  
// Disconnect from the current BLE device  
ble_client_result_t ble_client_disconnect(void);  
  
// Check if connected to a device  
bool ble_client_is_connected(void);  
  
// Discover all services on the connected device  
ble_client_result_t ble_client_discover_services(void);  
  
// Get the list of discovered services  
ble_client_result_t ble_client_get_services(ble_client_service_t *services, uint16_t *count);  
  
// Discover characteristics for a specific service  
ble_client_result_t ble_client_discover_characteristics(uint16_t service_handle);  
  
// Get the list of discovered characteristics for a service  
ble_client_result_t ble_client_get_characteristics(uint16_t service_handle, ble_client_char_t *chars, uint16_t *count);  
  
// Discover descriptors for a specific characteristic  
ble_client_result_t ble_client_discover_descriptors(uint16_t char_handle);  
  
// Get the list of discovered descriptors for a characteristic  
ble_client_result_t ble_client_get_descriptors(uint16_t char_handle, ble_client_descr_t *descrs, uint16_t *count);  
  
// Read a characteristic value  
ble_client_result_t ble_client_read_characteristic(uint16_t char_handle, uint8_t **data, uint16_t *data_len);  
  
// Write a characteristic value  
ble_client_result_t ble_client_write_characteristic(uint16_t char_handle, const uint8_t *data, uint16_t data_len, bool response);  
  
// Read a descriptor value  
ble_client_result_t ble_client_read_descriptor(uint16_t descr_handle, uint8_t **data, uint16_t *data_len);  
  
// Write a descriptor value  
ble_client_result_t ble_client_write_descriptor(uint16_t descr_handle, const uint8_t *data, uint16_t data_len);  
  
// Register for notifications/indications from a characteristic  
ble_client_result_t ble_client_register_for_notify(uint16_t char_handle, ble_client_notify_cb_t callback, void *user_data);  
  
// Unregister from notifications/indications from a characteristic  
ble_client_result_t ble_client_unregister_for_notify(uint16_t char_handle);  
  
#endif /* MAIN_BLE_BLE_CLIENT_H_ */
