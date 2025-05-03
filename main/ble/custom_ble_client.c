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
  
#include "custom_ble_client.h"  
  
#include <stdbool.h>  
#include <stdio.h>  
#include <stdlib.h>  
#include <stdint.h>  
#include <string.h>  
  
#include "freertos/FreeRTOS.h"  
#include "freertos/event_groups.h"  
#include "freertos/task.h"  
#include "freertos/semphr.h"  
  
#include "esp_bt.h"  
#include "esp_bt_defs.h"  
#include "esp_bt_device.h"  
#include "esp_bt_main.h"  
#include "esp_gap_ble_api.h"  
#include "esp_gatt_defs.h"  
#include "esp_gattc_api.h"  
#include "esp_log.h"  
#include "esp_system.h"  
  
#include "commands.h"  
#include "utils.h"  
  
//#define STORED_LOGF commands_printf  
//#define BLE_CLIENT_TAG "BLE_CLIENT"  
  
static const char* LOG_TAG = BLE_CLIENT_TAG;  
  
// Global variables  
static bool ble_client_initialized = false;  
static esp_gatt_if_t gattc_if = ESP_GATT_IF_NONE;  
static bool is_scanning = false;  
static bool is_connected = false;  
static bool is_discovering_services = false;  
static bool is_discovering_chars = false;  
static bool is_discovering_descrs = false;  
static bool is_reading = false;  
static bool is_writing = false;  
  
static esp_bd_addr_t connected_device_addr;  
static uint16_t conn_id = 0;  
  
// Scan results  
static ble_client_scan_device_t scan_devices[BLE_CLIENT_MAX_SCAN_DEVICES];  
static uint16_t scan_device_count = 0;  
static ble_client_scan_cb_t scan_callback = NULL;  
static void *scan_user_data = NULL;  
  
// Connection callback  
static ble_client_connect_cb_t connect_callback = NULL;  
static void *connect_user_data = NULL;  
  
// Service discovery results  
static ble_client_service_t discovered_services[BLE_CLIENT_MAX_SERVICES];  
static uint16_t discovered_service_count = 0;  
  
// Characteristic discovery results  
static ble_client_char_t discovered_chars[BLE_CLIENT_MAX_CHARS];  
static uint16_t discovered_char_count = 0;  
static uint16_t current_service_handle = 0;  
  
// Descriptor discovery results  
static ble_client_descr_t discovered_descrs[BLE_CLIENT_MAX_DESCRIPTORS];  
static uint16_t discovered_descr_count = 0;  
static uint16_t current_char_handle = 0;  
  
// Notify callbacks  
typedef struct {  
    uint16_t char_handle;  
    ble_client_notify_cb_t callback;  
    void *user_data;  
    bool active;  
} notify_cb_t;  
  
static notify_cb_t notify_callbacks[BLE_CLIENT_MAX_CHARS];  
static uint16_t notify_callback_count = 0;  
  
// Read/write operation results  
static uint8_t *read_data = NULL;  
static uint16_t read_data_len = 0;  
static esp_gatt_status_t operation_status = ESP_GATT_OK;  
  
// Synchronization primitives  
static SemaphoreHandle_t operation_mutex = NULL;  
static SemaphoreHandle_t operation_complete_sem = NULL;  
  
// Forward declarations of helper functions  
static void reset_scan_results(void);  
static int find_scan_device_by_addr(esp_bd_addr_t addr);  
static void reset_service_discovery(void);  
static void reset_char_discovery(void);  
static void reset_descriptor_discovery(void);  
static void reset_notify_callbacks(void);  
static bool bdaddr_equals(esp_bd_addr_t a, esp_bd_addr_t b);  
static int find_service_by_handle(uint16_t handle);  
static int find_char_by_handle(uint16_t handle);  
static int find_descr_by_handle(uint16_t handle);  
static int find_notify_cb_by_handle(uint16_t handle);  
static bool wait_for_operation_complete(uint32_t timeout_ms);  
static const char* bd_addr_to_str(esp_bd_addr_t addr);  
  
// GATTC event handler  
static void gattc_event_handler(  
    esp_gattc_cb_event_t event,  
    esp_gatt_if_t gattc_if,  
    esp_ble_gattc_cb_param_t *param  
);  
  
// GAP event handler  
static void gap_event_handler(  
    esp_gap_ble_cb_event_t event,   
    esp_ble_gap_cb_param_t *param  
);  
  
void ble_client_init(void) {  
    if (ble_client_initialized) {  
        return;  
    }  
  
    // Initialize mutexes and semaphores  
    operation_mutex = xSemaphoreCreateMutex();  
    operation_complete_sem = xSemaphoreCreateBinary();  
      
    if (!operation_mutex || !operation_complete_sem) {  
        STORED_LOGF("Failed to create synchronization primitives");  
        return;  
    }  
  
    // Register callbacks  
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));  
    ESP_ERROR_CHECK(esp_ble_gattc_register_callback(gattc_event_handler));  
    ESP_ERROR_CHECK(esp_ble_gattc_app_register(0));  
  
    // Initialize data structures  
    reset_scan_results();  
    reset_service_discovery();  
    reset_char_discovery();  
    reset_descriptor_discovery();  
    reset_notify_callbacks();  
  
    ble_client_initialized = true;  
    STORED_LOGF("BLE client initialized");  
}  
  
ble_client_result_t ble_client_scan_start(uint32_t duration_s, ble_client_scan_cb_t callback, void *user_data) {  
    if (!ble_client_initialized) {  
        return BLE_CLIENT_INVALID_STATE;  
    }  
  
    if (xSemaphoreTake(operation_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {  
        return BLE_CLIENT_TIMEOUT;  
    }  
  
    if (is_scanning) {  
        xSemaphoreGive(operation_mutex);  
        return BLE_CLIENT_SCAN_BUSY;  
    }  
  
    // Reset scan results  
    reset_scan_results();  
      
    // Store callback  
    scan_callback = callback;  
    scan_user_data = user_data;  
  
    // Configure scan parameters  
    esp_ble_scan_params_t scan_params = {  
        .scan_type = BLE_SCAN_TYPE_ACTIVE,  
        .own_addr_type = BLE_ADDR_TYPE_PUBLIC,  
        .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,  
        .scan_interval = 0x50,  // 50 ms  
        .scan_window = 0x30,    // 30 ms  
        .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE  
    };  
  
    esp_err_t ret = esp_ble_gap_set_scan_params(&scan_params);  
    if (ret != ESP_OK) {  
        STORED_LOGF("Set scan params error: %d", ret);  
        xSemaphoreGive(operation_mutex);  
        return BLE_CLIENT_ESP_ERROR;  
    }  
  
    // Start scanning  
    ret = esp_ble_gap_start_scanning(duration_s);  
    if (ret != ESP_OK) {  
        STORED_LOGF("Start scanning error: %d", ret);  
        xSemaphoreGive(operation_mutex);  
        return BLE_CLIENT_ESP_ERROR;  
    }  
  
    is_scanning = true;  
    xSemaphoreGive(operation_mutex);  
    return BLE_CLIENT_OK;  
}  
  
ble_client_result_t ble_client_scan_stop(void) {  
    if (!ble_client_initialized) {  
        return BLE_CLIENT_INVALID_STATE;  
    }  
  
    if (xSemaphoreTake(operation_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {  
        return BLE_CLIENT_TIMEOUT;  
    }  
  
    if (!is_scanning) {  
        xSemaphoreGive(operation_mutex);  
        return BLE_CLIENT_OK;  // Not an error if not scanning  
    }  
  
    esp_err_t ret = esp_ble_gap_stop_scanning();  
    if (ret != ESP_OK) {  
        STORED_LOGF("Stop scanning error: %d", ret);  
        xSemaphoreGive(operation_mutex);  
        return BLE_CLIENT_ESP_ERROR;  
    }  
  
    is_scanning = false;  
    xSemaphoreGive(operation_mutex);  
    return BLE_CLIENT_OK;  
}  
  
ble_client_result_t ble_client_get_scan_devices(ble_client_scan_device_t *devices, uint16_t *count) {  
    if (!ble_client_initialized || !devices || !count) {  
        return BLE_CLIENT_INVALID_STATE;  
    }  
  
    if (xSemaphoreTake(operation_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {  
        return BLE_CLIENT_TIMEOUT;  
    }  
  
    uint16_t num_to_copy = *count < scan_device_count ? *count : scan_device_count;  
    memcpy(devices, scan_devices, num_to_copy * sizeof(ble_client_scan_device_t));  
    *count = num_to_copy;  
  
    xSemaphoreGive(operation_mutex);  
    return BLE_CLIENT_OK;  
}  
  
ble_client_result_t ble_client_connect(esp_bd_addr_t bda, esp_ble_addr_type_t addr_type, ble_client_connect_cb_t callback, void *user_data) {  
    if (!ble_client_initialized) {  
        return BLE_CLIENT_INVALID_STATE;  
    }  
  
    if (xSemaphoreTake(operation_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {  
        return BLE_CLIENT_TIMEOUT;  
    }  
  
    if (is_connected) {  
        xSemaphoreGive(operation_mutex);  
        return BLE_CLIENT_ALREADY_CONNECTED;  
    }  
  
    // Store connection callback  
    connect_callback = callback;  
    connect_user_data = user_data;  
  
    // Open connection to the remote device  
    esp_err_t ret = esp_ble_gattc_open(gattc_if, bda, addr_type, true);  
    if (ret != ESP_OK) {  
        STORED_LOGF("Connect error: %d", ret);  
        xSemaphoreGive(operation_mutex);  
        return BLE_CLIENT_ESP_ERROR;  
    }  
  
    xSemaphoreGive(operation_mutex);  
    return BLE_CLIENT_OK;  
}  
  
ble_client_result_t ble_client_disconnect(void) {  
    if (!ble_client_initialized) {  
        return BLE_CLIENT_INVALID_STATE;  
    }  
  
    if (xSemaphoreTake(operation_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {  
        return BLE_CLIENT_TIMEOUT;  
    }  
  
    if (!is_connected) {  
        xSemaphoreGive(operation_mutex);  
        return BLE_CLIENT_NOT_CONNECTED;  
    }  
  
    esp_err_t ret = esp_ble_gattc_close(gattc_if, conn_id);  
    if (ret != ESP_OK) {  
        STORED_LOGF("Disconnect error: %d", ret);  
        xSemaphoreGive(operation_mutex);  
        return BLE_CLIENT_ESP_ERROR;  
    }  
  
    xSemaphoreGive(operation_mutex);  
    return BLE_CLIENT_OK;  
}  
  
bool ble_client_is_connected(void) {  
    if (!ble_client_initialized) {  
        return false;  
    }  
  
    if (xSemaphoreTake(operation_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {  
        return false;  
    }  
  
    bool connected = is_connected;  
    xSemaphoreGive(operation_mutex);  
    return connected;  
}  
  
ble_client_result_t ble_client_discover_services(void) {  
    if (!ble_client_initialized) {  
        return BLE_CLIENT_INVALID_STATE;  
    }  
  
    if (xSemaphoreTake(operation_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {  
        return BLE_CLIENT_TIMEOUT;  
    }  
  
    if (!is_connected) {  
        xSemaphoreGive(operation_mutex);  
        return BLE_CLIENT_NOT_CONNECTED;  
    }  
  
    if (is_discovering_services) {  
        xSemaphoreGive(operation_mutex);  
        return BLE_CLIENT_DISCOVERY_BUSY;  
    }  
  
    // Reset previous discovery results  
    reset_service_discovery();  
      
    // Start service discovery  
    esp_err_t ret = esp_ble_gattc_search_service(gattc_if, conn_id, NULL);  
    if (ret != ESP_OK) {  
        STORED_LOGF("Service discovery error: %d", ret);  
        xSemaphoreGive(operation_mutex);  
        return BLE_CLIENT_ESP_ERROR;  
    }  
  
    is_discovering_services = true;  
      
    // Wait for operation to complete  
    xSemaphoreGive(operation_mutex);  
    if (!wait_for_operation_complete(BLE_CLIENT_OPERATION_TIMEOUT)) {  
        STORED_LOGF("Service discovery timeout");  
        is_discovering_services = false;  
        return BLE_CLIENT_TIMEOUT;  
    }  
  
    return BLE_CLIENT_OK;  
}  
  
ble_client_result_t ble_client_get_services(ble_client_service_t *services, uint16_t *count) {  
    if (!ble_client_initialized || !services || !count) {  
        return BLE_CLIENT_INVALID_STATE;  
    }  
  
    if (xSemaphoreTake(operation_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {  
        return BLE_CLIENT_TIMEOUT;  
    }  
  
    if (!is_connected) {  
        xSemaphoreGive(operation_mutex);  
        return BLE_CLIENT_NOT_CONNECTED;  
    }  
  
    uint16_t num_to_copy = *count < discovered_service_count ? *count : discovered_service_count;  
    memcpy(services, discovered_services, num_to_copy * sizeof(ble_client_service_t));  
    *count = num_to_copy;  
  
    xSemaphoreGive(operation_mutex);  
    return BLE_CLIENT_OK;  
}  
  
ble_client_result_t ble_client_discover_characteristics(uint16_t service_handle) {  
    if (!ble_client_initialized) {  
        return BLE_CLIENT_INVALID_STATE;  
    }  
  
    if (xSemaphoreTake(operation_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {  
        return BLE_CLIENT_TIMEOUT;  
    }  
  
    if (!is_connected) {  
        xSemaphoreGive(operation_mutex);  
        return BLE_CLIENT_NOT_CONNECTED;  
    }  
  
    if (is_discovering_chars) {  
        xSemaphoreGive(operation_mutex);  
        return BLE_CLIENT_DISCOVERY_BUSY;  
    }  
  
    // Find the service by handle  
    int service_idx = find_service_by_handle(service_handle);  
    if (service_idx < 0) {  
        xSemaphoreGive(operation_mutex);  
        return BLE_CLIENT_INVALID_HANDLE;  
    }  
  
    // Reset previous discovery results  
    reset_char_discovery();  
    current_service_handle = service_handle;  
      
    // Get start and end handles for the service  
    uint16_t start_handle = discovered_services[service_idx].start_handle;  
    uint16_t end_handle = discovered_services[service_idx].end_handle;  
  
    // Start characteristic discovery  
    esp_err_t ret = esp_ble_gattc_get_all_char(  
        gattc_if, conn_id, start_handle, end_handle, NULL  
    );  
      
    if (ret != ESP_OK) {  
        STORED_LOGF("Characteristic discovery error: %d", ret);  
        xSemaphoreGive(operation_mutex);  
        return BLE_CLIENT_ESP_ERROR;  
    }  
  
    is_discovering_chars = true;  
      
    // Wait for operation to complete  
    xSemaphoreGive(operation_mutex);  
    if (!wait_for_operation_complete(BLE_CLIENT_OPERATION_TIMEOUT)) {  
        STORED_LOGF("Characteristic discovery timeout");  
        is_discovering_chars = false;  
        return BLE_CLIENT_TIMEOUT;  
    }  
  
    return BLE_CLIENT_OK;  
}  
  
ble_client_result_t ble_client_get_characteristics(uint16_t service_handle, ble_client_char_t *chars, uint16_t *count) {  
    if (!ble_client_initialized || !chars || !count) {  
        return BLE_CLIENT_INVALID_STATE;  
    }  
  
    if (xSemaphoreTake(operation_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {  
        return BLE_CLIENT_TIMEOUT;  
    }  
  
    if (!is_connected) {  
        xSemaphoreGive(operation_mutex);  
        return BLE_CLIENT_NOT_CONNECTED;  
    }  
  
    // Check if characteristics were discovered for this service  
    if (current_service_handle != service_handle) {  
        xSemaphoreGive(operation_mutex);  
        return BLE_CLIENT_NOT_FOUND;  
    }  
  
    uint16_t num_to_copy = *count < discovered_char_count ? *count : discovered_char_count;  
    memcpy(chars, discovered_chars, num_to_copy * sizeof(ble_client_char_t));  
    *count = num_to_copy;  
  
    xSemaphoreGive(operation_mutex);  
    return BLE_CLIENT_OK;  
}  
  
ble_client_result_t ble_client_discover_descriptors(uint16_t char_handle) {  
    if (!ble_client_initialized) {  
        return BLE_CLIENT_INVALID_STATE;  
    }  
  
    if (xSemaphoreTake(operation_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {  
        return BLE_CLIENT_TIMEOUT;  
    }  
  
    if (!is_connected) {  
        xSemaphoreGive(operation_mutex);  
        return BLE_CLIENT_NOT_CONNECTED;  
    }  
  
    if (is_discovering_descrs) {  
        xSemaphoreGive(operation_mutex);  
        return BLE_CLIENT_DISCOVERY_BUSY;  
    }  
  
    // Find the characteristic by handle  
    int char_idx = find_char_by_handle(char_handle);  
    if (char_idx < 0) {  
        xSemaphoreGive(operation_mutex);  
        return BLE_CLIENT_INVALID_HANDLE;  
    }  
  
    // Reset previous discovery results  
    reset_descriptor_discovery();  
    current_char_handle = char_handle;  
      
    // Get handle for the characteristic  
    uint16_t handle = discovered_chars[char_idx].handle;  
  
    // Start descriptor discovery  
    esp_err_t ret = esp_ble_gattc_get_all_descr(  
        gattc_if, conn_id, handle, NULL  
    );  
      
    if (ret != ESP_OK) {  
        STORED_LOGF("Descriptor discovery error: %d", ret);  
        xSemaphoreGive(operation_mutex);  
        return BLE_CLIENT_ESP_ERROR;  
    }  
  
    is_discovering_descrs = true;  
      
    // Wait for operation to complete  
    xSemaphoreGive(operation_mutex);  
    if (!wait_for_operation_complete(BLE_CLIENT_OPERATION_TIMEOUT)) {  
        STORED_LOGF("Descriptor discovery timeout");  
        is_discovering_descrs = false;  
        return BLE_CLIENT_TIMEOUT;  
    }  
  
    return BLE_CLIENT_OK;  
}  
  
ble_client_result_t ble_client_get_descriptors(uint16_t char_handle, ble_client_descr_t *descrs, uint16_t *count) {  
    if (!ble_client_initialized || !descrs || !count) {  
        return BLE_CLIENT_INVALID_STATE;  
    }  
  
    if (xSemaphoreTake(operation_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {  
        return BLE_CLIENT_TIMEOUT;  
    }  
  
    if (!is_connected) {  
        xSemaphoreGive(operation_mutex);  
        return BLE_CLIENT_NOT_CONNECTED;  
    }  
  
    // Check if descriptors were discovered for this characteristic  
    if (current_char_handle != char_handle) {  
        xSemaphoreGive(operation_mutex);  
        return BLE_CLIENT_NOT_FOUND;  
    }  
  
    uint16_t num_to_copy = *count < discovered_descr_count ? *count : discovered_descr_count;  
    memcpy(descrs, discovered_descrs, num_to_copy * sizeof(ble_client_descr_t));  
    *count = num_to_copy;  
  
    xSemaphoreGive(operation_mutex);  
    return BLE_CLIENT_OK;  
}  
  
//ble_client_result_t ble_client_read_characteristic(uint16_t char_handle,

// Read a characteristic value  
ble_client_result_t ble_client_read_characteristic(uint16_t char_handle, uint8_t **data, uint16_t *data_len){}//;  
  
// Write a characteristic value  
ble_client_result_t ble_client_write_characteristic(uint16_t char_handle, const uint8_t *data, uint16_t data_len, bool response){}//;  
  
// Read a descriptor value  
ble_client_result_t ble_client_read_descriptor(uint16_t descr_handle, uint8_t **data, uint16_t *data_len){}//;  
  
// Write a descriptor value  
ble_client_result_t ble_client_write_descriptor(uint16_t descr_handle, const uint8_t *data, uint16_t data_len){}//;  
  
// Register for notifications/indications from a characteristic  
ble_client_result_t ble_client_register_for_notify(uint16_t char_handle, ble_client_notify_cb_t callback, void *user_data){}//;  
  
// Unregister from notifications/indications from a characteristic  
ble_client_result_t ble_client_unregister_for_notify(uint16_t char_handle){}//; 
