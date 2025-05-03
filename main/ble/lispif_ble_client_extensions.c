/*  
	Copyright 2025  
  
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
  
#include "lispif_ble_client_extensions.h"  
  
#include <string.h>  
  
#include "esp_bt_defs.h"  
  
#include "custom_ble_client.h"  
#include "lispif_events.h"  
#include "lbm_vesc_utils.h"  
#include "utils.h"  
#include "heap.h"  
#include "lbm_defines.h"  
#include "lbm_memory.h"  
#include "lbm_flat_value.h"  
#include "eval_cps.h"  
#include "extensions.h"  
#include "commands.h"  
  
#define STORED_LOGF commands_printf  
#define BLE_CLIENT_TAG "BLE_CLIENT"  
  
// Error reason strings  
static const char *error_invalid_uuid = "Invalid UUID format";  
static const char *error_invalid_addr = "Invalid BLE address format";  
static const char *error_invalid_handle = "Invalid handle";  
static const char *error_not_connected = "Not connected to any BLE device";  
static const char *error_already_connected = "Already connected to a BLE device";  
static const char *error_scan_busy = "Scan already in progress";  
static const char *error_discovery_busy = "Discovery already in progress";  
static const char *error_memory_error = "Memory allocation failed";  
static const char *error_timeout = "Operation timed out";  
static const char *error_esp_error = "ESP BLE error";  
  
// LispBM symbols  
static lbm_uint symbol_uuid = 0;  
static lbm_uint symbol_name = 0;  
static lbm_uint symbol_rssi = 0;  
static lbm_uint symbol_addr = 0;  
static lbm_uint symbol_addr_type = 0;  
static lbm_uint symbol_handle = 0;  
static lbm_uint symbol_properties = 0;  
static lbm_uint symbol_start_handle = 0;  
static lbm_uint symbol_end_handle = 0;  
static lbm_uint symbol_service = 0;  
static lbm_uint symbol_characteristic = 0;  
static lbm_uint symbol_descriptor = 0;  
static lbm_uint symbol_value = 0;  
static lbm_uint symbol_notify = 0;  
static lbm_uint symbol_indicate = 0;  
static lbm_uint symbol_read = 0;  
static lbm_uint symbol_write = 0;  
static lbm_uint symbol_write_no_response = 0;  
static lbm_uint symbol_public = 0;  
static lbm_uint symbol_random = 0;  
static lbm_uint symbol_rpa_public = 0;  
static lbm_uint symbol_rpa_random = 0;  
  
// Event symbols  
static lbm_uint symbol_event_ble_client_scan = 0;  
static lbm_uint symbol_event_ble_client_connected = 0;  
static lbm_uint symbol_event_ble_client_disconnected = 0;  
static lbm_uint symbol_event_ble_client_notify = 0;  
  
// Forward declarations  
static void scan_callback(ble_client_scan_device_t *device, void *user_data);  
static void connect_callback(bool connected, void *user_data);  
static void notify_callback(uint16_t char_handle, const uint8_t *data, size_t data_len, bool is_notify, void *user_data);  
  
// Helper functions  
static bool register_symbols(void) {  
    bool res = true;  
  
    // clang-format off  
    res = res && lbm_add_symbol_const("uuid", &symbol_uuid);  
    res = res && lbm_add_symbol_const("name", &symbol_name);  
    res = res && lbm_add_symbol_const("rssi", &symbol_rssi);  
    res = res && lbm_add_symbol_const("addr", &symbol_addr);  
    res = res && lbm_add_symbol_const("addr-type", &symbol_addr_type);  
    res = res && lbm_add_symbol_const("handle", &symbol_handle);  
    res = res && lbm_add_symbol_const("properties", &symbol_properties);  
    res = res && lbm_add_symbol_const("start-handle", &symbol_start_handle);  
    res = res && lbm_add_symbol_const("end-handle", &symbol_end_handle);  
    res = res && lbm_add_symbol_const("service", &symbol_service);  
    res = res && lbm_add_symbol_const("characteristic", &symbol_characteristic);  
    res = res && lbm_add_symbol_const("descriptor", &symbol_descriptor);  
    res = res && lbm_add_symbol_const("value", &symbol_value);  
    res = res && lbm_add_symbol_const("notify", &symbol_notify);  
    res = res && lbm_add_symbol_const("indicate", &symbol_indicate);  
    res = res && lbm_add_symbol_const("read", &symbol_read);  
    res = res && lbm_add_symbol_const("write", &symbol_write);  
    res = res && lbm_add_symbol_const("write-no-response", &symbol_write_no_response);  
    res = res && lbm_add_symbol_const("public", &symbol_public);  
    res = res && lbm_add_symbol_const("random", &symbol_random);  
    res = res && lbm_add_symbol_const("rpa-public", &symbol_rpa_public);  
    res = res && lbm_add_symbol_const("rpa-random", &symbol_rpa_random);  
      
    // Event symbols  
    res = res && lbm_add_symbol_const("event-ble-client-scan", &symbol_event_ble_client_scan);  
    res = res && lbm_add_symbol_const("event-ble-client-connected", &symbol_event_ble_client_connected);  
    res = res && lbm_add_symbol_const("event-ble-client-disconnected", &symbol_event_ble_client_disconnected);  
    res = res && lbm_add_symbol_const("event-ble-client-notify", &symbol_event_ble_client_notify);  
    // clang-format on  
  
    return res;  
}  
  
// Convert LispBM array to BLE address  
static bool lbm_dec_ble_addr(lbm_value addr_val, esp_bd_addr_t addr) {  
    if (!lbm_is_array_r(addr_val)) {  
        return false;  
    }  
  
    uint32_t size = lbm_heap_array_get_size(addr_val);  
    if (size != 6) {  
        return false;  
    }  
  
    const uint8_t *data = (const uint8_t *)lbm_heap_array_get_data_ro(addr_val);  
    if (!data) {  
        return false;  
    }  
  
    memcpy(addr, data, 6);  
    return true;  
}  
  
// Convert LispBM symbol to address type  
static bool lbm_dec_addr_type(lbm_value type_val, esp_ble_addr_type_t *addr_type) {  
    if (!lbm_is_symbol(type_val)) {  
        return false;  
    }  
  
    lbm_uint sym = lbm_dec_sym(type_val);  
    if (sym == symbol_public) {  
        *addr_type = BLE_ADDR_TYPE_PUBLIC;  
    } else if (sym == symbol_random) {  
        *addr_type = BLE_ADDR_TYPE_RANDOM;  
    } else if (sym == symbol_rpa_public) {  
        *addr_type = BLE_ADDR_TYPE_RPA_PUBLIC;  
    } else if (sym == symbol_rpa_random) {  
        *addr_type = BLE_ADDR_TYPE_RPA_RANDOM;  
    } else {  
        return false;  
    }  
  
    return true;  
}  
  
// Convert UUID to LispBM array  
static lbm_value lbm_enc_uuid(esp_bt_uuid_t *uuid) {  
    lbm_value result;  
    uint8_t *data;  
      
    switch (uuid->len) {  
        case ESP_UUID_LEN_16:  
            data = lbm_malloc_reserve(2);  
            if (!data) {  
                return ENC_SYM_MERROR;  
            }  
            data[0] = (uuid->uuid.uuid16 >> 8) & 0xFF;  
            data[1] = uuid->uuid.uuid16 & 0xFF;  
            if (!lbm_lift_array(&result, (char *)data, 2)) {  
                lbm_free(data);  
                return ENC_SYM_MERROR;  
            }  
            break;  
              
        case ESP_UUID_LEN_32:  
            data = lbm_malloc_reserve(4);  
            if (!data) {  
                return ENC_SYM_MERROR;  
            }  
            data[0] = (uuid->uuid.uuid32 >> 24) & 0xFF;  
            data[1] = (uuid->uuid.uuid32 >> 16) & 0xFF;  
            data[2] = (uuid->uuid.uuid32 >> 8) & 0xFF;  
            data[3] = uuid->uuid.uuid32 & 0xFF;  
            if (!lbm_lift_array(&result, (char *)data, 4)) {  
                lbm_free(data);  
                return ENC_SYM_MERROR;  
            }  
            break;  
              
        case ESP_UUID_LEN_128:  
            data = lbm_malloc_reserve(16);  
            if (!data) {  
                return ENC_SYM_MERROR;  
            }  
            memcpy(data, uuid->uuid.uuid128, 16);  
            if (!lbm_lift_array(&result, (char *)data, 16)) {  
                lbm_free(data);  
                return ENC_SYM_MERROR;  
            }  
            break;  
              
        default:  
            return ENC_SYM_EERROR;  
    }  
      
    return result;  
}  
  
// Convert BLE address to LispBM array  
static lbm_value lbm_enc_ble_addr(esp_bd_addr_t addr) {  
    lbm_value result;  
    uint8_t *data = lbm_malloc_reserve(6);  
    if (!data) {  
        return ENC_SYM_MERROR;  
    }  
      
    memcpy(data, addr, 6);  
    if (!lbm_lift_array(&result, (char *)data, 6)) {  
        lbm_free(data);  
        return ENC_SYM_MERROR;  
    }  
      
    return result;  
}  
  
// Convert address type to LispBM symbol  
static lbm_value lbm_enc_addr_type(esp_ble_addr_type_t addr_type) {  
    switch (addr_type) {  
        case BLE_ADDR_TYPE_PUBLIC:  
            return ENC_SYM(symbol_public);  
        case BLE_ADDR_TYPE_RANDOM:  
            return ENC_SYM(symbol_random);  
        case BLE_ADDR_TYPE_RPA_PUBLIC:  
            return ENC_SYM(symbol_rpa_public);  
        case BLE_ADDR_TYPE_RPA_RANDOM:  
            return ENC_SYM(symbol_rpa_random);  
        default:  
            return ENC_SYM_NIL;  
    }  
}  
  
// Convert characteristic properties to LispBM list  
static lbm_value lbm_enc_char_properties(esp_gatt_char_prop_t props) {  
    lbm_value result = ENC_SYM_NIL;  
      
    if (props & ESP_GATT_CHAR_PROP_BIT_READ) {  
        lbm_value cons = lbm_cons(ENC_SYM(symbol_read), result);  
        if (cons == ENC_SYM_MERROR) {  
            return ENC_SYM_MERROR;  
        }  
        result = cons;  
    }  
      
    if (props & ESP_GATT_CHAR_PROP_BIT_WRITE) {  
        lbm_value cons = lbm_cons(ENC_SYM(symbol_write), result);  
        if (cons == ENC_SYM_MERROR) {  
            return ENC_SYM_MERROR;  
        }  
        result = cons;  
    }  
      
    if (props & ESP_GATT_CHAR_PROP_BIT_WRITE_NR) {  
        lbm_value cons = lbm_cons(ENC_SYM(symbol_write_no_response), result);  
        if (cons == ENC_SYM_MERROR) {  
            return ENC_SYM_MERROR;  
        }  
        result = cons;  
    }  
      
    if (props & ESP_GATT_CHAR_PROP_BIT_NOTIFY) {  
        lbm_value cons = lbm_cons(ENC_SYM(symbol_notify), result);
