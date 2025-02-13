//==================================================================================================
/// @file       ble_server.c
/// @brief      BLE GATT Server
/// @date       2025/1/4
//==================================================================================================

//==================================================================================================
// Include definition
//==================================================================================================
#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "esp_gatts_api.h"
#include "esp_log.h"
#include "esp_system.h"
#include "sdkconfig.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ble_server.h"
#include "smartlock.h"

//==================================================================================================
// Constant definition
//==================================================================================================
#define GATTS_TAG "SMART_LOCK_DEMO"
#define TEST_DEVICE_NAME "ESP_SMART_LOCK"
#define GATTS_DEMO_CHAR_VAL_LEN_MAX 0x40

// For advertising config
static uint8_t adv_config_done = 0;
#define adv_config_flag (1 << 0)
#define scan_rsp_config_flag (1 << 1)

// BLE Services / Characteristics
#define PROFILE_NUM 2
#define PROFILE_A_APP_ID 0
#define PROFILE_B_APP_ID 1
#define GATTS_SERVICE_UUID_TEST_A 0x00FF
#define GATTS_CHAR_UUID_TEST_A 0xFF01
#define GATTS_NUM_HANDLE_TEST_A 4
#define GATTS_SERVICE_UUID_TEST_B 0x00EE
#define GATTS_CHAR_UUID_TEST_B 0xEE01
#define GATTS_NUM_HANDLE_TEST_B 4

//==================================================================================================
// Private values definition
//==================================================================================================

// Characteristic initial value
static uint8_t char1_str[] = {0x11, 0x22, 0x33};
static esp_gatt_char_prop_t a_property = 0;
static esp_gatt_char_prop_t b_property = 0;

// GATT attribute for characteristic value
static esp_attr_value_t gatts_demo_char1_val = {
    .attr_max_len = GATTS_DEMO_CHAR_VAL_LEN_MAX,
    .attr_len = sizeof(char1_str),
    .attr_value = char1_str,
};

// Structure to store profile information.
struct gatts_profile_inst {
  esp_gatts_cb_t gatts_cb;
  uint16_t gatts_if;
  uint16_t app_id;
  uint16_t conn_id;
  uint16_t service_handle;
  esp_gatt_srvc_id_t service_id;
  uint16_t char_handle;
  esp_bt_uuid_t char_uuid;
  esp_gatt_perm_t perm;
  esp_gatt_char_prop_t property;
  uint16_t descr_handle;
  esp_bt_uuid_t descr_uuid;
};

// Prepare write structure for long write
typedef struct {
  uint8_t *prepare_buf;
  int prepare_len;
} prepare_type_env_t;

static prepare_type_env_t a_prepare_write_env;
static prepare_type_env_t b_prepare_write_env;

// BLE advertising data
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 16,
    .p_service_uuid =
        (uint8_t[]){0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10,
                    0x00, 0x00, 0xFF, 0x00, 0x00, 0x00},
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// BLE scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 16,
    .p_service_uuid =
        (uint8_t[]){0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10,
                    0x00, 0x00, 0xEE, 0x00, 0x00, 0x00},
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

//  BLE advertising parameters
static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// Profiles array
static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] =
        {
            .gatts_cb = NULL,
            .gatts_if = ESP_GATT_IF_NONE,
        },
    [PROFILE_B_APP_ID] =
        {
            .gatts_cb = NULL,
            .gatts_if = ESP_GATT_IF_NONE,
        },
};

//==================================================================================================
// Prototype
//==================================================================================================
static void gap_event_handler(esp_gap_ble_cb_event_t event,
                              esp_ble_gap_cb_param_t *param);
static void gatts_event_handler(esp_gatts_cb_event_t event,
                                esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param);

static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event,
                                          esp_gatt_if_t gatts_if,
                                          esp_ble_gatts_cb_param_t *param);

static void gatts_profile_b_event_handler(esp_gatts_cb_event_t event,
                                          esp_gatt_if_t gatts_if,
                                          esp_ble_gatts_cb_param_t *param);

//==================================================================================================
// Private functions
//==================================================================================================

//--------------------------------------------------------------------------------------------------
/// @brief  Utility functions for handling prepare write (long write)
//--------------------------------------------------------------------------------------------------
static void example_write_event_env(esp_gatt_if_t gatts_if,
                                    prepare_type_env_t *prepare_write_env,
                                    esp_ble_gatts_cb_param_t *param) {
  esp_gatt_status_t status = ESP_GATT_OK;
  if (param->write.need_rsp) {
    if (param->write.is_prep) {
      if (param->write.offset > GATTS_DEMO_CHAR_VAL_LEN_MAX) {
        status = ESP_GATT_INVALID_OFFSET;
      } else if ((param->write.offset + param->write.len) >
                 GATTS_DEMO_CHAR_VAL_LEN_MAX) {
        status = ESP_GATT_INVALID_ATTR_LEN;
      }
      if (status == ESP_GATT_OK && prepare_write_env->prepare_buf == NULL) {
        prepare_write_env->prepare_buf =
            (uint8_t *)malloc(GATTS_DEMO_CHAR_VAL_LEN_MAX);
        prepare_write_env->prepare_len = 0;
        if (prepare_write_env->prepare_buf == NULL) {
          ESP_LOGE(GATTS_TAG, "No memory for prepare write buffer");
          status = ESP_GATT_NO_RESOURCES;
        }
      }

      esp_gatt_rsp_t *gatt_rsp =
          (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
      if (gatt_rsp) {
        gatt_rsp->attr_value.len = param->write.len;
        gatt_rsp->attr_value.handle = param->write.handle;
        gatt_rsp->attr_value.offset = param->write.offset;
        gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
        memcpy(gatt_rsp->attr_value.value, param->write.value,
               param->write.len);
        esp_err_t response_err = esp_ble_gatts_send_response(
            gatts_if, param->write.conn_id, param->write.trans_id, status,
            gatt_rsp);
        if (response_err != ESP_OK) {
          ESP_LOGE(GATTS_TAG, "Send response error");
        }
        free(gatt_rsp);
      } else {
        ESP_LOGE(GATTS_TAG, "Failed to allocate memory for gatt_rsp");
        status = ESP_GATT_NO_RESOURCES;
      }
      if (status != ESP_GATT_OK) {
        return;
      }
      memcpy(prepare_write_env->prepare_buf + param->write.offset,
             param->write.value, param->write.len);
      prepare_write_env->prepare_len += param->write.len;
    } else {
      esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                  param->write.trans_id, status, NULL);
    }
  }
}

static void example_exec_write_event_env(prepare_type_env_t *prepare_write_env,
                                         esp_ble_gatts_cb_param_t *param) {
  if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC) {
    esp_log_buffer_hex(GATTS_TAG, prepare_write_env->prepare_buf,
                       prepare_write_env->prepare_len);
  } else {
    ESP_LOGI(GATTS_TAG, "Cancel prepare write");
  }
  if (prepare_write_env->prepare_buf) {
    free(prepare_write_env->prepare_buf);
    prepare_write_env->prepare_buf = NULL;
  }
  prepare_write_env->prepare_len = 0;
}

//--------------------------------------------------------------------------------------------------
/// @brief  Profile A event handler
//--------------------------------------------------------------------------------------------------
static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event,
                                          esp_gatt_if_t gatts_if,
                                          esp_ble_gatts_cb_param_t *param) {
  switch (event) {
  case ESP_GATTS_REG_EVT: {
    ESP_LOGI(GATTS_TAG, "Profile A: register app event, app_id=%d",
             param->reg.app_id);

    gl_profile_tab[PROFILE_A_APP_ID].service_id.is_primary = true;
    gl_profile_tab[PROFILE_A_APP_ID].service_id.id.inst_id = 0x00;
    gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
    gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.uuid.uuid16 =
        GATTS_SERVICE_UUID_TEST_A;

    // Set device name for advertising
    esp_err_t set_name_ret = esp_ble_gap_set_device_name(TEST_DEVICE_NAME);
    if (set_name_ret) {
      ESP_LOGE(GATTS_TAG, "Failed to set device name, err=0x%x", set_name_ret);
    }

    // Configure advertising data
    esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
    if (ret) {
      ESP_LOGE(GATTS_TAG, "Failed to configure adv data, err=0x%x", ret);
    }
    adv_config_done |= adv_config_flag;

    // Configure scan response data
    ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
    if (ret) {
      ESP_LOGE(GATTS_TAG, "Failed to configure scan_rsp data, err=0x%x", ret);
    }
    adv_config_done |= scan_rsp_config_flag;

    // Create primary service
    esp_ble_gatts_create_service(gatts_if,
                                 &gl_profile_tab[PROFILE_A_APP_ID].service_id,
                                 GATTS_NUM_HANDLE_TEST_A);
    break;
  }
  case ESP_GATTS_CREATE_EVT: {
    ESP_LOGI(GATTS_TAG, "Profile A: create service event, handle=%d",
             param->create.service_handle);

    gl_profile_tab[PROFILE_A_APP_ID].service_handle =
        param->create.service_handle;
    gl_profile_tab[PROFILE_A_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
    gl_profile_tab[PROFILE_A_APP_ID].char_uuid.uuid.uuid16 =
        GATTS_CHAR_UUID_TEST_A;

    esp_ble_gatts_start_service(
        gl_profile_tab[PROFILE_A_APP_ID].service_handle);

    // Characteristic property: read/write/notify
    a_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE |
                 ESP_GATT_CHAR_PROP_BIT_NOTIFY;
    esp_err_t add_char_ret =
        esp_ble_gatts_add_char(gl_profile_tab[PROFILE_A_APP_ID].service_handle,
                               &gl_profile_tab[PROFILE_A_APP_ID].char_uuid,
                               ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                               a_property, &gatts_demo_char1_val, NULL);
    if (add_char_ret) {
      ESP_LOGE(GATTS_TAG, "Failed to add characteristic, err=0x%x",
               add_char_ret);
    }
    break;
  }
  case ESP_GATTS_ADD_CHAR_EVT: {
    gl_profile_tab[PROFILE_A_APP_ID].char_handle = param->add_char.attr_handle;
    gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
    gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.uuid.uuid16 =
        ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
    esp_ble_gatts_add_char_descr(
        gl_profile_tab[PROFILE_A_APP_ID].service_handle,
        &gl_profile_tab[PROFILE_A_APP_ID].descr_uuid,
        ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
    break;
  }
  case ESP_GATTS_WRITE_EVT: {
    ESP_LOGI(GATTS_TAG, "Profile A: write event, handle=%d",
             param->write.handle);

    // Notify that we received some BLE data (blink LED1)
    smartlock_bluetooth_received();

    if (!param->write.is_prep) {
      // Check if the received data is "unlock"
      if (param->write.len == 6 &&
          memcmp(param->write.value, "unlock", 6) == 0) {
        ESP_LOGI(GATTS_TAG, "Unlock command received!");
        // Unlock (blink LED2 + lock open)
        smartlock_unlock();
      }

      // Handle descriptor writes (like CCCD for notifications)
      if (gl_profile_tab[PROFILE_A_APP_ID].descr_handle ==
              param->write.handle &&
          param->write.len == 2) {
        uint16_t descr_value =
            param->write.value[1] << 8 | param->write.value[0];
        if (descr_value == 0x0001) {
          // Notifications enabled
          if (a_property & ESP_GATT_CHAR_PROP_BIT_NOTIFY) {
            ESP_LOGI(GATTS_TAG, "Notifications enabled");
            uint8_t notify_data[3] = {0x01, 0x02, 0x03};
            esp_ble_gatts_send_indicate(
                gatts_if, param->write.conn_id,
                gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                sizeof(notify_data), notify_data, false);
          }
        } else if (descr_value == 0x0002) {
          // Indications enabled
          ESP_LOGI(GATTS_TAG, "Indications enabled");
        } else if (descr_value == 0x0000) {
          // Notifications/Indications disabled
          ESP_LOGI(GATTS_TAG, "Notifications disabled");
        } else {
          ESP_LOGE(GATTS_TAG, "Unknown descriptor value");
        }
      }
    }
    // Handle long write if necessary
    example_write_event_env(gatts_if, &a_prepare_write_env, param);
    break;
  }
  case ESP_GATTS_EXEC_WRITE_EVT: {
    esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                param->write.trans_id, ESP_GATT_OK, NULL);
    example_exec_write_event_env(&a_prepare_write_env, param);
    break;
  }
  default:
    break;
  }
}

//--------------------------------------------------------------------------------------------------
/// @brief  Profile B event handler
//--------------------------------------------------------------------------------------------------
static void gatts_profile_b_event_handler(esp_gatts_cb_event_t event,
                                          esp_gatt_if_t gatts_if,
                                          esp_ble_gatts_cb_param_t *param) {
  switch (event) {
  case ESP_GATTS_REG_EVT:
    ESP_LOGI(GATTS_TAG, "Profile B: register app event");
    gl_profile_tab[PROFILE_B_APP_ID].service_id.is_primary = true;
    gl_profile_tab[PROFILE_B_APP_ID].service_id.id.inst_id = 0x00;
    gl_profile_tab[PROFILE_B_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
    gl_profile_tab[PROFILE_B_APP_ID].service_id.id.uuid.uuid.uuid16 =
        GATTS_SERVICE_UUID_TEST_B;
    esp_ble_gatts_create_service(gatts_if,
                                 &gl_profile_tab[PROFILE_B_APP_ID].service_id,
                                 GATTS_NUM_HANDLE_TEST_B);
    break;
  case ESP_GATTS_CREATE_EVT:
    gl_profile_tab[PROFILE_B_APP_ID].service_handle =
        param->create.service_handle;
    gl_profile_tab[PROFILE_B_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
    gl_profile_tab[PROFILE_B_APP_ID].char_uuid.uuid.uuid16 =
        GATTS_CHAR_UUID_TEST_B;

    esp_ble_gatts_start_service(
        gl_profile_tab[PROFILE_B_APP_ID].service_handle);

    b_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE |
                 ESP_GATT_CHAR_PROP_BIT_NOTIFY;
    esp_err_t add_char_ret = esp_ble_gatts_add_char(
        gl_profile_tab[PROFILE_B_APP_ID].service_handle,
        &gl_profile_tab[PROFILE_B_APP_ID].char_uuid,
        ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, b_property, NULL, NULL);
    if (add_char_ret) {
      ESP_LOGE(GATTS_TAG, "Failed to add characteristic, err=0x%x",
               add_char_ret);
    }
    break;
  default:
    break;
  }
}

//--------------------------------------------------------------------------------------------------
/// @brief  GAP event handler
//--------------------------------------------------------------------------------------------------
static void gap_event_handler(esp_gap_ble_cb_event_t event,
                              esp_ble_gap_cb_param_t *param) {
  switch (event) {
  case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
    adv_config_done &= (~adv_config_flag);
    if (adv_config_done == 0) {
      esp_ble_gap_start_advertising(&adv_params);
    }
    break;
  case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
    adv_config_done &= (~scan_rsp_config_flag);
    if (adv_config_done == 0) {
      esp_ble_gap_start_advertising(&adv_params);
    }
    break;
  case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
    if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
      ESP_LOGE(GATTS_TAG, "Failed to start advertising");
    } else {
      ESP_LOGI(GATTS_TAG, "Advertising started successfully");
    }
    break;
  case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
    if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
      ESP_LOGE(GATTS_TAG, "Failed to stop advertising");
    } else {
      ESP_LOGI(GATTS_TAG, "Advertising stopped successfully");
    }
    break;
  default:
    break;
  }
}

//--------------------------------------------------------------------------------------------------
/// @brief  Main GATTS event handler
//--------------------------------------------------------------------------------------------------
static void gatts_event_handler(esp_gatts_cb_event_t event,
                                esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param) {
  // Register event
  if (event == ESP_GATTS_REG_EVT) {
    if (param->reg.status == ESP_GATT_OK) {
      gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
    } else {
      ESP_LOGI(GATTS_TAG, "Failed to register GATT, app_id=%04x, status=%d",
               param->reg.app_id, param->reg.status);
      return;
    }
  }

  // Dispatch to each profile's event handler
  for (int idx = 0; idx < PROFILE_NUM; idx++) {
    if (gatts_if == ESP_GATT_IF_NONE ||
        gatts_if == gl_profile_tab[idx].gatts_if) {
      if (gl_profile_tab[idx].gatts_cb) {
        gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
      }
    }
  }
}

//==================================================================================================
// Public functions
//==================================================================================================

//--------------------------------------------------------------------------------------------------
/// @brief  Public function to initialize and start BLE GATT server
//--------------------------------------------------------------------------------------------------
void ble_server_init(void) {
  ESP_LOGI(GATTS_TAG, "Initializing BLE GATT server...");

  // Assign event callbacks for Profile A
  gl_profile_tab[PROFILE_A_APP_ID].gatts_cb = gatts_profile_a_event_handler;
  gl_profile_tab[PROFILE_A_APP_ID].app_id = PROFILE_A_APP_ID;

  // Assign event callbacks for Profile B
  gl_profile_tab[PROFILE_B_APP_ID].gatts_cb = gatts_profile_b_event_handler;
  gl_profile_tab[PROFILE_B_APP_ID].app_id = PROFILE_B_APP_ID;

  // Register GATT server callback
  esp_err_t ret = esp_ble_gatts_register_callback(gatts_event_handler);
  if (ret) {
    ESP_LOGE(GATTS_TAG, "Failed to register GATTS callback, err=0x%x", ret);
    return;
  }

  // Register GAP callback
  ret = esp_ble_gap_register_callback(gap_event_handler);
  if (ret) {
    ESP_LOGE(GATTS_TAG, "Failed to register GAP callback, err=0x%x", ret);
    return;
  }

  // Register two profiles (Profile A and B)
  ret = esp_ble_gatts_app_register(PROFILE_A_APP_ID);
  if (ret) {
    ESP_LOGE(GATTS_TAG, "Failed to register Profile A, err=0x%x", ret);
    return;
  }
  ret = esp_ble_gatts_app_register(PROFILE_B_APP_ID);
  if (ret) {
    ESP_LOGE(GATTS_TAG, "Failed to register Profile B, err=0x%x", ret);
    return;
  }

  // Set local MTU size if needed
  esp_err_t mtu_ret = esp_ble_gatt_set_local_mtu(500);
  if (mtu_ret) {
    ESP_LOGE(GATTS_TAG, "Failed to set local MTU, err=0x%x", mtu_ret);
  }

  ESP_LOGI(GATTS_TAG, "BLE GATT server setup complete.");
}