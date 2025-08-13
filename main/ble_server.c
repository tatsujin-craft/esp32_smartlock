//==================================================================================================
/// @file       ble_server.c
/// @brief      BLE GATT Server
/// @date       2025/1/4
//==================================================================================================

//==================================================================================================
// Include definition
//==================================================================================================
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "esp_gatts_api.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include "ble_server.h"
#include "smartlock.h"

//==================================================================================================
// Constant definition
//==================================================================================================

// #define BLE_DEVICE_NAME "ESP_SMART_LOCK"
#ifdef CONFIG_BLE_DEVICE_NAME
#define BLE_SERVER_NAME (CONFIG_BLE_DEVICE_NAME)
#endif

#ifdef CONFIG_BT_SMP_MAX_BONDS
#define MAX_BONDED_DEVICES (CONFIG_BT_SMP_MAX_BONDS)
#endif

#define GATTS_DEMO_CHAR_VAL_LEN_MAX 0x40
#define GATTS_TAG "SMART_LOCK_DEMO"

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

#define BONDING_MODE_DURATION_MS (3 * 60 * 1000)

//==================================================================================================
// Prototype
//==================================================================================================
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param);
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t* param);
static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                          esp_ble_gatts_cb_param_t* param);
static void gatts_profile_b_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                          esp_ble_gatts_cb_param_t* param);
static void bonding_timer_callback(void* arg);
static bool is_bonded_device(esp_bd_addr_t addr);
static void store_bonded_device_to_nvs(esp_bd_addr_t addr);
static void remove_lru_bonded_device_if_needed(void);

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
  uint8_t* prepare_buf;
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
    .p_service_uuid = (uint8_t[]){0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00,
                                  0x00, 0xFF, 0x00, 0x00, 0x00},
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
    .p_service_uuid = (uint8_t[]){0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00,
                                  0x00, 0xEE, 0x00, 0x00, 0x00},
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

// Bonding mode
static esp_timer_handle_t bonding_timer = NULL;
static bool bonding_mode_enabled = false;
static bool is_advertising = false;

//==================================================================================================
// Public functions
//==================================================================================================

//--------------------------------------------------------------------------------------------------
/// @brief  Public function to initialize and start BLE GATT server
//--------------------------------------------------------------------------------------------------
void ble_server_init(void) {
  ESP_LOGI(GATTS_TAG, "Initializing BLE GATT server...");
  esp_err_t ret;

  // Release memory reserved for Classic Bluetooth
  ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

  // Initialize Bluetooth controller
  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  ret = esp_bt_controller_init(&bt_cfg);
  ESP_ERROR_CHECK(ret);

  // Enable BLE mode
  ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
  ESP_ERROR_CHECK(ret);

  // Initialize Bluedroid stack
  esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
  ret = esp_bluedroid_init_with_cfg(&bluedroid_cfg);
  ESP_ERROR_CHECK(ret);

  // Enable Bluedroid stack
  ret = esp_bluedroid_enable();
  ESP_ERROR_CHECK(ret);

  ESP_LOGI(GATTS_TAG, "Bluetooth driver initialized.");

  // Assign event callbacks for Profile A
  gl_profile_tab[PROFILE_A_APP_ID].gatts_cb = gatts_profile_a_event_handler;
  gl_profile_tab[PROFILE_A_APP_ID].app_id = PROFILE_A_APP_ID;

  // Assign event callbacks for Profile B
  gl_profile_tab[PROFILE_B_APP_ID].gatts_cb = gatts_profile_b_event_handler;
  gl_profile_tab[PROFILE_B_APP_ID].app_id = PROFILE_B_APP_ID;

  // Register GATT server callback
  ret = esp_ble_gatts_register_callback(gatts_event_handler);
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

  // Security parameter
  esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND;
  esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;
  uint8_t key_size = 16;
  uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;

  esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(auth_req));
  esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(iocap));
  esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(key_size));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(init_key));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(rsp_key));
  ESP_LOGI(GATTS_TAG, "Security parameters configured.");

  // Advertise
  esp_ble_gap_start_advertising(&adv_params);
  ESP_LOGI(GATTS_TAG, "Start advertising.");

  ESP_LOGI(GATTS_TAG, "BLE GATT server setup complete.");
}

//--------------------------------------------------------------------------------------------------
/// @brief  Start advertiseing (only debug)
//--------------------------------------------------------------------------------------------------
void ble_server_start_advertising(uint32_t duration_ms) {
  ESP_LOGI(GATTS_TAG, "Bonding mode enabled for %lu seconds", duration_ms / 1000);
  esp_ble_gap_start_advertising(&adv_params);

  if (!bonding_timer) {
    const esp_timer_create_args_t timer_args = {.callback = bonding_timer_callback,
                                                .arg = NULL,
                                                .dispatch_method = ESP_TIMER_TASK,
                                                .name = "bonding_timeout"};
    esp_timer_create(&timer_args, &bonding_timer);
  }

  esp_timer_start_once(bonding_timer, duration_ms * 1000);
}

//--------------------------------------------------------------------------------------------------
/// @brief  Stop advertiseing (only debug)
//--------------------------------------------------------------------------------------------------
void ble_server_stop_advertising(void) {
  if (esp_ble_gap_stop_advertising() == ESP_OK) {
    ESP_LOGI(GATTS_TAG, "Advertising manually stopped");
  } else {
    ESP_LOGW(GATTS_TAG, "Failed to stop advertising");
  }
  if (bonding_timer) {
    esp_timer_stop(bonding_timer);
  }
}

//--------------------------------------------------------------------------------------------------
/// @brief  Enable bonding mode
//--------------------------------------------------------------------------------------------------
void ble_server_enable_bonding_mode(void) {
  bonding_mode_enabled = true;

  // Advertise
  esp_ble_gap_start_advertising(&adv_params);

  if (bonding_timer == NULL) {
    const esp_timer_create_args_t timer_args = {.callback = bonding_timer_callback,
                                                .arg = NULL,
                                                .dispatch_method = ESP_TIMER_TASK,
                                                .name = "bonding_timer"};
    esp_timer_create(&timer_args, &bonding_timer);
  }

  esp_timer_stop(bonding_timer);
  esp_timer_start_once(bonding_timer, BONDING_MODE_DURATION_MS * 1000);
  ESP_LOGI("BLE_SERVER", "🔓 Bonding mode enabled for %d ms", BONDING_MODE_DURATION_MS);
}

//--------------------------------------------------------------------------------------------------
/// @brief  Disable bonding mode
//--------------------------------------------------------------------------------------------------
void ble_server_disable_bonding_mode(void) {
  bonding_mode_enabled = false;
  if (bonding_timer != NULL) {
    // xTimerStop(bonding_timer, 0);
    esp_timer_stop(bonding_timer);
  }
  ESP_LOGI("BLE_SERVER", "🔒 Bonding mode manually disabled.");
}

//--------------------------------------------------------------------------------------------------
/// @brief  Get bonding mode status
//--------------------------------------------------------------------------------------------------
bool ble_server_is_bonding_mode_enabled(void) { return bonding_mode_enabled; }

//--------------------------------------------------------------------------------------------------
/// @brief  Get advertising status
//--------------------------------------------------------------------------------------------------
bool ble_server_is_advertising(void) { return is_advertising; }

//--------------------------------------------------------------------------------------------------
/// @brief  Print bonding devices list
//--------------------------------------------------------------------------------------------------
void ble_server_print_bonded_devices(void) {
  int dev_num = esp_ble_get_bond_device_num();
  if (dev_num == 0) {
    ESP_LOGI(GATTS_TAG, "No bonded devices.");
    return;
  }

  esp_ble_bond_dev_t* dev_list = (esp_ble_bond_dev_t*)malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
  if (!dev_list) {
    ESP_LOGE(GATTS_TAG, "Failed to allocate memory for bonded device list");
    return;
  }

  esp_err_t err = esp_ble_get_bond_device_list(&dev_num, dev_list);
  if (err == ESP_OK) {
    ESP_LOGI(GATTS_TAG, "Bonded devices (%d):", dev_num);
    for (int i = 0; i < dev_num; i++) {
      ESP_LOGI(GATTS_TAG, "[%d] %02x:%02x:%02x:%02x:%02x:%02x", i, dev_list[i].bd_addr[0],
               dev_list[i].bd_addr[1], dev_list[i].bd_addr[2], dev_list[i].bd_addr[3],
               dev_list[i].bd_addr[4], dev_list[i].bd_addr[5]);
    }
  } else {
    ESP_LOGE(GATTS_TAG, "Failed to get bonded device list: %s", esp_err_to_name(err));
  }

  free(dev_list);
}

//--------------------------------------------------------------------------------------------------
/// @brief  Clear bonding devices
//--------------------------------------------------------------------------------------------------
void ble_server_clear_bonded_devices(void) {
  int dev_num = esp_ble_get_bond_device_num();
  if (dev_num == 0) {
    ESP_LOGI(GATTS_TAG, "No bonded devices to remove.");
    return;
  }

  esp_ble_bond_dev_t* dev_list = malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
  if (!dev_list) {
    ESP_LOGE(GATTS_TAG, "Failed to allocate memory");
    return;
  }

  esp_err_t err = esp_ble_get_bond_device_list(&dev_num, dev_list);
  if (err != ESP_OK) {
    ESP_LOGE(GATTS_TAG, "Failed to get bonded list: %s", esp_err_to_name(err));
    free(dev_list);
    return;
  }

  for (int i = 0; i < dev_num; i++) {
    ESP_LOGI(GATTS_TAG, "Removing bonded device: %02x:%02x:%02x:%02x:%02x:%02x",
             dev_list[i].bd_addr[0], dev_list[i].bd_addr[1], dev_list[i].bd_addr[2],
             dev_list[i].bd_addr[3], dev_list[i].bd_addr[4], dev_list[i].bd_addr[5]);

    esp_ble_remove_bond_device(dev_list[i].bd_addr);
  }

  free(dev_list);
  ESP_LOGI(GATTS_TAG, "All bonded devices cleared.");
}

//==================================================================================================
// Private functions
//==================================================================================================

//--------------------------------------------------------------------------------------------------
/// @brief  Utility functions for handling prepare write (long write)
//--------------------------------------------------------------------------------------------------
static void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t* prepare_write_env,
                                    esp_ble_gatts_cb_param_t* param) {
  esp_gatt_status_t status = ESP_GATT_OK;
  if (param->write.need_rsp) {
    if (param->write.is_prep) {
      if (param->write.offset > GATTS_DEMO_CHAR_VAL_LEN_MAX) {
        status = ESP_GATT_INVALID_OFFSET;
      } else if ((param->write.offset + param->write.len) > GATTS_DEMO_CHAR_VAL_LEN_MAX) {
        status = ESP_GATT_INVALID_ATTR_LEN;
      }
      if (status == ESP_GATT_OK && prepare_write_env->prepare_buf == NULL) {
        prepare_write_env->prepare_buf = (uint8_t*)malloc(GATTS_DEMO_CHAR_VAL_LEN_MAX);
        prepare_write_env->prepare_len = 0;
        if (prepare_write_env->prepare_buf == NULL) {
          ESP_LOGE(GATTS_TAG, "No memory for prepare write buffer");
          status = ESP_GATT_NO_RESOURCES;
        }
      }

      esp_gatt_rsp_t* gatt_rsp = (esp_gatt_rsp_t*)malloc(sizeof(esp_gatt_rsp_t));
      if (gatt_rsp) {
        gatt_rsp->attr_value.len = param->write.len;
        gatt_rsp->attr_value.handle = param->write.handle;
        gatt_rsp->attr_value.offset = param->write.offset;
        gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
        memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
        esp_err_t response_err = esp_ble_gatts_send_response(
            gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
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
      memcpy(prepare_write_env->prepare_buf + param->write.offset, param->write.value,
             param->write.len);
      prepare_write_env->prepare_len += param->write.len;
    } else {
      esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status,
                                  NULL);
    }
  }
}

static void example_exec_write_event_env(prepare_type_env_t* prepare_write_env,
                                         esp_ble_gatts_cb_param_t* param) {
  if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC) {
    esp_log_buffer_hex(GATTS_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
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
static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                          esp_ble_gatts_cb_param_t* param) {
  switch (event) {
    case ESP_GATTS_REG_EVT: {
      ESP_LOGI(GATTS_TAG, "Profile A: register app event, app_id=%d", param->reg.app_id);

      gl_profile_tab[PROFILE_A_APP_ID].service_id.is_primary = true;
      gl_profile_tab[PROFILE_A_APP_ID].service_id.id.inst_id = 0x00;
      gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
      gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_TEST_A;

      // Set device name for advertising
      esp_err_t set_name_ret = esp_ble_gap_set_device_name(BLE_SERVER_NAME);
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
      esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_A_APP_ID].service_id,
                                   GATTS_NUM_HANDLE_TEST_A);
      break;
    }

    case ESP_GATTS_CREATE_EVT: {
      ESP_LOGI(GATTS_TAG, "Profile A: create service event, handle=%d",
               param->create.service_handle);

      gl_profile_tab[PROFILE_A_APP_ID].service_handle = param->create.service_handle;
      gl_profile_tab[PROFILE_A_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
      gl_profile_tab[PROFILE_A_APP_ID].char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_TEST_A;

      esp_ble_gatts_start_service(gl_profile_tab[PROFILE_A_APP_ID].service_handle);

      // Characteristic property: read/write/notify
      a_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE |
                   ESP_GATT_CHAR_PROP_BIT_NOTIFY;
      esp_err_t add_char_ret = esp_ble_gatts_add_char(
          gl_profile_tab[PROFILE_A_APP_ID].service_handle,
          &gl_profile_tab[PROFILE_A_APP_ID].char_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
          a_property, &gatts_demo_char1_val, NULL);
      if (add_char_ret) {
        ESP_LOGE(GATTS_TAG, "Failed to add characteristic, err=0x%x", add_char_ret);
      }
      break;
    }

    case ESP_GATTS_ADD_CHAR_EVT: {
      gl_profile_tab[PROFILE_A_APP_ID].char_handle = param->add_char.attr_handle;
      gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
      gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
      esp_ble_gatts_add_char_descr(gl_profile_tab[PROFILE_A_APP_ID].service_handle,
                                   &gl_profile_tab[PROFILE_A_APP_ID].descr_uuid,
                                   ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
      break;
    }

    case ESP_GATTS_WRITE_EVT: {
      ESP_LOGI(GATTS_TAG, "Profile A: write event, handle=%d", param->write.handle);

      if (!is_bonded_device(param->write.bda)) {
        ESP_LOGW(GATTS_TAG, "❌ Data from unbonded device rejected.");
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id,
                                    ESP_GATT_INSUF_AUTHENTICATION, NULL);
        break;
      }

      ESP_LOGI(GATTS_TAG, "✅ Data received (%d bytes): %s", param->write.len, param->write.value);
      esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);

      // Notify that we received some BLE data (blink LED1)
      smartlock_bluetooth_received();

      if (!param->write.is_prep) {
        // Check if the received data is "unlock"
        if (param->write.len >= 6 && strncmp((const char*)param->write.value, "unlock", 6) == 0) {
          ESP_LOGI(GATTS_TAG, "Unlock command received!");
          // Unlock
          smartlock_unlock();
        }

        // Handle descriptor writes (like CCCD for notifications)
        if (gl_profile_tab[PROFILE_A_APP_ID].descr_handle == param->write.handle &&
            param->write.len == 2) {
          uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
          if (descr_value == 0x0001) {
            // Notifications enabled
            if (a_property & ESP_GATT_CHAR_PROP_BIT_NOTIFY) {
              ESP_LOGI(GATTS_TAG, "Notifications enabled");
              uint8_t notify_data[3] = {0x01, 0x02, 0x03};
              esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id,
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
      esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id,
                                  ESP_GATT_OK, NULL);
      example_exec_write_event_env(&a_prepare_write_env, param);
      break;
    }

    case ESP_GATTS_DISCONNECT_EVT: {
      ESP_LOGW(GATTS_TAG, "❗ Disconnected from client. Restarting advertising...");
      esp_ble_gap_start_advertising(&adv_params);
      break;
    }

    default:
      break;
  }
}

//--------------------------------------------------------------------------------------------------
/// @brief  Profile B event handler
//--------------------------------------------------------------------------------------------------
static void gatts_profile_b_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                          esp_ble_gatts_cb_param_t* param) {
  switch (event) {
    case ESP_GATTS_REG_EVT:
      ESP_LOGI(GATTS_TAG, "Profile B: register app event");
      gl_profile_tab[PROFILE_B_APP_ID].service_id.is_primary = true;
      gl_profile_tab[PROFILE_B_APP_ID].service_id.id.inst_id = 0x00;
      gl_profile_tab[PROFILE_B_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
      gl_profile_tab[PROFILE_B_APP_ID].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_TEST_B;
      esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_B_APP_ID].service_id,
                                   GATTS_NUM_HANDLE_TEST_B);
      break;

    case ESP_GATTS_CREATE_EVT:
      gl_profile_tab[PROFILE_B_APP_ID].service_handle = param->create.service_handle;
      gl_profile_tab[PROFILE_B_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
      gl_profile_tab[PROFILE_B_APP_ID].char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_TEST_B;

      esp_ble_gatts_start_service(gl_profile_tab[PROFILE_B_APP_ID].service_handle);

      b_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE |
                   ESP_GATT_CHAR_PROP_BIT_NOTIFY;
      esp_err_t add_char_ret =
          esp_ble_gatts_add_char(gl_profile_tab[PROFILE_B_APP_ID].service_handle,
                                 &gl_profile_tab[PROFILE_B_APP_ID].char_uuid,
                                 ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, b_property, NULL, NULL);
      if (add_char_ret) {
        ESP_LOGE(GATTS_TAG, "Failed to add characteristic, err=0x%x", add_char_ret);
      }
      break;

    default:
      break;
  }
}

//--------------------------------------------------------------------------------------------------
/// @brief  GAP event handler
//--------------------------------------------------------------------------------------------------
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param) {
  switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
      adv_config_done &= (~adv_config_flag);
      if (adv_config_done == 0) {
        esp_err_t ret = esp_ble_gap_start_advertising(&adv_params);
        if (ret != ESP_OK) {
          ESP_LOGE(GATTS_TAG, "❌ Failed to start advertising: %s", esp_err_to_name(ret));
        } else {
          ESP_LOGI(GATTS_TAG, "📡 Initial advertising started.");
        }
      }
      break;

    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
      adv_config_done &= (~scan_rsp_config_flag);
      break;

    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
      if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
        ESP_LOGE(GATTS_TAG, "Failed to start advertising");
      } else {
        is_advertising = true;
        ESP_LOGI(GATTS_TAG, "Advertising started successfully");
      }
      break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
      if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
        ESP_LOGE(GATTS_TAG, "Failed to stop advertising");
      } else {
        is_advertising = false;
        ESP_LOGI(GATTS_TAG, "Advertising stopped successfully");
      }
      break;

    case ESP_GAP_BLE_SEC_REQ_EVT:
      if (bonding_mode_enabled) {
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        bonding_mode_enabled = false;
        ESP_LOGI(GATTS_TAG, "🔐 Bonding request accepted, disable bonding mode");
      } else {
        ESP_LOGW(GATTS_TAG, "🔒 Rejecting bonding without clearing bond data");
        esp_ble_gap_disconnect(param->ble_security.ble_req.bd_addr);
      }
      break;

    // Passkey
    case ESP_GAP_BLE_PASSKEY_REQ_EVT:
      ESP_LOGW(GATTS_TAG, "⚠️  PASSKEY entry requested (unexpected in Just Works)");
      break;

    case ESP_GAP_BLE_PASSKEY_NOTIF_EVT:
      ESP_LOGI(GATTS_TAG, "🔢 PASSKEY to show: %06" PRIu32, param->ble_security.key_notif.passkey);
      break;

    case ESP_GAP_BLE_NC_REQ_EVT:
      ESP_LOGW(GATTS_TAG, "🧩 Numeric Comparison requested (Not supported)");
      break;

    // Key event
    case ESP_GAP_BLE_KEY_EVT:
      ESP_LOGI(GATTS_TAG, "🗝️  BLE Key event: type = %d",
               param->ble_security.ble_key.key_type);
      break;

    case ESP_GAP_BLE_AUTH_CMPL_EVT: {
      esp_ble_auth_cmpl_t* auth_cmpl = &param->ble_security.auth_cmpl;

      if (auth_cmpl->success) {
        esp_bd_addr_t addr;
        memcpy(addr, auth_cmpl->bd_addr, sizeof(esp_bd_addr_t));

        int dev_num = esp_ble_get_bond_device_num();
        ESP_LOGI(GATTS_TAG, "🔍 Bonded device count: %d", dev_num);

        if (dev_num == 0) {
          ESP_LOGI(GATTS_TAG, "📭 No bonded devices in NVS.");
        }

        esp_ble_bond_dev_t* bonded_dev_list = malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
        if (!bonded_dev_list) {
          ESP_LOGE(GATTS_TAG, "❌ Failed to allocate memory for bonded device list.");
          break;
        }

        esp_err_t err = esp_ble_get_bond_device_list(&dev_num, bonded_dev_list);
        if (err != ESP_OK) {
          ESP_LOGE(GATTS_TAG, "❌ Failed to get bonded device list: %s", esp_err_to_name(err));
          free(bonded_dev_list);
          break;
        }

        // Debug: Print bonded list
        for (int i = 0; i < dev_num; i++) {
          ESP_LOGI(GATTS_TAG, "📝 [%d] %02x:%02x:%02x:%02x:%02x:%02x", i,
                   bonded_dev_list[i].bd_addr[0], bonded_dev_list[i].bd_addr[1],
                   bonded_dev_list[i].bd_addr[2], bonded_dev_list[i].bd_addr[3],
                   bonded_dev_list[i].bd_addr[4], bonded_dev_list[i].bd_addr[5]);
        }

        ESP_LOGI(GATTS_TAG, "🔐 Checking device: %02x:%02x:%02x:%02x:%02x:%02x", addr[0], addr[1],
                 addr[2], addr[3], addr[4], addr[5]);

        bool already_bonded = false;
        for (int i = 0; i < dev_num; i++) {
          if (memcmp(addr, bonded_dev_list[i].bd_addr, sizeof(esp_bd_addr_t)) == 0) {
            already_bonded = true;
            ESP_LOGW(GATTS_TAG,
                     "⚠️ Bonding request received from already bonded device: "
                     "%02x:%02x:%02x:%02x:%02x:%02x",
                     addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
            break;
          }
        }

        if (!already_bonded) {
          ESP_LOGI(GATTS_TAG, "✅ New device bonded: %02x:%02x:%02x:%02x:%02x:%02x", addr[0],
                   addr[1], addr[2], addr[3], addr[4], addr[5]);
          bonding_mode_enabled = false;
          ESP_LOGI(GATTS_TAG, "Bonding complete. Bonding mode disabled.");

          // Manaage devices by LRU
          remove_lru_bonded_device_if_needed();
          store_bonded_device_to_nvs(addr);
        }

        free(bonded_dev_list);

      } else {
        ESP_LOGE(GATTS_TAG, "❌ Bonding failed. Reason: 0x%x", auth_cmpl->fail_reason);
      }

      break;
    }

    case ESP_GATTS_CONNECT_EVT: {
      ESP_LOGI(GATTS_TAG, "GATTS Connect event occurred.");
      break;
    }

    case ESP_GATTS_DISCONNECT_EVT: {
      ESP_LOGI(GATTS_TAG, "🔌 Disconnected. Restart advertising...");
      esp_ble_gap_start_advertising(&adv_params);
      break;
    }

    default:
      break;
  }
}

//--------------------------------------------------------------------------------------------------
/// @brief  Main GATTS event handler
//--------------------------------------------------------------------------------------------------
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t* param) {
  // Register event
  if (event == ESP_GATTS_REG_EVT) {
    if (param->reg.status == ESP_GATT_OK) {
      gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
    } else {
      ESP_LOGI(GATTS_TAG, "Failed to register GATT, app_id=%04x, status=%d", param->reg.app_id,
               param->reg.status);
      return;
    }
  }

  // Dispatch to each profile's event handler
  for (int idx = 0; idx < PROFILE_NUM; idx++) {
    if (gatts_if == ESP_GATT_IF_NONE || gatts_if == gl_profile_tab[idx].gatts_if) {
      if (gl_profile_tab[idx].gatts_cb) {
        gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
      }
    }
  }
}

//--------------------------------------------------------------------------------------------------
/// @brief  Bonding timer callback for timeout
//--------------------------------------------------------------------------------------------------
static void bonding_timer_callback(void* arg) {
  bonding_mode_enabled = false;
  ESP_LOGI("BLE_SERVER", "🔒 Bonding mode timed out and disabled.");
}

//--------------------------------------------------------------------------------------------------
/// @brief  Check bonded device
//--------------------------------------------------------------------------------------------------
static bool is_bonded_device(esp_bd_addr_t addr) {
  int dev_num = esp_ble_get_bond_device_num();
  if (dev_num == 0) return false;

  esp_ble_bond_dev_t* bond_list = malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
  if (!bond_list) return false;

  esp_ble_get_bond_device_list(&dev_num, bond_list);
  for (int i = 0; i < dev_num; ++i) {
    if (memcmp(bond_list[i].bd_addr, addr, sizeof(esp_bd_addr_t)) == 0) {
      free(bond_list);
      return true;
    }
  }

  free(bond_list);
  return false;
}

//--------------------------------------------------------------------------------------------------
/// @brief  Store bonded device to NVS
//--------------------------------------------------------------------------------------------------
static void store_bonded_device_to_nvs(esp_bd_addr_t addr) {
  nvs_handle_t nvs_handle;
  esp_err_t err = nvs_open("bonding", NVS_READWRITE, &nvs_handle);
  if (err != ESP_OK) return;

  uint32_t count = 0;
  nvs_get_u32(nvs_handle, "bonded_count", &count);

  char key[32];
  snprintf(key, sizeof(key), "bonded_dev_%" PRIu32, count);
  err = nvs_set_blob(nvs_handle, key, addr, sizeof(esp_bd_addr_t));
  if (err == ESP_OK) {
    count++;
    nvs_set_u32(nvs_handle, "bonded_count", count);
    nvs_commit(nvs_handle);
  }

  nvs_close(nvs_handle);
}

//--------------------------------------------------------------------------------------------------
/// @brief  Remove LRU bonded device if needed
//--------------------------------------------------------------------------------------------------
static void remove_lru_bonded_device_if_needed(void) {
  nvs_handle_t nvs_handle;
  esp_err_t err = nvs_open("bonding", NVS_READWRITE, &nvs_handle);
  if (err != ESP_OK) return;

  uint32_t count = 0;
  nvs_get_u32(nvs_handle, "bonded_count", &count);

  if (count < MAX_BONDED_DEVICES) {
    nvs_close(nvs_handle);
    return;
  }

  // Remove the oldest device (bonded_dev_0)
  char key[32];
  esp_bd_addr_t addr;
  size_t len = sizeof(addr);
  snprintf(key, sizeof(key), "bonded_dev_0");
  if (nvs_get_blob(nvs_handle, key, addr, &len) == ESP_OK) {
    esp_ble_remove_bond_device(addr);
    ESP_LOGI("BLE", "Removed LRU bonded device: %02x:%02x:%02x:%02x:%02x:%02x", addr[0], addr[1],
             addr[2], addr[3], addr[4], addr[5]);
  }
  nvs_erase_key(nvs_handle, key);

  // Shift remaining entries left
  for (uint32_t i = 1; i < count; i++) {
    char old_key[32], new_key[32];
    snprintf(old_key, sizeof(old_key), "bonded_dev_%" PRIu32, i);
    snprintf(new_key, sizeof(new_key), "bonded_dev_%" PRIu32, i - 1);

    if (nvs_get_blob(nvs_handle, old_key, addr, &len) == ESP_OK) {
      nvs_set_blob(nvs_handle, new_key, addr, sizeof(addr));
      nvs_erase_key(nvs_handle, old_key);
    }
  }

  // Update count
  count--;
  nvs_set_u32(nvs_handle, "bonded_count", count);
  nvs_commit(nvs_handle);
  nvs_close(nvs_handle);
}
