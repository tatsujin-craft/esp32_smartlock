//==================================================================================================
/// @file       main.c
/// @brief      Main application
/// @date       2025/1/4
//==================================================================================================

//==================================================================================================
// Include definition
//==================================================================================================
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include <stdio.h>

#include "ble_server.h"
#include "smartlock.h"

//==================================================================================================
// Constant definition
//==================================================================================================
#define MAIN_TAG "MAIN_APP"

//--------------------------------------------------------------------------------------------------
/// @brief  Main entry point
//--------------------------------------------------------------------------------------------------
void app_main(void) {
  esp_err_t ret;

  // Initialize NVS
  ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // Initialize Smart Lock (GPIO pins for lock + LEDs)
  smartlock_init();

  // Release memory for Classic Bluetooth
  ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

  // Initialize BT controller
  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  ret = esp_bt_controller_init(&bt_cfg);
  if (ret) {
    ESP_LOGE(MAIN_TAG, "Failed to init controller: %s", esp_err_to_name(ret));
    return;
  }

  // Enable BLE mode
  ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
  if (ret) {
    ESP_LOGE(MAIN_TAG, "Failed to enable controller: %s", esp_err_to_name(ret));
    return;
  }

  // Initialize Bluedroid stack
  esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
  ret = esp_bluedroid_init_with_cfg(&bluedroid_cfg);
  if (ret) {
    ESP_LOGE(MAIN_TAG, "Failed to init Bluedroid: %s", esp_err_to_name(ret));
    return;
  }

  // Enable Bluedroid
  ret = esp_bluedroid_enable();
  if (ret) {
    ESP_LOGE(MAIN_TAG, "Failed to enable Bluedroid: %s", esp_err_to_name(ret));
    return;
  }

  // Initialize and start BLE GATT server
  ble_server_init();

  ESP_LOGI(MAIN_TAG, "Main app initialized.");
}
