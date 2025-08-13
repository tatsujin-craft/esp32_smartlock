//==================================================================================================
/// @file       main.c
/// @brief      Main application
/// @date       2025/1/4
//==================================================================================================

//==================================================================================================
// Include definition
//==================================================================================================
#include <stdio.h>
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include "ble_server.h"
#include "smartlock.h"
#include "uart_command_task.h"

//==================================================================================================
// Constant definition
//==================================================================================================
#define MAIN_TAG "MAIN_APP"

//--------------------------------------------------------------------------------------------------
/// @brief  Main application
//--------------------------------------------------------------------------------------------------
void app_main(void) {
  // Initialize NVS flash storage
  ESP_ERROR_CHECK(nvs_flash_init());

  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // Initialize BLE server (controller, Bluedroid, GATT server)
  ble_server_init();

  // Initialize Smart Lock task (GPIO, interrupt, task creation)
  smart_lock_task_start();

  // Start UART command handler
  uart_command_task_start();

  ESP_LOGI(MAIN_TAG, "Main app initialized.");
}