//==================================================================================================
/// @file       smartlock.c
/// @brief      Electric lock and LED control by GPIO
/// @date       2025/1/4
//==================================================================================================

//==================================================================================================
// Include definition
//==================================================================================================
#include "smartlock.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

//==================================================================================================
// Constant definition
//==================================================================================================
#define SMARTLOCK_TAG "SMARTLOCK"

// GPIO pin for the electric lock
#define LOCK_GPIO_PIN 5

// GPIO pins for status LEDs
#define LED_BLUETOOTH 21 // LED1 (blink on BLE reception)
#define LED_UNLOCK 22    // LED2 (blink on unlock)

//==================================================================================================
// Prototype
//==================================================================================================
static void blink_led_once(int gpio_num, int duration_ms);

//==================================================================================================
// Private functions
//==================================================================================================

//--------------------------------------------------------------------------------------------------
/// @brief  Blink a LED for a short duration.
//--------------------------------------------------------------------------------------------------
static void blink_led_once(int gpio_num, int duration_ms) {
  gpio_set_level(gpio_num, 1);
  vTaskDelay(pdMS_TO_TICKS(duration_ms));
  gpio_set_level(gpio_num, 0);
}

//==================================================================================================
// Public functions
//==================================================================================================

//--------------------------------------------------------------------------------------------------
/// @brief  Initialize GPIO pins for lock and LEDs.
//--------------------------------------------------------------------------------------------------
void smartlock_init(void) {
  ESP_LOGI(SMARTLOCK_TAG, "Initializing Smart Lock GPIO...");

  // Lock pin
  gpio_reset_pin(LOCK_GPIO_PIN);
  gpio_set_direction(LOCK_GPIO_PIN, GPIO_MODE_OUTPUT);
  gpio_set_level(LOCK_GPIO_PIN, 0);

  // LED for Bluetooth reception
  gpio_reset_pin(LED_BLUETOOTH);
  gpio_set_direction(LED_BLUETOOTH, GPIO_MODE_OUTPUT);
  gpio_set_level(LED_BLUETOOTH, 0);

  // LED for unlock
  gpio_reset_pin(LED_UNLOCK);
  gpio_set_direction(LED_UNLOCK, GPIO_MODE_OUTPUT);
  gpio_set_level(LED_UNLOCK, 0);
}

//--------------------------------------------------------------------------------------------------
/// @brief  Blink LED1 when Bluetooth data is received.
//--------------------------------------------------------------------------------------------------
void smartlock_bluetooth_received(void) {
  // Blink LED_BLUETOOTH (LED1)
  blink_led_once(LED_BLUETOOTH, 200); // blink for 200ms
}

//--------------------------------------------------------------------------------------------------
/// @brief  Unlock routine: blink LED2 and activate the lock for 0.5 sec
//--------------------------------------------------------------------------------------------------
void smartlock_unlock(void) {
  ESP_LOGI(SMARTLOCK_TAG, "Smartlock unlock triggered.");

  // Blink LED_UNLOCK (LED2)
  blink_led_once(LED_UNLOCK, 300); // blink for 300ms

  // Activate the lock
  gpio_set_level(LOCK_GPIO_PIN, 1);
  vTaskDelay(pdMS_TO_TICKS(500)); // 0.5 second
  gpio_set_level(LOCK_GPIO_PIN, 0);

  ESP_LOGI(SMARTLOCK_TAG, "Unlock process complete.");
}
