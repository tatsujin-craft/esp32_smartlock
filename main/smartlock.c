//==================================================================================================
/// @file       smartlock.c
/// @brief      Electric lock and LED control by GPIO
/// @date       2025/1/4
//==================================================================================================

//==================================================================================================
// Include definition
//==================================================================================================
#include "driver/gpio.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ble_server.h"
#include "smartlock.h"

//==================================================================================================
// Constant definition
//==================================================================================================
#define SMARTLOCK_TAG "SMARTLOCK"

// GPIO pin for the electric lock
#define LOCK_GPIO_PIN 5

// GPIO pins for status LEDs
#define LED_BLUETOOTH 21  // LED1 (blink on BLE reception)
#define LED_UNLOCK 22     // LED2 (blink on unlock)

// Solenoid (Electromagnetic Lock)
#define SOLENOID_POWER_ON_MS 800

#define BUTTON_BONDING GPIO_NUM_4
#define BONDING_MODE_DURATION_MS (5 * 60 * 1000)

//==================================================================================================
// Prototype
//==================================================================================================
static void blink_led_once(int gpio_num, int duration_ms);
static void smart_lock_task(void* arg);

//==================================================================================================
// Private values definition
//==================================================================================================
static volatile bool button_pressed = false;

// Interrupt handler for bonding button (ISR context)
static void IRAM_ATTR bonding_button_isr_handler(void* arg) { button_pressed = true; }

//==================================================================================================
// Public functions
//==================================================================================================

// Initialize GPIO pins, interrupts, and start Smart Lock task
void smart_lock_task_start(void) {
  ESP_LOGI(SMARTLOCK_TAG, "Initializing Smart Lock GPIO...");

  // Configure Lock GPIO
  gpio_reset_pin(LOCK_GPIO_PIN);
  gpio_set_direction(LOCK_GPIO_PIN, GPIO_MODE_OUTPUT);
  gpio_set_level(LOCK_GPIO_PIN, 0);

  // Configure Bluetooth status LED GPIO
  gpio_reset_pin(LED_BLUETOOTH);
  gpio_set_direction(LED_BLUETOOTH, GPIO_MODE_OUTPUT);
  gpio_set_level(LED_BLUETOOTH, 0);

  // Configure Unlock status LED GPIO
  gpio_reset_pin(LED_UNLOCK);
  gpio_set_direction(LED_UNLOCK, GPIO_MODE_OUTPUT);
  gpio_set_level(LED_UNLOCK, 0);

  // Configure Bonding button GPIO (input with pull-up, interrupt on falling edge)
  gpio_config_t button_config = {.pin_bit_mask = (1ULL << BUTTON_BONDING),
                                 .mode = GPIO_MODE_INPUT,
                                 //  .pull_up_en = GPIO_PULLUP_ENABLE,
                                 .pull_up_en = GPIO_PULLUP_DISABLE,
                                 .pull_down_en = GPIO_PULLDOWN_DISABLE,
                                 .intr_type = GPIO_INTR_NEGEDGE};
  gpio_config(&button_config);

  // Install GPIO ISR service and attach interrupt handler
  gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);
  gpio_isr_handler_add(BUTTON_BONDING, bonding_button_isr_handler, NULL);

  // Create Smart Lock monitoring task
  xTaskCreate(smart_lock_task, "smart_lock_task", 2048, NULL, 5, NULL);
}

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
  blink_led_once(LED_BLUETOOTH, 200);  // blink for 200ms
}

//--------------------------------------------------------------------------------------------------
/// @brief  Unlock routine: blink LED2 and activate the lock for 0.5 sec
//--------------------------------------------------------------------------------------------------
void smartlock_unlock(void) {
  ESP_LOGI(SMARTLOCK_TAG, "Smartlock unlock triggered.");

  // Blink LED_UNLOCK (LED2)
  blink_led_once(LED_UNLOCK, 300);  // blink for 300ms

  // Activate the lock
  gpio_set_level(LOCK_GPIO_PIN, 1);
  // vTaskDelay(pdMS_TO_TICKS(500)); // 0.5 second
  vTaskDelay(pdMS_TO_TICKS(SOLENOID_POWER_ON_MS));  // 0.8 second
  gpio_set_level(LOCK_GPIO_PIN, 0);

  ESP_LOGI(SMARTLOCK_TAG, "Unlock process complete.");
}

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

//--------------------------------------------------------------------------------------------------
/// @brief  Smart Lock task
//--------------------------------------------------------------------------------------------------
static void smart_lock_task(void* arg) {
  bool led_state = false;
  TickType_t last_blink_time = xTaskGetTickCount();
  bool is_debug = false;

  while (1) {
    if (button_pressed) {
      ESP_LOGI(SMARTLOCK_TAG, "Bonding button pressed. Enabling bonding mode...");
      ble_server_enable_bonding_mode();
      button_pressed = false;
    }

    // LED blink
    if (ble_server_is_bonding_mode_enabled()) {
      if ((xTaskGetTickCount() - last_blink_time) >= pdMS_TO_TICKS(500)) {
        led_state = !led_state;
        if (is_debug)
          gpio_set_level(LED_BLUETOOTH, led_state);
        else
          ESP_LOGI(SMARTLOCK_TAG, "LED blinking.");
        last_blink_time = xTaskGetTickCount();
      }
    } else {
      gpio_set_level(LED_BLUETOOTH, 0);
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}