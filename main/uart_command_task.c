//==================================================================================================
/// @file       uart_command_task.c
/// @brief      UART command handler task
/// @date       2025/7/31
//==================================================================================================

//==================================================================================================
// Include definition
//==================================================================================================
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ble_server.h"
#include "smartlock.h"
#include "uart_command_task.h"

//==================================================================================================
// Constant definition
//==================================================================================================
#define MAX_LINE_LEN 128

//==================================================================================================
// Prototype
//==================================================================================================
static void cmd_help(const char* args);
static void cmd_version(const char* args);
static void cmd_unlock(const char* args);
static void uart_cmd_task(void* arg);
// static void cmd_advertise(const char* args);
static void cmd_start_advertise(const char* args);
static void cmd_stop_advertise(const char* args);
static void cmd_bond_list(const char* args);
static void cmd_bond_clear(const char* args);
static void cmd_enable_bonding(const char* args);
static void cmd_disable_bonding(const char* args);
static void cmd_status(const char* args);

//==================================================================================================
// Private values definition
//==================================================================================================

typedef void (*cmd_func_t)(const char* args);

typedef struct {
  const char* name;
  const char* help;
  cmd_func_t handler;
} command_entry_t;

static const command_entry_t command_table[] = {
    {"help", "Show this help message", cmd_help},
    {"h", "Show this help message", cmd_help},
    {"version", "Show firmware version", cmd_version},
    {"v", "Show firmware version", cmd_version},
    {"unlock", "Unlock test", cmd_unlock},
    {"enable_bond", "Enable bonding mode", cmd_enable_bonding},
    {"en", "Enable bonding mode", cmd_enable_bonding},
    {"disable_bond", "Disable bonding mode", cmd_disable_bonding},
    {"dis", "Disable bonding mode", cmd_disable_bonding},
    {"start_advertise", "Start BLE advertising", cmd_start_advertise},
    {"start", "Start BLE advertising", cmd_start_advertise},
    {"stop_advertise", "Stop BLE advertising", cmd_stop_advertise},
    {"stop", "Stop BLE advertising", cmd_stop_advertise},
    {"list", "Show bonded BLE device list", cmd_bond_list},
    {"clear", "Clear all bonded BLE devices", cmd_bond_clear},
    {"status", "Show status adv|bond", cmd_status},
};

//==================================================================================================
// Public functions
//==================================================================================================

void uart_command_task_start(void) {
  xTaskCreate(uart_cmd_task, "uart_cmd_task", 4096, NULL, 5, NULL);
}

//==================================================================================================
// Private functions
//==================================================================================================

static void cmd_help(const char* args) {
  printf("\nCommand List:\n");
  for (size_t i = 0; i < sizeof(command_table) / sizeof(command_table[0]); i++) {
    printf("  %-10s - %s\n", command_table[i].name, command_table[i].help);
  }
}

static void cmd_version(const char* args) {
  printf("\nProduct name: BLE Server\nFW Version: %s\n", CONFIG_FW_VERSION);
}

static void cmd_unlock(const char* args) { smartlock_unlock(); }

static void cmd_start_advertise(const char* args) {
  printf("Start BLE bonding mode\n");
  ble_server_start_advertising(3 * 60 * 1000);
}

static void cmd_stop_advertise(const char* args) {
  printf("Stop BLE advertising\n");
  ble_server_stop_advertising();
}

static void cmd_bond_list(const char* args) { ble_server_print_bonded_devices(); }

static void cmd_bond_clear(const char* args) { ble_server_clear_bonded_devices(); }

static void cmd_enable_bonding(const char* args) {
  ble_server_enable_bonding_mode();
  printf("🔓 Bonding mode enabled.\n");
}

static void cmd_disable_bonding(const char* args) {
  ble_server_disable_bonding_mode();
  printf("🔒 Bonding mode disabled\n");
}

static void cmd_status(const char* args) {
  printf("=== BLE Server Status ===\n");
  printf("Advertising:    %s\n", ble_server_is_advertising() ? "ON" : "OFF");
  printf("Bonding Mode:   %s\n", ble_server_is_bonding_mode_enabled() ? "ON" : "OFF");
}

//--------------------------------------------------------------------------------------------------
/// @brief  UART command handler task
//--------------------------------------------------------------------------------------------------
static void uart_cmd_task(void* arg) {
  char line[MAX_LINE_LEN];

  while (1) {
    // Read line from stdin
    if (fgets(line, sizeof(line), stdin) == NULL) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    // Trim newline
    char* newline = strchr(line, '\n');
    if (newline) *newline = '\0';

    // Skip empty lines
    if (strlen(line) == 0) continue;

    // Extract command and args
    char* cmd = strtok(line, " \t");
    // get remaining part after first space
    char* args = strtok(NULL, "");

    if (!cmd) continue;  // safety check

    // Search and execute matching command
    bool matched = false;
    for (size_t i = 0; i < sizeof(command_table) / sizeof(command_table[0]); i++) {
      if (strcmp(cmd, command_table[i].name) == 0) {
        // Call handler with args (can be NULL)
        command_table[i].handler(args);
        matched = true;
        break;
      }
    }

    if (!matched) {
      printf("Unknown command: %s\n", cmd);
      cmd_help(NULL);
    }
  }
}
