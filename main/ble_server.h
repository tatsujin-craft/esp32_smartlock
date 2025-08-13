#ifndef BLE_SERVER_H
#define BLE_SERVER_H

#ifdef __cplusplus
extern "C" {
#endif

// Initializes the BLE GATT server
void ble_server_init(void);
void ble_server_start_advertising(uint32_t duration_ms);
void ble_server_stop_advertising(void);
void ble_server_enable_bonding_mode(void);
void ble_server_disable_bonding_mode(void);
bool ble_server_is_bonding_mode_enabled(void);
bool ble_server_is_advertising(void);
void ble_server_print_bonded_devices(void);
void ble_server_clear_bonded_devices(void);

#ifdef __cplusplus
}
#endif

#endif  // BLE_SERVER_H
