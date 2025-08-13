#ifndef SMARTLOCK_H
#define SMARTLOCK_H

#ifdef __cplusplus
extern "C" {
#endif

void smart_lock_task_start(void);

// Initialize lock/LED GPIO pins
void smartlock_init(void);

// Blink LED1 (for Bluetooth reception)
void smartlock_bluetooth_received(void);

// Unlock routine (blink LED2 and power the lock)
void smartlock_unlock(void);

#ifdef __cplusplus
}
#endif

#endif // SMARTLOCK_H
