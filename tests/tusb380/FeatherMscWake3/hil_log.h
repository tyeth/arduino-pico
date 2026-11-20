// Forced-included (-include) into every translation unit of the FeatherMscFixDbg build so the Adafruit TinyUSB
// debug printf (CFG_TUSB_DEBUG_PRINTF, which writes to SERIAL_TUSB_DEBUG) can target a lock-free ring buffer:
// TinyUSB host logging runs on core1 (task and PIO-USB IRQ context) and must not block on USB CDC; core0 drains it.
#pragma once
#ifndef HIL_LOG_H
#define HIL_LOG_H
#ifdef __cplusplus
#include <stddef.h>
#include <stdint.h>

class HilLog {
public:
    void begin(unsigned long) {}
    size_t write(const char *s); // producers: either core (spin lock)
    size_t drain(char *dst, size_t max); // consumer: core0 only
    uint32_t dropped() const {
        return _dropped;
    }

private:
    static const uint32_t SIZE = 8192;
    char _buf[SIZE];
    volatile uint32_t _head = 0, _tail = 0, _dropped = 0;
};
extern HilLog hilLog;
#endif
#endif // HIL_LOG_H
