// HIL: Adafruit TinyUSB device (usbstack=tinyusb) + LowPower dormant.
// The SDK's low_power calls tud_deinit(0) / tud_init(0) on the library's TinyUSB
// around dormant; check the CDC device comes back (re-enumerates, mounts, sends).
// Logs on Serial1 (UART0 GP0/GP1, Debug Probe UART) and on USB CDC Serial.
#include <Adafruit_TinyUSB.h>
#include <LowPower.h>

// RP2040 timed dormant needs an external clock (LowPower.dormantFor() returns
// PICO_ERROR_PRECONDITION_NOT_MET without one), so there wake on a falling edge on
// GP1 = UART0 RX: any byte the bench sends through the Debug Probe UART.
static int doDormant(uint32_t ms) {
#if defined(PICO_RP2040)
    (void) ms;
    return LowPower.dormantUntilPin(1, FALLING);
#else
    return LowPower.dormantFor(ms);
#endif
}

void setup() {
    Serial1.begin(115200);
    Serial.begin(115200);
    delay(200);
    Serial1.println("HIL_LP device-mode start");
}

void loop() {
    static int n = 0;
    uint32_t t = millis();
    while (!TinyUSBDevice.mounted() && millis() - t < 10000) {
        delay(10);
    }
    Serial1.printf("CYCLE %d mounted=%d wait=%lums\n", n, (int)TinyUSBDevice.mounted(), (unsigned long)(millis() - t));
    if (n >= 4) {
        Serial1.println("HIL_LP device-mode DONE");
        Serial.println("HIL_LP device-mode DONE");
        delay(5000);
        return;
    }
    for (int i = 0; i < 16; i++) {
        Serial.printf("CDC tick %d.%d\n", n, i);
        Serial1.printf("UART tick %d.%d\n", n, i);
        delay(500);
    }
    Serial.flush();
    Serial1.printf("ENTER_DORMANT %d\n", n);
    Serial1.flush();
    int rc = doDormant(4000);
    Serial1.printf("WAKE %d rc=%d mounted=%d\n", n, rc, (int)TinyUSBDevice.mounted());
    n++;
}
