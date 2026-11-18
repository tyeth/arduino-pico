// HIL: LowPower dormant with no USB-specific code (logs on Serial1, UART0 GP0/GP1).
// Build with different usbstack / board options to isolate repeated-dormant failures.
#include <LowPower.h>

// RP2040 timed dormant needs an external clock, so there wake on a falling edge on
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
    delay(200);
    Serial1.println("HIL_LP plain start");
}

void loop() {
    static int n = 0;
    if (n >= 6) {
        Serial1.println("HIL_LP plain DONE");
        delay(5000);
        return;
    }
    for (int i = 0; i < 4; i++) {
        Serial1.printf("UART tick %d.%d\n", n, i);
        delay(500);
    }
    Serial1.printf("ENTER_DORMANT %d\n", n);
    Serial1.flush();
    int rc = doDormant(3000);
    Serial1.printf("WAKE %d rc=%d\n", n, rc);
    n++;
}
