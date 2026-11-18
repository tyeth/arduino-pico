// HIL trace: Adafruit TinyUSB device (or the core's SDK stack, as a control) + LowPower dormant, 8 cycles.
// Counts the claimed hardware spin locks and user IRQs before each dormant and after each wake, and
// hooks (link-time --wrap) the libpico low_power calls into the USB stack plus panic(), printing raw
// to UART0 (Serial1, GP0) so a panic inside the SDK's post-wake tud_init() is visible on the bench.
// Build with: compiler.c.elf.extra_flags=-Wl,--wrap=tud_deinit,--wrap=tud_rhport_init,--wrap=panic
#ifdef USE_TINYUSB
#include <Adafruit_TinyUSB.h>
#endif
#include <LowPower.h>
#include <stdarg.h>
#include <hardware/sync.h>
#include <hardware/irq.h>
#include <hardware/uart.h>
#include <hardware/clocks.h>
#include <tusb.h>

static const int CYCLES = 8;

static unsigned spinClaimed(uint32_t *mask) {
    unsigned n = 0;
    uint32_t m = 0;
    for (uint i = PICO_SPINLOCK_ID_CLAIM_FREE_FIRST; i <= PICO_SPINLOCK_ID_CLAIM_FREE_LAST; i++) {
        if (spin_lock_is_claimed(i)) {
            n++;
            m |= 1u << i;
        }
    }
    if (mask) {
        *mask = m;
    }
    return n;
}

static uint32_t userIrqClaimed() {
    uint32_t m = 0;
    for (uint i = FIRST_USER_IRQ; i < FIRST_USER_IRQ + NUM_USER_IRQS; i++) {
        if (user_irq_is_claimed(i)) {
            m |= 1u << (i - FIRST_USER_IRQ);
        }
    }
    return m;
}

// Raw UART0 print usable inside the SDK's clock switch: programs the divisor for the current clk_peri,
// prints, then puts Serial1's divisor back.
static void rawPrint(const char *s) {
    if (!(uart0_hw->cr & UART_UARTCR_UARTEN_BITS)) {
        return;
    }
    uart_tx_wait_blocking(uart0);
    uint32_t ib = uart0_hw->ibrd, fb = uart0_hw->fbrd;
    uart_set_baudrate(uart0, 115200);
    while (*s) {
        while (uart0_hw->fr & UART_UARTFR_TXFF_BITS) {
        }
        uart0_hw->dr = *s++;
    }
    uart_tx_wait_blocking(uart0);
    uart0_hw->ibrd = ib;
    uart0_hw->fbrd = fb;
    hw_set_bits(&uart0_hw->lcr_h, 0);
}

static void rawPrintf(const char *fmt, ...) {
    char b[160];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(b, sizeof(b), fmt, ap);
    va_end(ap);
    rawPrint(b);
}

extern "C" {
    bool __real_tud_deinit(uint8_t rhport);
    bool __wrap_tud_deinit(uint8_t rhport) {
        uint32_t m0, m1;
        unsigned n0 = spinClaimed(&m0);
        bool r = __real_tud_deinit(rhport);
        unsigned n1 = spinClaimed(&m1);
        rawPrintf("  [lp] tud_deinit rc=%d spinlocks %u->%u (0x%08lx->0x%08lx)\r\n", (int)r, n0, n1, (unsigned long)m0, (unsigned long)m1);
        return r;
    }
    bool __real_tud_rhport_init(uint8_t rhport, const void *rh_init);
    bool __wrap_tud_rhport_init(uint8_t rhport, const void *rh_init) {
        uint32_t m0, m1;
        unsigned n0 = spinClaimed(&m0);
        rawPrintf("  [lp] tud_rhport_init enter spinlocks=%u (0x%08lx)\r\n", n0, (unsigned long)m0);
        bool r = __real_tud_rhport_init(rhport, rh_init);
        unsigned n1 = spinClaimed(&m1);
        rawPrintf("  [lp] tud_rhport_init rc=%d spinlocks %u->%u (0x%08lx)\r\n", (int)r, n0, n1, (unsigned long)m1);
        return r;
    }
    void __real_panic(const char *fmt, ...);
    void __wrap_panic(const char *fmt, ...) {
        uint32_t m;
        unsigned n = spinClaimed(&m);
        rawPrintf("\r\nPANIC: \"%s\" spinlocks=%u (0x%08lx)\r\n", fmt ? fmt : "(null)", n, (unsigned long)m);
        while (true) {
            tight_loop_contents();
        }
    }
}

// RP2040 timed dormant needs an external clock, so there wake on a falling edge on GP1 = UART0 RX
// (any byte the bench sends through the Debug Probe UART).
static int doDormant(uint32_t ms) {
#if defined(PICO_RP2040)
    (void) ms;
    return LowPower.dormantUntilPin(1, FALLING);
#else
    return LowPower.dormantFor(ms);
#endif
}

static void state(const char *tag, int n) {
    uint32_t m;
    unsigned c = spinClaimed(&m);
    Serial1.printf("STATE %s %d spinlocks=%u/%u mask=0x%08lx userirq=0x%02lx tud_inited=%d clk_sys=%lu\n", tag, n, c,
                   (unsigned)(PICO_SPINLOCK_ID_CLAIM_FREE_LAST - PICO_SPINLOCK_ID_CLAIM_FREE_FIRST + 1), (unsigned long)m,
                   (unsigned long)userIrqClaimed(), (int)tud_inited(), (unsigned long)clock_get_hz(clk_sys));
}

static bool usbMounted() {
#ifdef USE_TINYUSB
    return TinyUSBDevice.mounted();
#else
    return tud_mounted();
#endif
}

void setup() {
    Serial1.begin(115200);
    delay(200);
#ifdef USE_TINYUSB
    Serial1.println("HIL_LP trace start stack=adafruit");
#else
    Serial1.println("HIL_LP trace start stack=sdk");
#endif
}

void loop() {
    static int n = 0;
    uint32_t t = millis();
    while (!usbMounted() && millis() - t < 10000) {
        delay(10);
    }
    Serial1.printf("CYCLE %d mounted=%d wait=%lums\n", n, (int)usbMounted(), (unsigned long)(millis() - t));
    if (n >= CYCLES) {
        state("final", n);
        Serial1.println("HIL_LP device-mode DONE");
        delay(5000);
        return;
    }
    for (int i = 0; i < 6; i++) {
        Serial1.printf("UART tick %d.%d\n", n, i);
        delay(500);
    }
    state("pre", n);
    Serial1.printf("ENTER_DORMANT %d\n", n);
    Serial1.flush();
    int rc = doDormant(4000);
    Serial1.printf("WAKE %d rc=%d mounted=%d\n", n, rc, (int)usbMounted());
    state("post", n);
    n++;
}
