// HIL: LowPower light sleep + dormant cycles on a Pico W / Pico 2 W with Adafruit TinyUSB,
// device (usbstack=tinyusb) or native host (usbstack=tinyusb_host).
// Logs on Serial1 (UART0 GP0/GP1 = Debug Probe UART).  Pin wakes use GP1 (UART0 RX): the bench
// sends a byte through the probe when a line says WAKEBYTE.
// Schedule: L = sleepFor(2000), P = sleepUntilPin(GP1, FALLING), D = dormantUntilPin(GP1, FALLING),
// T = dormantFor(3000) (RP2350 only; RP2040 timed dormant needs an external clock).
#include <Adafruit_TinyUSB.h>
#include <LowPower.h>
#include <hardware/structs/usb.h>
#include <hardware/sync.h>

#if defined(PICO_RP2040)
static const char sched[] = "LPLPDDDDDDDD";
#else
static const char sched[] = "LPLPDTDTDTDT";
#endif

#if defined(USE_TINYUSB_HOST)
Adafruit_USBH_Host USBHost;
static void hostTask(uint32_t ms) {
    uint32_t t = millis();
    while (millis() - t < ms) {
        USBHost.task(0);
    }
}
static void dumpUsb(const char *tag, int n) {
    Serial1.printf("%s %d tuh_inited=%d main_ctrl=%08lx sie_ctrl=%08lx inte=%08lx\n", tag, n, (int)tuh_inited(),
                   (unsigned long)usb_hw->main_ctrl, (unsigned long)usb_hw->sie_ctrl, (unsigned long)usb_hw->inte);
}
#endif

static int claimedSpinLocks() {
    int c = 0;
    for (uint i = 0; i < NUM_SPIN_LOCKS; i++) {
        c += spin_lock_is_claimed(i) ? 1 : 0;
    }
    return c;
}

void setup() {
    Serial1.begin(115200);
#if defined(USE_TINYUSB_HOST)
    USBHost.begin(0);
    Serial1.printf("HIL380 start host tusb=%d.%d.%d sched=%s\n", TUSB_VERSION_MAJOR, TUSB_VERSION_MINOR, TUSB_VERSION_REVISION, sched);
#else
    Serial.begin(115200);
    Serial1.printf("HIL380 start device tusb=%d.%d.%d sched=%s\n", TUSB_VERSION_MAJOR, TUSB_VERSION_MINOR, TUSB_VERSION_REVISION, sched);
#endif
}

void loop() {
    static uint n = 0;
    if (n >= sizeof(sched) - 1) {
        Serial1.println("HIL380 DONE");
#if !defined(USE_TINYUSB_HOST)
        Serial.println("HIL380 DONE");
#endif
        delay(5000);
        return;
    }
    char m = sched[n];
#if defined(USE_TINYUSB_HOST)
    hostTask(1500);
    dumpUsb("PRE", n);
#else
    uint32_t t = millis();
    while (!TinyUSBDevice.mounted() && millis() - t < 10000) {
        delay(10);
    }
    Serial1.printf("CYCLE %d mounted=%d wait=%lums spinlocks=%d\n", n, (int)TinyUSBDevice.mounted(), (unsigned long)(millis() - t),
                   claimedSpinLocks());
    for (int i = 0; i < 4; i++) {
        Serial.printf("CDC %d.%d\n", n, i);
        delay(250);
    }
    Serial.flush();
#endif
    bool pin = (m == 'P' || m == 'D');
    Serial1.printf("ENTER %c %d%s\n", m, n, pin ? " WAKEBYTE" : "");
    Serial1.flush();
    int rc;
    switch (m) {
    case 'L': rc = LowPower.sleepFor(2000); break;
    case 'P': rc = LowPower.sleepUntilPin(1, FALLING); break;
    case 'D': rc = LowPower.dormantUntilPin(1, FALLING); break;
    default: rc = LowPower.dormantFor(3000); break;
    }
#if defined(USE_TINYUSB_HOST)
    Serial1.printf("WAKE %c %d rc=%d\n", m, n, rc);
    dumpUsb("POST", n);
    hostTask(500);
    Serial1.printf("ALIVE %d spinlocks=%d\n", n, claimedSpinLocks());
#else
    Serial1.printf("WAKE %c %d rc=%d mounted=%d\n", m, n, rc, (int)TinyUSBDevice.mounted());
#endif
    n++;
}
