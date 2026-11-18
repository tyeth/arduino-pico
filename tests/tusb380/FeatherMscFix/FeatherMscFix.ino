// HIL (Feather RP2040 USB Host, usbstack=tinyusb, freq=120): stale-device test for the Pico-PIO-USB MSC host.
// LowPower freezes core1 (and the PIO-USB SOF timer that polls for disconnects) during light sleep and dormant, so
// a device swapped while asleep leaves the root port "connected" to a stale address.  Each cycle light-sleeps 15 s
// (timer wake, so the bench can power-cycle the USB-A device via its EN pin and let it fully reboot meanwhile):
//   L = no recovery: read straight away.  Stock library: the read never completes; fixed library: it fails fast,
//       then the sketch forces a reconnect and reads again.
//   R = proactive: force a reconnect right after waking, then read.
// forceReconnect() marks the root port disconnected on core1, exactly as Pico-PIO-USB does when it sees SE0, so the
// next frame re-detects the device and TinyUSB re-enumerates it.
#include "pio_usb.h"
#include "SdFat_Adafruit_Fork.h"
#include <Adafruit_TinyUSB.h>
#include <LowPower.h>
#include <stdarg.h>
#include <hardware/clocks.h>

static const char sched[] = "LRLR";

Adafruit_USBH_Host USBHost;
Adafruit_USBH_MSC_BlockDevice msc_block_dev;
FatVolume fatfs;
static volatile bool fsMounted = false;
static volatile uint32_t mscMounts = 0;
static volatile bool reconnectReq = false;

struct Ev {
    uint32_t ms;
    char kind;
    uint8_t addr;
};
static Ev evs[32];
static volatile uint32_t evHead = 0;
static uint32_t evPrinted = 0;

static void addEv(char kind, uint8_t addr) {
    Ev &e = evs[evHead % 32];
    e.ms = millis();
    e.kind = kind;
    e.addr = addr;
    evHead = evHead + 1;
}

extern "C" {
    void tuh_mount_cb(uint8_t addr) {
        addEv('M', addr);
    }
    void tuh_umount_cb(uint8_t addr) {
        addEv('U', addr);
    }
    void tuh_msc_mount_cb(uint8_t addr) {
        msc_block_dev.begin(addr);
        msc_block_dev.setActiveLUN(0);
        fsMounted = fatfs.begin(&msc_block_dev);
        mscMounts = mscMounts + 1;
        addEv(fsMounted ? 'F' : 'f', addr);
    }
    void tuh_msc_umount_cb(uint8_t addr) {
        fsMounted = false;
        fatfs.end();
        msc_block_dev.end();
        addEv('u', addr);
    }
}

// pio_reconnect.c (C, since pio_usb_ll.h is not C++-clean): mark the root port disconnected and fail its transfers
extern "C" void pio_usb_force_reconnect(uint8_t root_idx);

// Runs on core1, which owns the PIO-USB host
static void doForceReconnect() {
    pio_usb_force_reconnect(0);
    addEv('R', 0);
}

void setup1() {
    pinMode(PIN_5V_EN, OUTPUT);
    digitalWrite(PIN_5V_EN, HIGH);
    pio_usb_configuration_t cfg = PIO_USB_DEFAULT_CONFIG;
    cfg.pin_dp = PIN_USB_HOST_DP;
    USBHost.configure_pio_usb(1, &cfg);
    USBHost.begin(1);
}

void loop1() {
    if (reconnectReq) {
        doForceReconnect();
        reconnectReq = false;
    }
    USBHost.task();
}

static void logf(const char *fmt, ...) {
    char buf[200];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    Serial.print(buf);
    Serial.flush();
}

static void flushEvents() {
    while (evPrinted < evHead) {
        const Ev &e = evs[evPrinted % 32];
        logf("EV %c t=%lu addr=%u\n", e.kind, (unsigned long)e.ms, e.addr);
        evPrinted++;
    }
}

static uint32_t waitHost(uint32_t ms) {
    uint32_t t = millis();
    while (!(TinyUSBDevice.mounted() && Serial) && millis() - t < ms) {
        delay(10);
    }
    return millis() - t;
}

// Wait for a *new* MSC mount (mount count past `before`) with the filesystem up
static bool waitRemount(uint32_t before, uint32_t ms) {
    uint32_t t = millis();
    while (!(fsMounted && mscMounts != before) && millis() - t < ms) {
        flushEvents();
        delay(10);
    }
    flushEvents();
    return fsMounted && mscMounts != before;
}

static bool waitMounted(uint32_t ms) {
    uint32_t t = millis();
    while (!fsMounted && millis() - t < ms) {
        flushEvents();
        delay(10);
    }
    flushEvents();
    return fsMounted;
}

static void forceReconnect() {
    reconnectReq = true;
    uint32_t t = millis();
    while (reconnectReq && millis() - t < 1000) {
        delay(1);
    }
}

// List the root directory and read the start of the first regular file
static bool readTest(const char *tag, int n) {
    if (!fsMounted) {
        logf("READ %s %d ok=0 no_fs\n", tag, n);
        return false;
    }
    logf("READ %s %d begin t=%lu\n", tag, n, (unsigned long)millis());
    uint32_t t0 = millis();
    FatFile root, f;
    if (!root.openRoot(&fatfs)) {
        logf("READ %s %d ok=0 openRoot_failed after=%lums\n", tag, n, (unsigned long)(millis() - t0));
        return false;
    }
    char name[64] = "?";
    uint32_t size = 0;
    int got = -1, entries = 0;
    while (f.openNext(&root, O_RDONLY)) {
        entries++;
        if (got < 0 && !f.isDir()) {
            f.getName(name, sizeof(name));
            size = f.fileSize();
            uint8_t buf[256];
            got = f.read(buf, sizeof(buf));
        }
        f.close();
    }
    root.close();
    bool ok = entries > 0 && got >= 0;
    logf("READ %s %d ok=%d entries=%d first=%s size=%lu read=%d after=%lums\n", tag, n, (int)ok, entries, name,
         (unsigned long)size, got, (unsigned long)(millis() - t0));
    return ok;
}

void setup() {
    Serial.begin(115200);
    waitHost(60000);
    logf("HIL380 fix start clk_sys=%lu tusb=%d.%d.%d sched=%s\n", (unsigned long)clock_get_hz(clk_sys), TUSB_VERSION_MAJOR,
         TUSB_VERSION_MINOR, TUSB_VERSION_REVISION, sched);
}

void loop() {
    static uint n = 0;
    waitHost(60000);
    flushEvents();
    if (n >= sizeof(sched) - 1) {
        logf("HIL380 fix DONE\n");
        delay(3000);
        return;
    }
    char m = sched[n];
    if (!waitMounted(60000)) {
        logf("CYCLE %d mode=%c NO_DEVICE\n", n, m);
        n++;
        return;
    }
    readTest("PRE", n);
    logf("ENTER %c %d: light sleep 15 s; swap (power-cycle) the USB-A device now\n", m, n);
    delay(200);
    uint32_t mountsBefore = mscMounts;
    int rc = LowPower.sleepFor(15000);
    waitHost(30000);
    logf("WAKE %c %d rc=%d t=%lu\n", m, n, rc, (unsigned long)millis());
    flushEvents();
    if (m == 'R') {
        forceReconnect();
        logf("RECONNECT forced %d t=%lu\n", n, (unsigned long)millis());
        bool back = waitRemount(mountsBefore, 20000);
        logf("REMOUNT %d ok=%d t=%lu\n", n, (int)back, (unsigned long)millis());
    }
    bool ok = readTest("POST", n);
    if (!ok && m == 'L') {
        logf("RECOVER %d: read failed, forcing reconnect t=%lu\n", n, (unsigned long)millis());
        forceReconnect();
        bool back = waitRemount(mountsBefore, 20000);
        logf("REMOUNT %d ok=%d t=%lu\n", n, (int)back, (unsigned long)millis());
        readTest("RETRY", n);
    }
    flushEvents();
    n++;
}
