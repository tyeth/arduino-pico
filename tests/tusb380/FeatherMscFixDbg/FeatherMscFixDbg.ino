// HIL DEBUG build of FeatherMscFix (Feather RP2040 USB Host, usbstack=tinyusb, freq=120).
// Same stale-device cycles, plus:
//  - the TinyUSB host debug log (CFG_TUSB_DEBUG=2, routed by build flags through hil_log.h into a lock-free ring
//    buffer that core1 fills and core0 drains to USB CDC), so failed TU_ASSERTs during re-enumeration show file:line;
//  - a Pico-PIO-USB root port dump (initialized/connected/suspended/speed/line) while waiting for a remount;
//  - reads only start after the whole device is mounted (tuh_mount_cb), not just the MSC interface.
// Cycles light-sleep 15 s (timer wake; the bench power-cycles the USB-A device via its EN pin meanwhile):
//   L = read straight away; on failure force a reconnect and read again.
//   R = force a reconnect right after waking, then read.
#include "pio_usb.h"
#include "SdFat_Adafruit_Fork.h"
#include <Adafruit_TinyUSB.h>
#include <LowPower.h>
#include <stdarg.h>
#include <hardware/clocks.h>
#include <hardware/sync.h>

static const char sched[] = "LRLRLR";

Adafruit_USBH_Host USBHost;
Adafruit_USBH_MSC_BlockDevice msc_block_dev;
FatVolume fatfs;
static volatile bool fsMounted = false;
static volatile uint32_t mscMounts = 0;
static volatile uint32_t devMounts = 0; // tuh_mount_cb count: every interface of the device configured
static volatile bool reconnectReq = false;

HilLog hilLog;
// Producers: TinyUSB host log on core1 (task + PIO-USB IRQ) and MSC calls from core0, so serialise writers
static spin_lock_t *hilLk = spin_lock_init(spin_lock_claim_unused(true));

size_t HilLog::write(const char *str) {
    size_t n = 0;
    uint32_t s = spin_lock_blocking(hilLk);
    for (; str[n]; n++) {
        uint32_t h = _head, next = (h + 1) % SIZE;
        if (next == _tail) {
            _dropped = _dropped + 1;
            continue;
        }
        _buf[h] = str[n];
        __dmb();
        _head = next;
    }
    spin_unlock(hilLk, s);
    return n;
}

size_t HilLog::drain(char *dst, size_t max) {
    size_t n = 0;
    while (n < max && _tail != _head) {
        dst[n++] = _buf[_tail];
        __dmb();
        _tail = (_tail + 1) % SIZE;
    }
    return n;
}

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
        devMounts = devMounts + 1;
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

// pio_reconnect.c (C, since pio_usb_ll.h is not C++-clean)
extern "C" void pio_usb_force_reconnect(uint8_t root_idx);
extern "C" void pio_usb_root_state(uint8_t root_idx, uint8_t out[5]);

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
    char buf[220];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    Serial.print(buf);
    Serial.flush();
}

static void drainLog() {
    static char chunk[256];
    size_t n;
    while ((n = hilLog.drain(chunk, sizeof(chunk) - 1)) > 0) {
        chunk[n] = 0;
        Serial.print(chunk);
    }
    Serial.flush();
}

static void flushEvents() {
    drainLog();
    while (evPrinted < evHead) {
        const Ev &e = evs[evPrinted % 32];
        logf("EV %c t=%lu addr=%u\n", e.kind, (unsigned long)e.ms, e.addr);
        evPrinted++;
    }
}

static void dumpRoot(const char *tag) {
    uint8_t r[5];
    pio_usb_root_state(0, r);
    logf("ROOT %s t=%lu init=%u connected=%u suspended=%u fs=%u line=%u tuh_mounted1=%d msc=%lu dev=%lu logdrop=%lu\n", tag,
         (unsigned long)millis(), r[0], r[1], r[2], r[3], r[4], (int)tuh_mounted(1), (unsigned long)mscMounts,
         (unsigned long)devMounts, (unsigned long)hilLog.dropped());
}

static uint32_t waitHost(uint32_t ms) {
    uint32_t t = millis();
    while (!(TinyUSBDevice.mounted() && Serial) && millis() - t < ms) {
        drainLog();
        delay(10);
    }
    return millis() - t;
}

// Wait for a new full device mount (tuh_mount_cb) with the filesystem up; dump the root port every 250 ms
static bool waitRemount(uint32_t mscBefore, uint32_t devBefore, uint32_t ms) {
    uint32_t t = millis(), lastDump = 0;
    while (!(fsMounted && mscMounts != mscBefore && devMounts != devBefore) && millis() - t < ms) {
        flushEvents();
        if (millis() - lastDump >= 250) {
            lastDump = millis();
            dumpRoot("wait");
        }
        delay(10);
    }
    flushEvents();
    dumpRoot("end");
    return fsMounted && mscMounts != mscBefore && devMounts != devBefore;
}

// Wait until a device is fully mounted (MSC filesystem + tuh_mount_cb)
static bool waitMounted(uint32_t ms) {
    uint32_t t = millis();
    while (!(fsMounted && devMounts > 0 && tuh_mounted(1)) && millis() - t < ms) {
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
    flushEvents();
    return ok;
}

void setup() {
    Serial.begin(115200);
    waitHost(60000);
    logf("HIL380 fixdbg start clk_sys=%lu tusb=%d.%d.%d debug=%d sched=%s\n", (unsigned long)clock_get_hz(clk_sys),
         TUSB_VERSION_MAJOR, TUSB_VERSION_MINOR, TUSB_VERSION_REVISION, CFG_TUSB_DEBUG, sched);
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
        dumpRoot("nodev");
        n++;
        return;
    }
    readTest("PRE", n);
    logf("ENTER %c %d: light sleep 15 s; swap (power-cycle) the USB-A device now\n", m, n);
    delay(200);
    uint32_t mscBefore = mscMounts, devBefore = devMounts;
    int rc = LowPower.sleepFor(15000);
    waitHost(30000);
    logf("WAKE %c %d rc=%d t=%lu\n", m, n, rc, (unsigned long)millis());
    dumpRoot("wake");
    flushEvents();
    if (m == 'R') {
        forceReconnect();
        logf("RECONNECT forced %d t=%lu\n", n, (unsigned long)millis());
        bool back = waitRemount(mscBefore, devBefore, 20000);
        logf("REMOUNT %d ok=%d t=%lu\n", n, (int)back, (unsigned long)millis());
    }
    bool ok = readTest("POST", n);
    if (!ok && m == 'L') {
        logf("RECOVER %d: read failed, forcing reconnect t=%lu\n", n, (unsigned long)millis());
        dumpRoot("before");
        forceReconnect();
        bool back = waitRemount(mscBefore, devBefore, 20000);
        logf("REMOUNT %d ok=%d t=%lu\n", n, (int)back, (unsigned long)millis());
        readTest("RETRY", n);
    }
    flushEvents();
    n++;
}
