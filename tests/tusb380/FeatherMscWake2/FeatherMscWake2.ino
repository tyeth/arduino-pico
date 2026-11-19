// HIL: FeatherMscWake on the library API under test: Adafruit_USBH_Host::recheck() (tuh_rhport_recheck()), from the
// Adafruit_TinyUSB_Arduino branch host-sleep-recheck (tests/tusb380/patches/adafruit-host-sleep-recheck.mbox).
// Feather RP2040 USB Host, usbstack=tinyusb, freq=120; the TinyUSB host log goes through hil_log.h like FeatherMscFixDbg.
//
// After every wake core1 calls USBHost.recheck(). The outcome is read from the mount callbacks: an unmount within
// 3 s means the library re-enumerated the port ("reenumerate"), none means it kept the device ("same"); the debug
// log also carries the library's own "Re-check:" lines.
//
// Schedule: S = the bench power-cycles the USB-A device (CircuitPython Feather ESP32-S2, EN via Jumperless) during
// the 15 s light sleep ("ENTER" line); N = the device is left alone ("NOSWAP" line).
#include "hil_log.h"
#include "pio_usb.h"
#include "SdFat_Adafruit_Fork.h"
#include <Adafruit_TinyUSB.h>
#include <LowPower.h>
#include <stdarg.h>
#include <hardware/clocks.h>
#include <hardware/sync.h>

static const char sched[] = "SSNSSNSSNSSNSSN";

Adafruit_USBH_Host USBHost;
Adafruit_USBH_MSC_BlockDevice msc_block_dev;
FatVolume fatfs;
static volatile bool fsMounted = false;
static volatile uint32_t mscMounts = 0;
static volatile uint32_t devMounts = 0;
static volatile uint32_t devUnmounts = 0;

HilLog hilLog;
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

extern "C" {
    void tuh_mount_cb(uint8_t addr) {
        (void)addr;
        devMounts = devMounts + 1;
    }
    void tuh_umount_cb(uint8_t addr) {
        (void)addr;
        devUnmounts = devUnmounts + 1;
    }
    void tuh_msc_mount_cb(uint8_t addr) {
        msc_block_dev.begin(addr);
        msc_block_dev.setActiveLUN(0);
        fsMounted = fatfs.begin(&msc_block_dev);
        mscMounts = mscMounts + 1;
    }
    void tuh_msc_umount_cb(uint8_t addr) {
        (void)addr;
        fsMounted = false;
        fatfs.end();
        msc_block_dev.end();
    }
}

//--------------------------------------------------------------------+
// Wake check: the library call, from core1 (the core that runs USBHost.task())
//--------------------------------------------------------------------+
static volatile bool recheckReq = false;
static volatile bool recheckQueued = false;

void setup1() {
    pinMode(PIN_5V_EN, OUTPUT);
    digitalWrite(PIN_5V_EN, HIGH);
    pio_usb_configuration_t cfg = PIO_USB_DEFAULT_CONFIG;
    cfg.pin_dp = PIN_USB_HOST_DP;
    USBHost.configure_pio_usb(1, &cfg);
    USBHost.begin(1);
}

void loop1() {
    if (recheckReq) {
        recheckQueued = USBHost.recheck();
        recheckReq = false;
    }
    USBHost.task();
}

//--------------------------------------------------------------------+
// core0: logging, schedule, reads
//--------------------------------------------------------------------+
static void plog(const char *fmt, ...) {
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

static void idle(uint32_t ms) {
    uint32_t t = millis();
    while (millis() - t < ms) {
        drainLog();
        delay(5);
    }
}

static bool waitMounted(uint32_t ms) {
    uint32_t t = millis();
    while (!(fsMounted && devMounts > 0) && millis() - t < ms) {
        idle(10);
    }
    return fsMounted;
}

static bool readTest(const char *tag, int i) {
    if (!fsMounted) {
        plog("READ %s %d ok=0 no_fs\n", tag, i);
        return false;
    }
    uint32_t t0 = millis();
    FatFile root, f;
    if (!root.openRoot(&fatfs)) {
        plog("READ %s %d ok=0 openRoot_failed after=%lums\n", tag, i, (unsigned long)(millis() - t0));
        return false;
    }
    int got = -1, entries = 0;
    while (f.openNext(&root, O_RDONLY)) {
        entries++;
        if (got < 0 && !f.isDir()) {
            uint8_t buf[256];
            got = f.read(buf, sizeof(buf));
        }
        f.close();
    }
    root.close();
    bool ok = entries > 0 && got >= 0;
    plog("READ %s %d ok=%d entries=%d after=%lums\n", tag, i, (int)ok, entries, (unsigned long)(millis() - t0));
    return ok;
}


void setup() {
    Serial.begin(115200);
    uint32_t t = millis();
    while (!(TinyUSBDevice.mounted() && Serial) && millis() - t < 60000) {
        idle(10);
    }
    plog("HIL380 mscwake start clk_sys=%lu debug=%d sched=%s\n", (unsigned long)clock_get_hz(clk_sys), CFG_TUSB_DEBUG,
         sched);
}

void loop() {
    static unsigned n = 0;
    if (n >= sizeof(sched) - 1) {
        plog("HIL380 mscwake fix DONE\n"); // "fix DONE" ends the bench logger/orchestrator
        idle(3000);
        return;
    }
    char m = sched[n];
    if (!waitMounted(30000)) {
        plog("CYCLE %u NO_DEVICE before sleep\n", n);
    }
    readTest("PRE", n);
    if (m == 'S') {
        plog("ENTER S %u: light sleep 15 s; swap (power-cycle) the USB-A device now\n", n);
    } else {
        plog("NOSWAP %u: light sleep 15 s, device left alone\n", n);
    }
    delay(200);
    uint32_t mscBefore = mscMounts, devBefore = devMounts, umBefore = devUnmounts;
    LowPower.sleepFor(15000);
    uint32_t tw = millis();

    recheckQueued = false;
    recheckReq = true;
    while (recheckReq && millis() - tw < 1000) {
        idle(1);
    }
    // an unmount within 3 s = the library found the device gone or replaced and is re-enumerating the port
    while (devUnmounts == umBefore && millis() - tw < 3000) {
        idle(5);
    }
    bool reenum = devUnmounts != umBefore;
    uint32_t tCheck = millis() - tw;
    bool remounted = false;
    if (reenum || !fsMounted) {
        while (!(fsMounted && mscMounts != mscBefore && devMounts != devBefore) && millis() - tw < 20000) {
            idle(10);
        }
        remounted = fsMounted && mscMounts != mscBefore && devMounts != devBefore;
    }
    bool ok = readTest("POST", n);
    plog("RESULT %u mode=%c wake=%s queued=%d check=%lums remounted=%d total=%lums read=%d\n", n, m,
         reenum ? "reenumerate" : "same", (int)recheckQueued, (unsigned long)tCheck, (int)remounted,
         (unsigned long)(millis() - tw), (int)ok);
    idle(500);
    n++;
}
