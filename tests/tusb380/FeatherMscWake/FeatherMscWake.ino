// HIL: re-detect a USB device swapped during LowPower sleep using only the TinyUSB host API (no Pico-PIO-USB change).
// Feather RP2040 USB Host, usbstack=tinyusb, freq=120; the TinyUSB host log goes through hil_log.h like FeatherMscFixDbg.
//
// LowPower idles core1, so Pico-PIO-USB cannot see a detach during sleep: a device swapped (or reset) while asleep
// leaves TinyUSB holding the old one at its old address.  After every wake, core1 runs a "wake check":
//   - every device mounted directly on a root port gets a GET_DESCRIPTOR(device);
//   - one that answers is the same device: nothing to do (no needless remount);
//   - one that doesn't (a new/reset device sits at address 0) gets the root port re-enumerated by posting an attach
//     event, hcd_event_device_attach(): usbh tears the stale device down first, resets the port and enumerates.
// A device that was only unplugged is caught by Pico-PIO-USB itself once SOFs resume.
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
#include "host/hcd.h" // hcd_event_device_attach()

static const char sched[] = "SSNSSNSSNSSNSSN";

Adafruit_USBH_Host USBHost;
Adafruit_USBH_MSC_BlockDevice msc_block_dev;
FatVolume fatfs;
static volatile bool fsMounted = false;
static volatile uint32_t mscMounts = 0;
static volatile uint32_t devMounts = 0;

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
// Wake check (core1: the core that runs the host task)
//--------------------------------------------------------------------+
enum { WAKE_IDLE, WAKE_REQUESTED, WAKE_PROBING, WAKE_SAME, WAKE_REENUM, WAKE_NODEV };
static volatile uint8_t wakeState = WAKE_IDLE;
static volatile uint8_t wakeAddr = 0;
static uint8_t probeRhport;
static uint8_t probeBuf[18];

static void reenumerate(uint8_t rhport) {
    // Same event the stack posts for a new attach (and for its own root port retry).  Posted from core1 with
    // interrupts off: Pico-PIO-USB's SOF interrupt posts events on this core too, and its hcd_int_disable() is empty.
    uint32_t s = save_and_disable_interrupts();
    hcd_event_device_attach(rhport, false);
    restore_interrupts(s);
    wakeState = WAKE_REENUM;
}

static void probe_cb(tuh_xfer_t *xfer) {
    if (xfer->result == XFER_RESULT_SUCCESS) {
        wakeState = WAKE_SAME;
    } else {
        reenumerate(probeRhport);
    }
}

static void wakeCheck() {
    for (uint8_t d = 1; d <= CFG_TUH_DEVICE_MAX; d++) {
        tuh_bus_info_t bi;
        if (tuh_mounted(d) && tuh_bus_info_get(d, &bi) && bi.hub_addr == 0) {
            // TODO more than one root port: probe each
            wakeAddr = d;
            probeRhport = bi.rhport;
            wakeState = WAKE_PROBING;
            if (!tuh_descriptor_get_device(d, probeBuf, sizeof(probeBuf), probe_cb, 0)) {
                reenumerate(bi.rhport);
            }
            return;
        }
    }
    wakeState = WAKE_NODEV;
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
    if (wakeState == WAKE_REQUESTED) {
        wakeCheck();
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

static const char *wakeName(uint8_t s) {
    switch (s) {
    case WAKE_SAME: return "same";
    case WAKE_REENUM: return "reenumerate";
    case WAKE_NODEV: return "nodev";
    case WAKE_PROBING: return "probe_timeout";
    default: return "?";
    }
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
    uint32_t mscBefore = mscMounts, devBefore = devMounts;
    LowPower.sleepFor(15000);
    uint32_t tw = millis();

    wakeState = WAKE_REQUESTED;
    while ((wakeState == WAKE_REQUESTED || wakeState == WAKE_PROBING) && millis() - tw < 3000) {
        idle(1);
    }
    uint8_t ws = wakeState;
    uint32_t tCheck = millis() - tw;
    bool remounted = false;
    if (ws == WAKE_REENUM || ws == WAKE_NODEV) {
        // wait for the device (new, or plugged in while asleep) to mount
        while (!(fsMounted && mscMounts != mscBefore && devMounts != devBefore) && millis() - tw < 20000) {
            idle(10);
        }
        remounted = fsMounted && mscMounts != mscBefore && devMounts != devBefore;
    }
    bool ok = readTest("POST", n);
    plog("RESULT %u mode=%c wake=%s addr=%u check=%lums remounted=%d total=%lums read=%d\n", n, m, wakeName(ws),
         wakeAddr, (unsigned long)tCheck, (int)remounted, (unsigned long)(millis() - tw), (int)ok);
    wakeState = WAKE_IDLE;
    idle(500);
    n++;
}
