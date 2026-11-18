// HIL: device-vs-host discriminator for the re-enumeration failures seen after LowPower sleep (Feather RP2040 USB Host,
// usbstack=tinyusb, freq=120, built with the TinyUSB host log routed through hil_log.h like FeatherMscFixDbg).
// The host never sleeps.  USB-A device: a CircuitPython Feather ESP32-S2 whose EN the bench power-cycles on "ENTER".
//   A (10x): device healthy and mounted, no power cycle; force a reconnect (bus reset, re-enumerate) and read.
//            STALLs here = the device mishandles a plain bus reset.
//   B (5x):  "ENTER": the bench power-cycles the device while the host keeps SOFing; natural detach/attach.
//   C (5x):  silence the root port (no SOF, port stays "connected", as when LowPower idles core1), "ENTER": the bench
//            power-cycles the device, wait 15 s, force a reconnect.  Same as the sleep runs minus LowPower itself
//            (no pin parking, clock changes or core1 idle): failing like them points at the device booting without
//            SOF; clean points back at LowPower.
#include "pio_usb.h"
#include "SdFat_Adafruit_Fork.h"
#include <Adafruit_TinyUSB.h>
#include <stdarg.h>
#include <hardware/clocks.h>
#include <hardware/sync.h>

static const int N_A = 10, N_B = 5, N_C = 5;

Adafruit_USBH_Host USBHost;
Adafruit_USBH_MSC_BlockDevice msc_block_dev;
FatVolume fatfs;
static volatile bool fsMounted = false;
static volatile uint32_t mscMounts = 0;
static volatile uint32_t devMounts = 0;
static volatile uint8_t core1Cmd = 0; // 1 = force reconnect, 2 = silence root port

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

extern "C" void pio_usb_force_reconnect(uint8_t root_idx);
extern "C" void pio_usb_root_silence(uint8_t root_idx);
extern "C" void pio_usb_root_state(uint8_t root_idx, uint8_t out[5]);

void setup1() {
    pinMode(PIN_5V_EN, OUTPUT);
    digitalWrite(PIN_5V_EN, HIGH);
    pio_usb_configuration_t cfg = PIO_USB_DEFAULT_CONFIG;
    cfg.pin_dp = PIN_USB_HOST_DP;
    USBHost.configure_pio_usb(1, &cfg);
    USBHost.begin(1);
}

void loop1() {
    if (core1Cmd) {
        if (core1Cmd == 1) {
            pio_usb_force_reconnect(0);
        } else if (core1Cmd == 2) {
            pio_usb_root_silence(0);
        }
        core1Cmd = 0;
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

// delay() that keeps draining the TinyUSB log ring
static void idle(uint32_t ms) {
    uint32_t t = millis();
    while (millis() - t < ms) {
        drainLog();
        delay(5);
    }
}

static void dumpRoot(const char *tag) {
    uint8_t r[5];
    pio_usb_root_state(0, r);
    logf("ROOT %s t=%lu connected=%u suspended=%u line=%u msc=%lu dev=%lu logdrop=%lu\n", tag, (unsigned long)millis(),
         r[1], r[2], r[4], (unsigned long)mscMounts, (unsigned long)devMounts, (unsigned long)hilLog.dropped());
}

static void core1Do(uint8_t cmd) {
    core1Cmd = cmd;
    uint32_t t = millis();
    while (core1Cmd && millis() - t < 2000) {
        drainLog();
        delay(1);
    }
    logf("CORE1 cmd=%u took=%lums\n", cmd, (unsigned long)(millis() - t));
}

// Wait for a new device mount (tuh_mount_cb) with the filesystem up
static bool waitRemount(uint32_t mscBefore, uint32_t devBefore, uint32_t ms) {
    uint32_t t = millis();
    while (!(fsMounted && mscMounts != mscBefore && devMounts != devBefore) && millis() - t < ms) {
        idle(10);
    }
    drainLog();
    return fsMounted && mscMounts != mscBefore && devMounts != devBefore;
}

static bool waitMounted(uint32_t ms) {
    uint32_t t = millis();
    while (!(fsMounted && devMounts > 0 && tuh_mounted(1)) && millis() - t < ms) {
        idle(10);
    }
    return fsMounted;
}

// Forced reconnects (up to 3) until a new full mount; returns the attempt that worked, 0 if none
static int reconnect(const char *ph, int i) {
    for (int a = 1; a <= 3; a++) {
        uint32_t mscBefore = mscMounts, devBefore = devMounts;
        uint32_t t0 = millis();
        logf("RECONNECT %s %d attempt=%d t=%lu\n", ph, i, a, (unsigned long)t0);
        core1Do(1);
        bool back = waitRemount(mscBefore, devBefore, 15000);
        logf("REMOUNT %s %d attempt=%d ok=%d after=%lums\n", ph, i, a, (int)back, (unsigned long)(millis() - t0));
        if (back) {
            return a;
        }
        dumpRoot("fail");
        idle(2000);
    }
    return 0;
}

static bool readTest(const char *ph, int i) {
    if (!fsMounted) {
        logf("READ %s %d ok=0 no_fs\n", ph, i);
        return false;
    }
    uint32_t t0 = millis();
    FatFile root, f;
    if (!root.openRoot(&fatfs)) {
        logf("READ %s %d ok=0 openRoot_failed\n", ph, i);
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
    logf("READ %s %d ok=%d entries=%d after=%lums\n", ph, i, (int)ok, entries, (unsigned long)(millis() - t0));
    return ok;
}

static void ensureMounted(const char *ph, int i) {
    if (!waitMounted(20000)) {
        logf("NODEV %s %d\n", ph, i);
        dumpRoot("nodev");
        reconnect("fix", i);
    }
}

void setup() {
    Serial.begin(115200);
    uint32_t t = millis();
    while (!(TinyUSBDevice.mounted() && Serial) && millis() - t < 60000) {
        idle(10);
    }
    logf("HIL380 reenumAB start clk_sys=%lu debug=%d A=%d B=%d C=%d\n", (unsigned long)clock_get_hz(clk_sys),
         CFG_TUSB_DEBUG, N_A, N_B, N_C);
}

void loop() {
    static int phase = 0, i = 0;
    const char *ph = phase == 0 ? "A" : phase == 1 ? "B" : "C";
    int nmax = phase == 0 ? N_A : phase == 1 ? N_B : N_C;
    if (phase > 2) {
        logf("HIL380 reenumAB fix DONE\n"); // "fix DONE" ends the bench logger/orchestrator
        idle(3000);
        return;
    }
    if (i >= nmax) {
        phase++;
        i = 0;
        return;
    }
    ensureMounted(ph, i);
    readTest(ph, i);
    if (phase == 0) {
        int a = reconnect("A", i);
        logf("RESULT A %d first_try=%d attempt=%d\n", i, (int)(a == 1), a);
    } else if (phase == 1) {
        uint32_t mscBefore = mscMounts, devBefore = devMounts;
        uint32_t t0 = millis();
        logf("ENTER B %d: power-cycle the device; host stays awake and SOFing\n", i);
        bool back = waitRemount(mscBefore, devBefore, 40000);
        logf("REMOUNT B %d natural ok=%d after=%lums\n", i, (int)back, (unsigned long)(millis() - t0));
        int a = back ? 0 : reconnect("B", i);
        logf("RESULT B %d first_try=%d attempt=%d\n", i, (int)back, back ? 0 : a);
    } else {
        core1Do(2);
        dumpRoot("silenced");
        logf("ENTER C %d: power-cycle the device; root port silenced (no SOF)\n", i);
        idle(15000);
        dumpRoot("pre-reconnect");
        int a = reconnect("C", i);
        logf("RESULT C %d first_try=%d attempt=%d\n", i, (int)(a == 1), a);
    }
    readTest(ph, i);
    idle(1000);
    i++;
}
