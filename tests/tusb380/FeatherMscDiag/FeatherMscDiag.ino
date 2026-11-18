// HIL DIAG (Feather RP2040 USB Host, usbstack=tinyusb, freq=120): native USB device CDC on USB-C for logs,
// Pico-PIO-USB host on the USB-A port (core1) with a USB SD-card reader (MSC + SdFat).
// Every cycle: dormant, woken by unplugging and re-inserting the reader (rising edge on its pulled-up line), then
//   V = pulse the USB-A 5 V (PIN_5V_EN) off for 300 ms after wake, to force a real disconnect + re-enumeration
//   S = no pulse (expected: the first MSC read never completes until another physical replug)
// Every log line is flushed (Adafruit CDC only sends a partial FIFO from yield()), and GPIO18 state is dumped.
#include "pio_usb.h"
#include "SdFat_Adafruit_Fork.h"
#include <Adafruit_TinyUSB.h>
#include <LowPower.h>
#include <stdarg.h>
#include <hardware/clocks.h>
#include <hardware/gpio.h>
#include <hardware/structs/io_bank0.h>
#include <hardware/structs/padsbank0.h>
#include <hardware/structs/sio.h>
#include <hardware/sync.h>

static const char sched[] = "VVS";

Adafruit_USBH_Host USBHost;
Adafruit_USBH_MSC_BlockDevice msc_block_dev;
FatVolume fatfs;
static volatile bool fsMounted = false;
static volatile uint32_t mscMounts = 0;

struct Ev {
    uint32_t ms;
    char kind;
    uint8_t addr;
};
static Ev evs[32];
static volatile uint32_t evHead = 0; // written on core1 (or core0 inside wait_for_io), read on core0
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

void setup1() {
    pinMode(PIN_5V_EN, OUTPUT);
    digitalWrite(PIN_5V_EN, HIGH);
    pio_usb_configuration_t cfg = PIO_USB_DEFAULT_CONFIG;
    cfg.pin_dp = PIN_USB_HOST_DP;
    USBHost.configure_pio_usb(1, &cfg);
    USBHost.begin(1);
}

void loop1() {
    USBHost.task();
}

// printf to CDC and push it out now
static void logf(const char *fmt, ...) {
    char buf[200];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    Serial.print(buf);
    Serial.flush();
}

static int claimedSpinLocks() {
    int c = 0;
    for (uint i = 0; i < NUM_SPIN_LOCKS; i++) {
        c += spin_lock_is_claimed(i) ? 1 : 0;
    }
    return c;
}

static bool rawPad(uint p) {
    return (io_bank0_hw->io[p].status & IO_BANK0_GPIO0_STATUS_INFROMPAD_BITS) != 0;
}

static void dumpVbus(const char *tag, int n) {
    logf("%s %d 5V_EN out=%d oe=%d ctrl=%08lx pad=%08lx raw dp=%d dm=%d\n", tag, n, (int)gpio_get_out_level(PIN_5V_EN),
         (int)((sio_hw->gpio_oe >> PIN_5V_EN) & 1), (unsigned long)io_bank0_hw->io[PIN_5V_EN].ctrl,
         (unsigned long)pads_bank0_hw->io[PIN_5V_EN], (int)rawPad(PIN_USB_HOST_DP), (int)rawPad(PIN_USB_HOST_DM));
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

// List the root directory and read the start of the first regular file
static bool readTest(const char *tag, int n) {
    if (!fsMounted) {
        logf("READ %s %d ok=0 no_fs\n", tag, n);
        return false;
    }
    FatFile root, f;
    if (!root.openRoot(&fatfs)) {
        logf("READ %s %d ok=0 openRoot_failed\n", tag, n);
        return false;
    }
    char name[64] = "?";
    uint32_t size = 0, sum = 0;
    int got = -1, entries = 0;
    while (f.openNext(&root, O_RDONLY)) {
        entries++;
        if (got < 0 && !f.isDir()) {
            f.getName(name, sizeof(name));
            size = f.fileSize();
            uint8_t buf[256];
            got = f.read(buf, sizeof(buf));
            for (int i = 0; i < got; i++) {
                sum += buf[i];
            }
        }
        f.close();
    }
    root.close();
    logf("READ %s %d ok=%d entries=%d first=%s size=%lu read=%d sum=%lu t=%lu\n", tag, n, (int)(got >= 0), entries, name,
         (unsigned long)size, got, (unsigned long)sum, (unsigned long)millis());
    return got >= 0;
}

void setup() {
    Serial.begin(115200);
    waitHost(60000);
    logf("HIL380 diag start clk_sys=%lu tusb=%d.%d.%d sched=%s\n", (unsigned long)clock_get_hz(clk_sys), TUSB_VERSION_MAJOR,
         TUSB_VERSION_MINOR, TUSB_VERSION_REVISION, sched);
}

void loop() {
    static uint n = 0;
    waitHost(60000);
    flushEvents();
    if (n >= sizeof(sched) - 1) {
        logf("HIL380 diag DONE\n");
        delay(3000);
        return;
    }
    char m = sched[n];
    uint32_t t = millis();
    while (!fsMounted && millis() - t < 60000) {
        flushEvents();
        delay(10);
    }
    flushEvents();
    readTest("PRE", n);
    bool dp = rawPad(PIN_USB_HOST_DP), dm = rawPad(PIN_USB_HOST_DM);
    int replugPin = dp ? PIN_USB_HOST_DP : (dm ? PIN_USB_HOST_DM : -1);
    logf("CYCLE %d mode=%c raw dp=%d dm=%d spinlocks=%d\n", n, m, dp, dm, claimedSpinLocks());
    dumpVbus("VBUS-PRE", n);
    logf("ENTER %c %d: unplug and re-insert the reader to wake (wake pin %d)\n", m, n, replugPin);
    delay(300);
    uint32_t mountsBefore = mscMounts;
    LowPower.keepPins((1ULL << PIN_USB_HOST_DP) | (1ULL << PIN_USB_HOST_DM) | (1ULL << PIN_5V_EN));
    int rc;
    if (replugPin < 0) {
        rc = -999;
    } else {
        // Pico-PIO-USB inverts the D+/D- inputs (INOVER); the dormant wake detector must see the real level
        uint32_t cDp = io_bank0_hw->io[PIN_USB_HOST_DP].ctrl, cDm = io_bank0_hw->io[PIN_USB_HOST_DM].ctrl;
        gpio_set_inover(PIN_USB_HOST_DP, GPIO_OVERRIDE_NORMAL);
        gpio_set_inover(PIN_USB_HOST_DM, GPIO_OVERRIDE_NORMAL);
        rc = LowPower.dormantUntilPin(replugPin, RISING);
        io_bank0_hw->io[PIN_USB_HOST_DP].ctrl = cDp;
        io_bank0_hw->io[PIN_USB_HOST_DM].ctrl = cDm;
    }
    uint32_t tWake = millis();
    uint32_t cdcMs = waitHost(30000);
    logf("WAKE %c %d rc=%d clk_sys=%lu cdc_back_after=%lums spinlocks=%d t=%lu\n", m, n, rc, (unsigned long)clock_get_hz(clk_sys),
         (unsigned long)cdcMs, claimedSpinLocks(), (unsigned long)millis());
    dumpVbus("VBUS-WAKE", n);
    if (m == 'V') {
        digitalWrite(PIN_5V_EN, LOW);
        delay(300);
        digitalWrite(PIN_5V_EN, HIGH);
        logf("VBUS pulsed %d t=%lu\n", n, (unsigned long)millis());
        dumpVbus("VBUS-AFTER", n);
    }
    t = millis();
    while (mscMounts == mountsBefore && millis() - t < 20000) {
        delay(10);
    }
    flushEvents();
    logf("POSTWAKE %c %d msc_remounted=%d fs_mounted=%d after=%lums t=%lu\n", m, n, (int)(mscMounts != mountsBefore), (int)fsMounted,
         (unsigned long)(millis() - tWake), (unsigned long)millis());
    dumpVbus("VBUS-POST", n);
    logf("READ POST %d begin t=%lu\n", n, (unsigned long)millis());
    readTest("POST", n);
    flushEvents();
    n++;
}
