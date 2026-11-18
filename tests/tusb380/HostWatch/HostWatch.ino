// HIL WATCH (Feather RP2040 USB Host, usbstack=tinyusb, freq=120): never sleeps.  Logs every USB-A host
// mount/unmount event, the raw D+/D- pad levels when they change, and reads the MSC card every 5 s.
// Used to check a device (e.g. a CircuitPython Feather on the USB-A port) is stable before sleep testing.
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

// Configuration descriptor header of the last mounted device: wTotalLength shows whether a composite
// device's descriptor fits the host enumeration buffer (CFG_TUH_ENUMERATION_BUFSIZE)
static uint8_t cfgHdr[9];
static volatile uint16_t cfgTotal = 0;
static volatile uint8_t cfgItf = 0;

static void cfgHdrCb(tuh_xfer_t *xfer) {
    if (xfer->result == XFER_RESULT_SUCCESS) {
        cfgTotal = (uint16_t)(cfgHdr[2] | (cfgHdr[3] << 8));
        cfgItf = cfgHdr[4];
        addEv('C', xfer->daddr);
    } else {
        addEv('c', xfer->daddr);
    }
}

extern "C" {
    void tuh_mount_cb(uint8_t addr) {
        addEv('M', addr);
        tuh_descriptor_get_configuration(addr, 0, cfgHdr, sizeof(cfgHdr), cfgHdrCb, 0);
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
        if (e.kind == 'C') {
            logf("EV C t=%lu addr=%u config wTotalLength=%u interfaces=%u enum_bufsize=%u\n", (unsigned long)e.ms, e.addr,
                 (unsigned)cfgTotal, (unsigned)cfgItf, (unsigned)CFG_TUH_ENUMERATION_BUFSIZE);
        } else {
            logf("EV %c t=%lu addr=%u\n", e.kind, (unsigned long)e.ms, e.addr);
        }
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
    waitHost(30000);
    logf("HIL380 watch start clk_sys=%lu tusb=%d.%d.%d\n", (unsigned long)clock_get_hz(clk_sys), TUSB_VERSION_MAJOR,
         TUSB_VERSION_MINOR, TUSB_VERSION_REVISION);
}

void loop() {
    static uint32_t lastRead = 0, lastBeat = 0;
    static int lastDp = -1, lastDm = -1;
    static int n = 0;
    flushEvents();
    int dp = rawPad(PIN_USB_HOST_DP), dm = rawPad(PIN_USB_HOST_DM);
    if (dp != lastDp || dm != lastDm) {
        logf("LINE t=%lu raw dp=%d dm=%d\n", (unsigned long)millis(), dp, dm);
        lastDp = dp;
        lastDm = dm;
    }
    if (millis() - lastRead >= 5000) {
        lastRead = millis();
        if (fsMounted) {
            readTest("WATCH", n++);
        }
    }
    if (millis() - lastBeat >= 30000) {
        lastBeat = millis();
        logf("BEAT t=%lu fs_mounted=%d mounts=%lu spinlocks=%d\n", (unsigned long)millis(), (int)fsMounted,
             (unsigned long)mscMounts, claimedSpinLocks());
    }
    delay(20);
}
