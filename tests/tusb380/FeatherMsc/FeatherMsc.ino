// HIL (Feather RP2040 USB Host, usbstack=tinyusb, freq=120): native USB device CDC on USB-C for logs,
// Pico-PIO-USB host on the USB-A port (core1) with a USB SD-card reader (MSC + SdFat), through
// LowPower light sleep and dormant.  Before every sleep the card is mounted and a file is read; after
// every wake the reader must still (or again) be mounted and the card readable.
// Schedule: L = sleepFor(3000); B = dormant until D5 (GPIO5, pulled up here) is pulled low, reader left in;
// R = dormant until the reader is unplugged and re-inserted (rising edge on its pulled-up data line).
#include "pio_usb.h"
#include "SdFat_Adafruit_Fork.h"
#include <Adafruit_TinyUSB.h>
#include <LowPower.h>
#include <hardware/clocks.h>
#include <hardware/gpio.h>
#include <hardware/structs/io_bank0.h>
#include <hardware/sync.h>

static const char sched[] = "LLBBRR";

// Header pin the bench (Jumperless) shorts to GND to wake a B cycle.  GPIO7 is only the BOOT button.
#define WAKE_PIN 5

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
static volatile uint32_t evHead = 0; // written on core1, read on core0
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

static void flushEvents() {
    while (evPrinted < evHead) {
        const Ev &e = evs[evPrinted % 32];
        Serial.printf("EV %c t=%lu addr=%u\n", e.kind, (unsigned long)e.ms, e.addr);
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
        Serial.printf("READ %s %d ok=0 no_fs\n", tag, n);
        return false;
    }
    FatFile root, f;
    if (!root.openRoot(&fatfs)) {
        Serial.printf("READ %s %d ok=0 openRoot_failed\n", tag, n);
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
    Serial.printf("READ %s %d ok=%d entries=%d first=%s size=%lu read=%d sum=%lu\n", tag, n, (int)(got >= 0), entries, name,
                  (unsigned long)size, got, (unsigned long)sum);
    return got >= 0;
}

void setup() {
    Serial.begin(115200);
    pinMode(WAKE_PIN, INPUT_PULLUP);
    waitHost(60000);
    Serial.printf("HIL380 feather start clk_sys=%lu tusb=%d.%d.%d sched=%s\n", (unsigned long)clock_get_hz(clk_sys), TUSB_VERSION_MAJOR,
                  TUSB_VERSION_MINOR, TUSB_VERSION_REVISION, sched);
}

void loop() {
    static uint n = 0;
    waitHost(60000);
    flushEvents();
    if (n >= sizeof(sched) - 1) {
        Serial.println("HIL380 feather DONE");
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
    Serial.printf("CYCLE %d mode=%c raw dp=%d dm=%d spinlocks=%d\n", n, m, dp, dm, claimedSpinLocks());
    if (m == 'L') {
        Serial.printf("ENTER L %d: light sleep 3 s, nothing to do\n", n);
    } else if (m == 'B') {
        Serial.printf("ENTER B %d: leave the reader in; pull D5 (GPIO5) to GND to wake\n", n);
    } else {
        Serial.printf("ENTER R %d: unplug and re-insert the reader to wake (wake pin %d)\n", n, replugPin);
    }
    Serial.flush();
    delay(300);
    uint32_t mountsBefore = mscMounts;
    LowPower.keepPins((1ULL << PIN_USB_HOST_DP) | (1ULL << PIN_USB_HOST_DM) | (1ULL << PIN_5V_EN));
    int rc;
    if (m == 'L') {
        rc = LowPower.sleepFor(3000);
    } else if (m == 'B') {
        rc = LowPower.dormantUntilPin(WAKE_PIN, FALLING);
    } else if (replugPin < 0) {
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
    Serial.printf("WAKE %c %d rc=%d clk_sys=%lu cdc_back_after=%lums spinlocks=%d\n", m, n, rc, (unsigned long)clock_get_hz(clk_sys),
                  (unsigned long)cdcMs, claimedSpinLocks());
    t = millis();
    uint32_t settle = (m == 'R') ? 20000 : 5000;
    while (mscMounts == mountsBefore && millis() - t < settle) {
        delay(10);
    }
    flushEvents();
    Serial.printf("POSTWAKE %c %d msc_remounted=%d fs_mounted=%d after=%lums\n", m, n, (int)(mscMounts != mountsBefore), (int)fsMounted,
                  (unsigned long)(millis() - tWake));
    readTest("POST", n);
    n++;
}
