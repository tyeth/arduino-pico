// HIL: the msc_file_explorer_sleep example (Adafruit_TinyUSB_Arduino branch host-sleep-deinit), unchanged in what it
// does, plus the TinyUSB host log (hil_log.h ring, drained by core0 while it waits) and a record of where core1 is
// (core1Stage), printed every 5 s while core0 waits. Repro for a hang seen after a wake ~1.6 s after an S2 swap:
// core0 waited forever in ask_core1(). Feather RP2040 USB Host, usbstack=tinyusb, freq=120.
// The rig swaps the S2 on each "Sleeping 10 s" line (pre_s=1: the swap ends ~1.6 s before the wake).
#include "hil_log.h"
#include "pio_usb.h"
#include "SdFat_Adafruit_Fork.h"
#include <Adafruit_TinyUSB.h>
#include <LowPower.h>
#include <stdarg.h>
#include <hardware/sync.h>

static const unsigned CYCLES = 12;

Adafruit_USBH_Host USBHost;
Adafruit_USBH_MSC_BlockDevice msc_block_dev;
FatVolume fatfs;
volatile bool is_mounted = false;

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

// core1 logs through the ring too, so its lines stay in order with TinyUSB's
static void c1log(const char *fmt, ...) {
    char buf[200];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    hilLog.write(buf);
}

// where core1 is, for the heartbeat
enum {
    ST_IDLE, ST_TASK, ST_LIST, ST_END, ST_BEGIN, ST_MOUNT_CB, ST_FAT_BEGIN, ST_UMOUNT_CB, ST_FAT_END
};
static const char *const stName[] = {"idle", "task", "list", "end", "begin", "mount_cb", "fat_begin", "umount_cb",
                                     "fat_end"
                                    };
static volatile int core1Stage = ST_IDLE;
static volatile uint32_t core1Loops = 0;

enum { REQ_NONE, REQ_LIST, REQ_STOP, REQ_START };
volatile int core1_request = REQ_NONE;

static void drainLog() {
    static char chunk[256];
    size_t n;
    while ((n = hilLog.drain(chunk, sizeof(chunk) - 1)) > 0) {
        chunk[n] = 0;
        Serial.print(chunk);
    }
    Serial.flush();
}

static void ask_core1(int request) {
    core1_request = request;
    uint32_t t0 = millis(), beat = t0;
    while (core1_request != REQ_NONE) {
        drainLog();
        delay(5);
        if (millis() - beat >= 5000) {
            beat = millis();
            Serial.printf("WAIT req=%d for %lus core1=%s loops=%lu mounted=%d dropped=%lu\n", request,
                          (unsigned long)((millis() - t0) / 1000), stName[core1Stage], (unsigned long)core1Loops,
                          (int)is_mounted, (unsigned long)hilLog.dropped());
        }
    }
    drainLog();
}

void setup() {
    Serial.begin(115200);
    uint32_t t = millis();
    while (!(TinyUSBDevice.mounted() && Serial) && millis() - t < 60000) {
        drainLog();
        delay(10);
    }
    Serial.println("HIL380 example-dbg start: TinyUSB Host Mass Storage: sleep example");
}

void loop() {
    static unsigned n = 0;
    if (n >= CYCLES) {
        Serial.println("HIL380 example-dbg fix DONE");
        drainLog();
        delay(3000);
        return;
    }
    for (uint32_t t0 = millis(); !is_mounted && millis() - t0 < 10000;) {
        drainLog();
        delay(10);
    }
    Serial.printf("CYCLE %u mounted=%d\n", n, (int)is_mounted);
    ask_core1(REQ_LIST);
    ask_core1(REQ_STOP);

    Serial.println("Sleeping 10 s: unplug, swap or replug the drive now if you like");
    Serial.flush();
    LowPower.sleepFor(10000);
    Serial.println("Awake");

    ask_core1(REQ_START);
    n++;
}

//------------- Core1 -------------//
static bool host_running = false;

void setup1() {
    pinMode(PIN_5V_EN, OUTPUT);
    digitalWrite(PIN_5V_EN, HIGH);
    pio_usb_configuration_t cfg = PIO_USB_DEFAULT_CONFIG;
    cfg.pin_dp = PIN_USB_HOST_DP;
    USBHost.configure_pio_usb(1, &cfg);
    host_running = USBHost.begin(1);
}

void loop1() {
    core1Loops = core1Loops + 1;
    if (host_running) {
        core1Stage = ST_TASK;
        USBHost.task();
        core1Stage = ST_IDLE;
    }

    switch (core1_request) {
        case REQ_LIST:
            core1Stage = ST_LIST;
            if (is_mounted) {
                fatfs.ls(&Serial, LS_SIZE);
            } else {
                Serial.println("No drive mounted");
            }
            break;

        case REQ_STOP:
            core1Stage = ST_END;
            c1log("core1: end() = %d\r\n", (int)USBHost.end());
            host_running = false;
            break;

        case REQ_START:
            core1Stage = ST_BEGIN;
            host_running = USBHost.begin(1);
            c1log("core1: begin() = %d\r\n", (int)host_running);
            break;

        default:
            return;
    }
    core1Stage = ST_IDLE;
    core1_request = REQ_NONE;
}

//--------------------------------------------------------------------+
// TinyUSB Host callbacks (core1)
//--------------------------------------------------------------------+
extern "C" {

    void tuh_msc_mount_cb(uint8_t dev_addr) {
        int prev = core1Stage;
        core1Stage = ST_MOUNT_CB;
        c1log("core1: Drive attached, address = %d\r\n", dev_addr);
        msc_block_dev.begin(dev_addr);
        msc_block_dev.setActiveLUN(0);
        core1Stage = ST_FAT_BEGIN;
        is_mounted = fatfs.begin(&msc_block_dev);
        c1log("core1: fatfs.begin() = %d\r\n", (int)is_mounted);
        core1Stage = prev;
    }

    void tuh_msc_umount_cb(uint8_t dev_addr) {
        int prev = core1Stage;
        core1Stage = ST_UMOUNT_CB;
        c1log("core1: Drive removed, address = %d\r\n", dev_addr);
        is_mounted = false;
        core1Stage = ST_FAT_END;
        fatfs.end();
        msc_block_dev.end();
        core1Stage = prev;
    }
}
