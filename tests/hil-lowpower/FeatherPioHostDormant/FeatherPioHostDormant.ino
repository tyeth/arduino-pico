// HIL (Feather RP2040 USB Host): the board's default setup, Adafruit TinyUSB device on
// USB-C plus a Pico-PIO-USB host on the USB-A port (core1), through LowPower dormant.
// Dormant wakes when the USB-A device is unplugged and re-inserted: a rising edge on the
// data line its pull-up drives (D+ = GPIO16 full speed, D- = GPIO17 low speed).
// Reports over USB CDC whether CDC comes back and whether the PIO host re-enumerates.
#include "pio_usb.h"
#include <Adafruit_TinyUSB.h>
#include <LowPower.h>
#include <hardware/clocks.h>

Adafruit_USBH_Host USBHost;

struct Ev {
    uint32_t ms;
    char kind;
    uint8_t addr;
    uint16_t vid, pid;
};
static Ev evs[32];
static volatile uint32_t evHead = 0; // written on core1, read on core0
static uint32_t evPrinted = 0;
static volatile uint8_t mountedAddr = 0;
static volatile uint32_t mounts = 0;
static volatile uint16_t mvid = 0, mpid = 0;

static void addEv(char kind, uint8_t addr, uint16_t vid, uint16_t pid) {
    Ev &e = evs[evHead % 32];
    e.ms = millis();
    e.kind = kind;
    e.addr = addr;
    e.vid = vid;
    e.pid = pid;
    evHead = evHead + 1;
}

extern "C" void tuh_mount_cb(uint8_t addr) {
    uint16_t vid = 0, pid = 0;
    tuh_vid_pid_get(addr, &vid, &pid);
    mvid = vid;
    mpid = pid;
    mountedAddr = addr;
    mounts = mounts + 1;
    addEv('M', addr, vid, pid);
}

extern "C" void tuh_umount_cb(uint8_t addr) {
    if (mountedAddr == addr) {
        mountedAddr = 0;
    }
    addEv('U', addr, 0, 0);
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

static void flushEvents() {
    while (evPrinted < evHead) {
        const Ev &e = evs[evPrinted % 32];
        Serial.printf("EV %c t=%lu addr=%u %04x:%04x\n", e.kind, (unsigned long)e.ms, e.addr, e.vid, e.pid);
        evPrinted++;
    }
}

// Wait until the PC has the CDC port open again (mounted + DTR)
static uint32_t waitHost(uint32_t ms) {
    uint32_t t = millis();
    while (!(TinyUSBDevice.mounted() && Serial) && millis() - t < ms) {
        delay(10);
    }
    return millis() - t;
}

void setup() {
    Serial.begin(115200);
    waitHost(60000);
    Serial.printf("HIL_LP feather start clk_sys=%lu\n", (unsigned long)clock_get_hz(clk_sys));
}

void loop() {
    static int n = 0;
    waitHost(60000);
    flushEvents();
    if (n >= 3) {
        Serial.println("HIL_LP feather DONE");
        delay(3000);
        return;
    }
    uint32_t t = millis();
    while (!mountedAddr && millis() - t < 60000) {
        flushEvents();
        delay(10);
    }
    flushEvents();
    if (!mountedAddr) {
        Serial.println("NO_DEVICE: plug a USB device into the USB-A port");
        return;
    }
    delay(500);
    bool dp = gpio_get(PIN_USB_HOST_DP), dm = gpio_get(PIN_USB_HOST_DM);
    int pin = dp ? PIN_USB_HOST_DP : (dm ? PIN_USB_HOST_DM : -1);
    Serial.printf("CYCLE %d device %04x:%04x addr=%u dp=%d dm=%d wake_pin=%d\n", n, mvid, mpid, mountedAddr, dp, dm, pin);
    if (pin < 0) {
        Serial.println("NO_PULLUP: neither data line is high");
        delay(2000);
        return;
    }
    Serial.printf("ENTER_DORMANT %d: unplug and re-insert the USB-A device to wake\n", n);
    Serial.flush();
    delay(300);
    uint32_t mountsBefore = mounts;
    LowPower.keepPins((1ULL << PIN_USB_HOST_DP) | (1ULL << PIN_USB_HOST_DM) | (1ULL << PIN_5V_EN));
    int rc = LowPower.dormantUntilPin(pin, RISING);
    uint32_t tWake = millis();
    uint32_t cdcMs = waitHost(30000);
    Serial.printf("WAKE %d rc=%d clk_sys=%lu cdc_back_after=%lums cdc_ok=%d\n", n, rc, (unsigned long)clock_get_hz(clk_sys),
                  (unsigned long)cdcMs, (int)(TinyUSBDevice.mounted() && Serial));
    t = millis();
    while (mounts == mountsBefore && millis() - t < 15000) {
        delay(10);
    }
    flushEvents();
    Serial.printf("POSTWAKE %d host_remounted=%d device %04x:%04x after=%lums\n", n, (int)(mounts != mountsBefore), mvid, mpid,
                  (unsigned long)(millis() - tWake));
    n++;
}
