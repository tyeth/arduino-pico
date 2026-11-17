// HIL: Adafruit TinyUSB native host (usbstack=tinyusb_host) + LowPower dormant.
// Logs on Serial1 (UART0 GP0/GP1, Debug Probe UART).  Checks the sketch survives
// dormant and the native host controller state before and after each wake.
#include <Adafruit_TinyUSB.h>
#include <LowPower.h>
#include <hardware/structs/usb.h>

Adafruit_USBH_Host USBHost;

static void dumpUsb(const char *tag) {
    Serial1.printf("%s tuh_inited=%d main_ctrl=%08lx sie_ctrl=%08lx sie_status=%08lx inte=%08lx\n", tag,
                   (int)tuh_inited(), (unsigned long)usb_hw->main_ctrl, (unsigned long)usb_hw->sie_ctrl,
                   (unsigned long)usb_hw->sie_status, (unsigned long)usb_hw->inte);
}

void setup() {
    Serial1.begin(115200);
    delay(500);
    Serial1.println("HIL_LP host-mode start");
    bool ok = USBHost.begin(0);
    Serial1.printf("USBHost.begin=%d\n", (int)ok);
    dumpUsb("BOOT");
}

void loop() {
    static int n = 0;
    if (n >= 4) {
        Serial1.println("HIL_LP host-mode DONE");
        delay(5000);
        return;
    }
    uint32_t t = millis();
    while (millis() - t < 2000) {
        USBHost.task(0);
    }
    dumpUsb("PRE");
    int rc = LowPower.dormantFor(3000);
    Serial1.printf("WAKE %d rc=%d lastSleepMs=%lu\n", n, rc, (unsigned long)LowPower.lastSleepMs());
    dumpUsb("POST");
    t = millis();
    while (millis() - t < 500) {
        USBHost.task(0);
    }
    Serial1.printf("ALIVE %d\n", n);
    n++;
}
