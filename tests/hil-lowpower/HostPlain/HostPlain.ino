// Build check: Adafruit TinyUSB native host (usbstack=tinyusb_host) without LowPower.
// On stock 6.4.0 this fails to link via stdio_flush() -> tud_cdc_write_flush().
#include <Adafruit_TinyUSB.h>

Adafruit_USBH_Host USBHost;

void setup() {
    Serial1.begin(115200);
    USBHost.begin(0);
    Serial1.println("HIL_LP host-plain start");
}

void loop() {
    USBHost.task(0);
}
