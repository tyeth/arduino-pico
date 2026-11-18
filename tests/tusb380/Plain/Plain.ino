// Link check, no LowPower.  Builds on every usbstack (tinyusb, tinyusb_host, picosdk, nousb).
#if defined(USE_TINYUSB)
#include <Adafruit_TinyUSB.h>
#endif

#if defined(USE_TINYUSB_HOST)
Adafruit_USBH_Host USBHost;
#endif

void setup() {
    Serial1.begin(115200);
#if defined(USE_TINYUSB_HOST)
    USBHost.begin(0);
#endif
    Serial1.println("plain");
}

void loop() {
#if defined(USE_TINYUSB_HOST)
    USBHost.task(0);
#endif
}
