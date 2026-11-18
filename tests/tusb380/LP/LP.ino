// Link check with LowPower: sleepFor, sleepUntilPin, dormantUntilPin (+ dormantFor on RP2350).
// Builds on every usbstack (tinyusb, tinyusb_host, picosdk, nousb).
#if defined(USE_TINYUSB)
#include <Adafruit_TinyUSB.h>
#endif
#include <LowPower.h>

#if defined(USE_TINYUSB_HOST)
Adafruit_USBH_Host USBHost;
#endif

void setup() {
    Serial1.begin(115200);
#if defined(USE_TINYUSB_HOST)
    USBHost.begin(0);
#endif
}

void loop() {
#if defined(USE_TINYUSB_HOST)
    USBHost.task(0);
#endif
    int a = LowPower.sleepFor(1000);
    int b = LowPower.sleepUntilPin(2, FALLING);
    int c = LowPower.dormantUntilPin(2, FALLING);
    int d = 0;
#if !defined(PICO_RP2040)
    d = LowPower.dormantFor(1000);
#endif
    Serial1.printf("%d %d %d %d\n", a, b, c, d);
}
