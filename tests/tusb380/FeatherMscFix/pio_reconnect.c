// Force Pico-PIO-USB root port re-detection (C: pio_usb_ll.h is not C++-clean).
// Marks the root port disconnected and fails its queued transfers, exactly as connection_check() does when it sees
// SE0.  The next SOF frame then re-detects the attached device and TinyUSB re-enumerates it.  Call on the core that
// runs the PIO-USB host (core1 here).
#include <stdbool.h>
#include <stdint.h>
#include "hardware/sync.h"
#include "pio_usb.h"
#include "pio_usb_ll.h"

void pio_usb_force_reconnect(uint8_t root_idx) {
    uint32_t s = save_and_disable_interrupts();
    root_port_t *root = PIO_USB_ROOT_PORT(root_idx);
    if (root->initialized && root->connected) {
        root->connected = false;
        root->suspended = true;
        root->ints |= PIO_USB_INTS_DISCONNECT_BITS;
        for (int i = 0; i < PIO_USB_EP_POOL_CNT; i++) {
            endpoint_t *ep = PIO_USB_ENDPOINT(i);
            if (ep->root_idx == root_idx && ep->size && ep->has_transfer) {
                pio_usb_ll_transfer_complete(ep, PIO_USB_INTS_ENDPOINT_ERROR_BITS);
            }
        }
    }
    restore_interrupts(s);
}
