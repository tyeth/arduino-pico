#!/usr/bin/env python3
"""Experiment on top of fix_enum.py: space enumeration retries out (CFG_TUH_ENUM_RETRY_DELAY_MS, default 1000 ms)
instead of re-trying after the 150 ms attach debounce.  A CircuitPython Feather ESP32-S2 enumerated right after it
booted while the host was asleep STALLs SET_CONFIGURATION / hangs EP0 status stages for a second or two; back-to-back
retries spent the whole budget (2 attempts x (1 + 3 root retries)) inside that window.

Applies to both retry paths: enum_retry() (same attach, port reset again) and the root port retry that
enum_full_complete() posts as a new attach (root_retry_count > 0 while that attach is being enumerated).

usage: fix_enum_backoff.py <path to Adafruit_TinyUSB_Arduino>
"""
import pathlib
import sys

root = pathlib.Path(sys.argv[1])
rel = "src/host/usbh.c"
p = root / rel
s = p.read_text()

pairs = [
    (
        """  _usbh_data.enum_attempt  = 0;
  usbh_defer_func_ms_async(ENUM_DEBOUNCING_DELAY_MS, enum_delay_async, ENUM_AFTER_DEBOUNCING_DELAY);
""",
        """  _usbh_data.enum_attempt  = 0;
  // a root port retry of a failed attach: give the device time to settle before resetting it again
  usbh_defer_func_ms_async(_usbh_data.root_retry_count ? CFG_TUH_ENUM_RETRY_DELAY_MS : ENUM_DEBOUNCING_DELAY_MS,
                           enum_delay_async, ENUM_AFTER_DEBOUNCING_DELAY);
""",
    ),
    (
        """  _usbh_data.enumerating_daddr = 0;
  usbh_defer_func_ms_async(ENUM_DEBOUNCING_DELAY_MS, enum_delay_async, ENUM_AFTER_DEBOUNCING_DELAY);
""",
        """  _usbh_data.enumerating_daddr = 0;
  usbh_defer_func_ms_async(CFG_TUH_ENUM_RETRY_DELAY_MS, enum_delay_async, ENUM_AFTER_DEBOUNCING_DELAY);
""",
    ),
    (
        """enum {
  USBH_CONTROL_RETRY_MAX = 3,
""",
        """#ifndef CFG_TUH_ENUM_RETRY_DELAY_MS
#define CFG_TUH_ENUM_RETRY_DELAY_MS 1000 // wait before re-enumerating a device that failed enumeration
#endif

enum {
  USBH_CONTROL_RETRY_MAX = 3,
""",
    ),
]
for old, new in pairs:
    n = s.count(old)
    if n != 1:
        sys.exit(f"{rel}: pattern found {n} times (want 1):\n{old}")
    s = s.replace(old, new)
p.write_text(s)
print(f"patched {rel}")
