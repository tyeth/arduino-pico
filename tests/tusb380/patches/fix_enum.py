#!/usr/bin/env python3
"""Patch Adafruit_TinyUSB_Arduino (3.8.0) host enumeration so a device that half-fails configuration is retried
instead of wedging the host stack (seen on a Feather RP2040 USB Host re-enumerating a CircuitPython Feather ESP32-S2
after a forced root port reconnect: the S2 STALLed SET_CONFIGURATION, TinyUSB carried on as "Device configured", the
MSC driver's Test Unit Ready / Request Sense then failed on the unconfigured endpoints, config_request_sense_complete
TU_ASSERTed out without completing its set_config step, and enumerating_daddr stayed set forever: no mount, no retry).

1. src/host/usbh.c, process_enumeration(): a STALLed SET_CONFIGURATION is an enumeration failure (enum_retry(), then
   the root port retry in enum_full_complete()), not success.  Other STALLs keep their meaning (string descriptors,
   class requests).
2. src/class/msc/msc_host.c: the MSC set_config chain (Test Unit Ready / Request Sense / Read Capacity) is bounded and,
   when it fails, still completes its set_config step (usbh_driver_set_config_complete) so the rest of the device and
   the host's enumeration carry on; the MSC interface just doesn't mount.

usage: fix_enum.py <path to Adafruit_TinyUSB_Arduino>   (apply after fix_msc_host.py, either order works)
"""
import pathlib
import sys

root = pathlib.Path(sys.argv[1])


def patch(rel, pairs):
    p = root / rel
    s = p.read_text()
    for old, new in pairs:
        n = s.count(old)
        if n != 1:
            sys.exit(f"{rel}: pattern found {n} times (want 1):\n{old}")
        s = s.replace(old, new)
    p.write_text(s)
    print(f"patched {rel}")


patch("src/host/usbh.c", [(
    """static void process_enumeration(tuh_xfer_t *xfer) {
  if (XFER_RESULT_FAILED == xfer->result || XFER_RESULT_TIMEOUT == xfer->result) {
    if (!enum_retry()) {
      enum_full_complete(false);
    }
    return;
  }
""",
    """static void process_enumeration(tuh_xfer_t *xfer) {
  if (XFER_RESULT_FAILED == xfer->result || XFER_RESULT_TIMEOUT == xfer->result) {
    if (!enum_retry()) {
      enum_full_complete(false);
    }
    return;
  }

  // A device that STALLs SET_CONFIGURATION is not configured: its endpoints don't exist and every driver will fail
  // against it.  Re-enumerate (fresh port reset) instead of carrying on as configured.
  if (XFER_RESULT_STALLED == xfer->result && ENUM_CONFIG_DRIVER == xfer->user_data) {
    TU_LOG_USBH("[%u] SET_CONFIGURATION STALLed\\r\\n", xfer->daddr);
    if (!enum_retry()) {
      enum_full_complete(false);
    }
    return;
  }
""")])

patch("src/class/msc/msc_host.c", [
    (
        """static void config_get_maxlun_complete(tuh_xfer_t* xfer);
""",
        """static void config_get_maxlun_complete(tuh_xfer_t* xfer);
static bool config_failed(uint8_t dev_addr, const char* why);
// Test Unit Ready / Request Sense rounds allowed while the MSC interface is being configured
#ifndef CFG_TUH_MSC_CONFIG_TUR_MAX
#define CFG_TUH_MSC_CONFIG_TUR_MAX 100
#endif
static uint16_t _config_tur_count;
"""),
    (
        """  // TODO multiple LUN support
  TU_LOG_DRV("SCSI Test Unit Ready\\r\\n");
  uint8_t const lun = 0;
  tuh_msc_test_unit_ready(daddr, lun, config_test_unit_ready_complete, 0);
}
""",
        """  // TODO multiple LUN support
  TU_LOG_DRV("SCSI Test Unit Ready\\r\\n");
  uint8_t const lun = 0;
  _config_tur_count = 0;
  if (!tuh_msc_test_unit_ready(daddr, lun, config_test_unit_ready_complete, 0)) {
    config_failed(daddr, "queue Test Unit Ready");
  }
}

// The MSC interface could not be brought up.  Still complete its set_config step: otherwise usbh waits for it forever
// (enumerating_daddr stays set) and no other device, nor a retry of this one, is ever enumerated.
static bool config_failed(uint8_t dev_addr, const char* why) {
  msch_interface_t* p_msc = get_itf(dev_addr);
  TU_LOG_DRV("  MSC config failed: %s, continuing without it\\r\\n", why);
  (void) why;
  usbh_driver_set_config_complete(dev_addr, p_msc->itf_num);
  return false;
}
"""),
    (
        """    // TODO limit number of retries
    TU_LOG_DRV("SCSI Request Sense\\r\\n");
    TU_ASSERT(tuh_msc_request_sense(dev_addr, cbw->lun, enum_buf, config_request_sense_complete, 0));
  }
""",
        """    if (++_config_tur_count >= CFG_TUH_MSC_CONFIG_TUR_MAX) {
      return config_failed(dev_addr, "Test Unit Ready never passed");
    }
    TU_LOG_DRV("SCSI Request Sense\\r\\n");
    if (!tuh_msc_request_sense(dev_addr, cbw->lun, enum_buf, config_request_sense_complete, 0)) {
      return config_failed(dev_addr, "queue Request Sense");
    }
  }
"""),
    (
        """  TU_ASSERT(csw->status == 0);
  TU_ASSERT(tuh_msc_test_unit_ready(dev_addr, cbw->lun, config_test_unit_ready_complete, 0));
  return true;
}
""",
        """  if (csw->status != 0) {
    return config_failed(dev_addr, "Request Sense failed");
  }
  if (!tuh_msc_test_unit_ready(dev_addr, cbw->lun, config_test_unit_ready_complete, 0)) {
    return config_failed(dev_addr, "queue Test Unit Ready");
  }
  return true;
}
"""),
    (
        """  msc_csw_t const* csw = cb_data->csw;
  TU_ASSERT(csw->status == 0);
  msch_interface_t* p_msc = get_itf(dev_addr);
  uint8_t* enum_buf = usbh_get_enum_buf();
""",
        """  msc_csw_t const* csw = cb_data->csw;
  if (csw->status != 0) {
    return config_failed(dev_addr, "Read Capacity failed");
  }
  msch_interface_t* p_msc = get_itf(dev_addr);
  uint8_t* enum_buf = usbh_get_enum_buf();
"""),
])
