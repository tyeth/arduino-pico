#!/usr/bin/env python3
"""Test only: TinyUSB master behaviour in Adafruit_TinyUSB_Arduino 3.8.0's MSC host driver (part 1 of fix_msc_host.py):
a failed CBW/data/CSW transfer completes the command as failed instead of TU_ASSERT-returning with it pending.
Original docstring:

1. src/class/msc/msc_host.c, msch_xfer_cb(): a failed/short transfer in any stage (or a failure to queue the next
   stage) used to TU_ASSERT-return without invoking complete_cb, leaving the command pending forever.  Now the command
   is completed with a failed CSW (stage back to idle) so the application callback always runs.
2. src/arduino/msc/Adafruit_USBH_MSC.{h,cpp}: wait_for_io() gets a timeout (ADAFRUIT_USBH_MSC_IO_TIMEOUT_MS, default
   3000) and the block device honours csw->status (it used to assume success).

usage: fix_msc_host.py <path to Adafruit_TinyUSB_Arduino>
"""
import pathlib
import sys

root = pathlib.Path(sys.argv[1])


def patch(rel, pairs):
    p = root / rel
    s = p.read_text()
    for old, new in pairs:
        n = s.count(old)
        if n < 1:
            sys.exit(f"{rel}: pattern not found:\n{old}")
        s = s.replace(old, new)
    p.write_text(s)
    print(f"patched {rel}")


helper = """// Complete the pending command with a failed CSW so the caller's complete_cb always runs, instead of leaving the
// command pending forever (e.g. a stale device address after a disconnect the root port never saw)
static void msch_complete_failed(uint8_t dev_addr, msch_interface_t* p_msc, msc_cbw_t const* cbw, msc_csw_t* csw) {
  csw->signature    = MSC_CSW_SIGNATURE;
  csw->tag          = cbw->tag;
  csw->data_residue = cbw->total_bytes;
  csw->status       = MSC_CSW_STATUS_FAILED;
  p_msc->stage      = MSC_STAGE_IDLE;
  if (p_msc->complete_cb != NULL) {
    tuh_msc_complete_data_t const cb_data = {
        .cbw = cbw,
        .csw = csw,
        .scsi_data = p_msc->buffer,
        .user_arg = p_msc->complete_arg
    };
    (void) p_msc->complete_cb(dev_addr, &cb_data);
  }
}

bool msch_xfer_cb(uint8_t dev_addr, uint8_t ep_addr, xfer_result_t event, uint32_t xferred_bytes) {"""

patch("src/class/msc/msc_host.c", [
    ("bool msch_xfer_cb(uint8_t dev_addr, uint8_t ep_addr, xfer_result_t event, uint32_t xferred_bytes) {", helper),
    ("""      // Must be Command Block
      TU_ASSERT(ep_addr == p_msc->ep_out && event == XFER_RESULT_SUCCESS && xferred_bytes == sizeof(msc_cbw_t));""",
     """      // Must be Command Block
      if (!(ep_addr == p_msc->ep_out && event == XFER_RESULT_SUCCESS && xferred_bytes == sizeof(msc_cbw_t))) {
        msch_complete_failed(dev_addr, p_msc, cbw, csw);
        return true;
      }"""),
    ("""    case MSC_STAGE_DATA:
      p_msc->data_xferred += xferred_bytes;""",
     """    case MSC_STAGE_DATA:
      if (event == XFER_RESULT_FAILED) {
        msch_complete_failed(dev_addr, p_msc, cbw, csw);
        return true;
      }
      p_msc->data_xferred += xferred_bytes;"""),
    ("""    case MSC_STAGE_STATUS:
      // SCSI op is complete
      p_msc->stage = MSC_STAGE_IDLE;""",
     """    case MSC_STAGE_STATUS:
      // SCSI op is complete
      if (event != XFER_RESULT_SUCCESS) {
        csw->status = MSC_CSW_STATUS_FAILED;
      }
      p_msc->stage = MSC_STAGE_IDLE;"""),
    ("TU_ASSERT(data_stage_xfer(dev_addr, p_msc, cbw));",
     "if (!data_stage_xfer(dev_addr, p_msc, cbw)) { msch_complete_failed(dev_addr, p_msc, cbw, csw); return true; }"),
    ("TU_ASSERT(status_stage_xfer(dev_addr, p_msc, csw));",
     "if (!status_stage_xfer(dev_addr, p_msc, csw)) { msch_complete_failed(dev_addr, p_msc, cbw, csw); return true; }"),
])
