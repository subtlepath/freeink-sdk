#pragma once

// FreeInk simulator — USB mass storage shim.
//
// The daemon exposes the virtual card to the CLI rather than to a real USB
// host: `freeink-sim usb msc read/write` issues sector callbacks into the
// firmware, so the MSC read/write handlers are genuinely exercised — including
// the "don't touch the filesystem while the host owns the card" invariant.

#include <Arduino.h>
#include <freeink_sim_abi.h>

typedef int32_t (*msc_read_cb)(uint32_t lba, uint32_t offset, void* buffer, uint32_t bufsize);
typedef int32_t (*msc_write_cb)(uint32_t lba, uint32_t offset, uint8_t* buffer, uint32_t bufsize);
typedef bool (*msc_start_stop_cb)(uint8_t power_condition, bool start, bool load_eject);

class USBMSC {
 public:
  bool begin(uint32_t block_count, uint16_t block_size);
  void end();
  void vendorID(const char*) {}
  void productID(const char*) {}
  void productRevision(const char*) {}
  void mediaPresent(bool present) { _present = present; }
  void isWritable(bool writable) { _writable = writable; }
  void onRead(msc_read_cb cb);
  void onWrite(msc_write_cb cb);
  void onStartStop(msc_start_stop_cb cb);

 private:
  bool _present = false;
  bool _writable = false;
};
