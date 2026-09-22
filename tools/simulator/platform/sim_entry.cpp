// FreeInk simulator — firmware bundle entry stub.
//
// The daemon dlopen()s the bundle and calls freeink_sim_main(). This file is
// the only place that knows the firmware is an Arduino sketch: it reports the
// compiled-in board profile to the daemon (so the daemon's virtual devices are
// wired to the same pins the SDK will drive), then runs setup()/loop().
//
// The board description is read from BoardConfig::ACTIVE rather than restated,
// so a board added to the SDK becomes simulatable without touching the daemon.

#include <Arduino.h>
#include <BoardConfig.h>
#include <freeink_sim_abi.h>

#include "../boards/BoardDesc.h"

#include <cstring>

// The consumer firmware's sketch entry points.
extern void setup();
extern void loop();

namespace {

void describeActiveBoard() {
  // Straight out of the profile the firmware was compiled with, through the
  // one translation the board table also uses.
  const fsim_board_desc desc = freeink::sim::boards::describe(BoardConfig::ACTIVE);
  fsim_describe_board(&desc);
}

}  // namespace

extern "C" {

// Identity, read by the daemon before it starts the firmware.
const freeink_sim_bundle_info* freeink_sim_info(void) {
  static freeink_sim_bundle_info info = {
      FREEINK_SIM_ABI_MAJOR,
      FREEINK_SIM_ABI_MINOR,
#ifdef FSIM_FIRMWARE_NAME
      FSIM_FIRMWARE_NAME,
#else
      "firmware",
#endif
      BoardConfig::ACTIVE.name,
      BoardConfig::ACTIVE.displayWidth,
      BoardConfig::ACTIVE.displayHeight,
#ifdef FSIM_BUILD_FLAGS
      FSIM_BUILD_FLAGS,
#else
      "",
#endif
  };
  return &info;
}

// Runs on the daemon's firmware thread. Returns only when the machine is torn
// down — a restart or deep-sleep wake unwinds this through fsim_power_event()
// and the daemon calls it again on a fresh bundle instance.
void freeink_sim_main(void) {
  describeActiveBoard();
  setup();
  while (!fsim_should_exit()) {
    loop();
    // Cooperative point: lets the daemon service control commands and run the
    // panel model even if the firmware's loop never sleeps.
    fsim_yield();
  }
}

// The daemon calls this when the board profile changes at runtime — X3 and X4
// share one C3 binary and pick a profile in setDisplayX3() — so the virtual
// devices follow the firmware's choice instead of the boot default.
void freeink_sim_redescribe(void) { describeActiveBoard(); }
}
