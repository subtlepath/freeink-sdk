// Browser host for source-compiled firmware. The same SPI panel and electrical
// inputs as the native daemon, with firmware/RTOS threads off the browser UI.
#include "../core/Machine.h"
#include "../core/Panel.h"
#include <freeink_sim_abi.h>
#include <emscripten.h>
#include <freertos/task.h>
#include <thread>
#include <string>

extern "C" void freeink_sim_main();
using freeink::sim::Machine;
using freeink::sim::PowerState;
static freeink::sim::Frame frame;
static bool started = false;
static uint64_t logSequence = 0;
static std::string logText;

extern "C" {
EMSCRIPTEN_KEEPALIVE void web_start(int powerWake) {
  if (started) return;
  started = true;
  auto& machine = Machine::instance();
  machine.clock().setMode(freeink::sim::ClockMode::Realtime);
  machine.storage().mountCard("/sd", 64 * 1024 * 1024);
  machine.storage().nvsSet("tinta", "sim_pack", "/course.pack");
  machine.storage().nvsSet("tinta", "sim_clock", "2026-10-04T12:00:00");
  // These demos model the SSD1677-equipped S3 boards. Seed the same factory
  // calibration byte used on hardware so automatic detection keeps that panel.
  machine.storage().nvsSet("hw_calib", "screenType", std::string(1, '\x03'));
  machine.setButton(6, powerWake != 0);
  // SPI transactions often charge only microseconds. A JavaScript timer has
  // millisecond granularity and would turn every byte wait into a visible
  // stall. Advance the model on a dedicated pthread, like the native daemon.
  std::thread([powerWake] {
    auto& machine = Machine::instance();
    bool bootHeld = powerWake != 0;
    while (!machine.tearingDown()) {
      machine.clock().tick();
      machine.panel().tick();
      if (bootHeld && machine.clock().nowUs() > 1000000) {
        machine.setButton(6, false);
        bootHeld = false;
      }
      std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
  }).detach();
  xTaskCreate([](void*) { freeink_sim_main(); }, "firmware", 2 * 1024 * 1024, nullptr, 1, nullptr);
}
EMSCRIPTEN_KEEPALIVE int web_tick() {
  auto& machine = Machine::instance();
  if (machine.shouldWake()) {
    machine.requestTeardown();
    return 2; // Worker reloads a fresh firmware instance, as a hardware reset.
  }
  return machine.powerState() == PowerState::Running ? 0 : 1;
}
EMSCRIPTEN_KEEPALIVE void web_button(int index, int pressed) {
  if (index < 0 || index >= Machine::kButtonCount) return;
  Machine::instance().setButton(index, pressed != 0);
}
EMSCRIPTEN_KEEPALIVE void web_touch(int x, int y, int down) {
  Machine::instance().setTouch(x, y, down != 0);
}
EMSCRIPTEN_KEEPALIVE const uint8_t* web_frame() {
  frame = Machine::instance().panel().currentFrame();
  return frame.pixels.data();
}
EMSCRIPTEN_KEEPALIVE int web_width() { return frame.width; }
EMSCRIPTEN_KEEPALIVE int web_height() { return frame.height; }
EMSCRIPTEN_KEEPALIVE double web_sequence() { return static_cast<double>(Machine::instance().panel().frameSeq()); }
EMSCRIPTEN_KEEPALIVE const char* web_logs() {
  logText.clear();
  for (const auto& line : Machine::instance().log().since(logSequence)) {
    logSequence = line.first;
    logText += line.second;
    logText += '\n';
  }
  return logText.c_str();
}
}
