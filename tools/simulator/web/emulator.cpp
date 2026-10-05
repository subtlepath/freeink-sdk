#include "../emu/EmuMachine.h"
#include "../boards/BoardTable.h"
#include "../core/EmuBoard.h"
#include "../core/Machine.h"
#include "../core/Panel.h"
#include <emscripten.h>
#include <memory>
#include <string>

using namespace freeink::sim;
static std::unique_ptr<emu::EmuMachine> cpu;
static std::unique_ptr<EmuBoard> bridge;
static Frame frame;
static std::string error, console;
extern "C" {
EMSCRIPTEN_KEEPALIVE int web_load(const char* path, const char* name) {
  const auto* board = boards::byName(name);
  if (!board) { error = "Unknown board"; return 0; }
  emu::FlashImage image;
  if (!image.load(path, &error)) return 0;
  const auto* soc = emu::socForChip(image.app().chip);
  if (!soc || (std::string(name) == "X3") != (soc->arch == emu::Arch::RiscV)) {
    error = "Firmware chip does not match the selected device"; return 0;
  }
  auto& device = Machine::instance();
  device.describeBoard(*board);
  device.clock().setMode(ClockMode::Virtual);
  device.clock().setRate(0);
  if (!device.mountCard("/sd", 0, &error)) return 0;
  device.setButton(6, true);
  bridge = std::make_unique<EmuBoard>(device, soc->name);
  cpu = std::make_unique<emu::EmuMachine>();
  cpu->setBoard(bridge.get());
  cpu->setConsole([](const char*, const char* text, size_t size) {
    if (console.size() < 65536) console.append(text, size);
  });
  emu::EmuOptions options;
  options.romSymbolDir = "/rom";
  return cpu->load(path, options, &error) ? 1 : 0;
}
EMSCRIPTEN_KEEPALIVE int web_run(int budget) {
  if (!cpu || cpu->stopped()) return 0;
  const auto before = cpu->timeUs();
  cpu->run(budget);
  Machine::instance().clock().advanceExternal(cpu->timeUs() - before);
  Machine::instance().panel().tick();
  return cpu->stopped() ? 0 : 1;
}
EMSCRIPTEN_KEEPALIVE const char* web_error() {
  if (cpu && cpu->stopped()) error = cpu->stopReason();
  return error.c_str();
}
EMSCRIPTEN_KEEPALIVE void web_button(int index, int down) {
  if (index >= 0 && index < Machine::kButtonCount) Machine::instance().setButton(index, down != 0);
}
EMSCRIPTEN_KEEPALIVE void web_touch(int x, int y, int down) { Machine::instance().setTouch(x, y, down != 0); }
EMSCRIPTEN_KEEPALIVE const uint8_t* web_frame() { frame = Machine::instance().panel().currentFrame(); return frame.pixels.data(); }
EMSCRIPTEN_KEEPALIVE int web_width() { return frame.width; }
EMSCRIPTEN_KEEPALIVE int web_height() { return frame.height; }
EMSCRIPTEN_KEEPALIVE double web_sequence() { return static_cast<double>(Machine::instance().panel().frameSeq()); }
EMSCRIPTEN_KEEPALIVE const char* web_logs() { return console.c_str(); }
EMSCRIPTEN_KEEPALIVE void web_clear_logs() { console.clear(); }
EMSCRIPTEN_KEEPALIVE double web_time_ms() { return cpu ? cpu->timeUs() / 1000.0 : 0; }
}
