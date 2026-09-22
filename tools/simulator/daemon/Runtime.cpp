// FreeInk simulator — firmware bundle lifecycle.

#include "Runtime.h"

#include "../boards/BoardTable.h"
#include "../core/EmuBoard.h"
#include "../core/Machine.h"
#include "../core/Panel.h"
#include "../emu/EmuMachine.h"

#include <dlfcn.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <fstream>
#include <thread>

namespace freeink::sim {

Runtime::Runtime(Machine& machine) : machine_(machine) {}

Runtime::~Runtime() { stop(); }

namespace {

// A device image starts with the ESP32 image magic. Sniffing the file rather
// than trusting the extension means `sim load` does the right thing whatever
// the firmware is called.
bool looksLikeDeviceImage(const std::string& path) {
  std::ifstream file(path, std::ios::binary);
  if (!file) return false;
  char magic = 0;
  file.read(&magic, 1);
  return file.gcount() == 1 && static_cast<unsigned char>(magic) == 0xE9;
}

}  // namespace

bool Runtime::load(const std::string& path, std::string* error) {
  std::lock_guard<std::mutex> lock(lifecycle_);
  stopLocked();
  emulator_.reset();
  return looksLikeDeviceImage(path) ? loadImage(path, error) : loadBundle(path, error);
}

bool Runtime::loadImage(const std::string& path, std::string* error) {
  auto emulator = std::make_unique<emu::EmuMachine>();

  // The image names its chip; the board is either the operator's choice or the
  // usual one for that chip. Either way it is reported, because the difference
  // between "the X4's panel" and "the X4 Pro's panel" is not something the
  // reader should have to assume.
  emu::FlashImage probe;
  std::string probeError;
  const std::string chip =
      probe.load(path, &probeError) ? emu::chipName(probe.app().chip) : std::string();
  std::string boardName = deviceName_;
  const fsim_board_desc* board = nullptr;
  if (!boardName.empty()) {
    board = boards::byName(boardName);
    if (!board) {
      if (error) *error = "no board profile named " + boardName;
      return false;
    }
  } else {
    board = boards::defaultForChip(chip, &boardName);
  }
  if (board) {
    machine_.describeBoard(*board);
    board_ = std::make_unique<EmuBoard>(machine_, chip);
    emulator->setBoard(board_.get());
  }

  emu::EmuOptions options;
  options.romSymbolDir = romSymbolDir_;
  // Console output from every source the chip has — the ROM's printf, the
  // UART FIFO and the USB serial endpoint — lands in the same log the CLI
  // tails, so `sim log` shows a device image's boot exactly as a serial
  // monitor would.
  emulator->setConsole([this](const char* source, const char* text, size_t length) {
    // The ROM console and the UART are the same stream as far as a reader is
    // concerned; a USB serial console is a second copy of it, so it gets its
    // own channel and `log --channel firmware` shows each line once.
    machine_.log().append(std::strcmp(source, "usb") == 0 ? "usb" : "firmware", text, length);
  });

  if (!emulator->load(path, options, error)) return false;

  const emu::AppImage& app = emulator->flash().app();
  firmwareName_ = app.desc.projectName.empty() ? "device image" : app.desc.projectName;
  buildFlags_ = std::string(emulator->soc().name) + ", ESP-IDF " + app.desc.idfVersion;
  bundlePath_ = path;
  emulator_ = std::move(emulator);

  std::string banner = "[sim] emulating " + std::string(emulator_->soc().name) + " image " +
                       firmwareName_ + " " + app.desc.version + "\n";
  if (board) {
    banner += "[sim] board " + boardName + " (" + board->board_name + "), " +
              std::to_string(board->panel_width) + "x" + std::to_string(board->panel_height) + " " +
              machine_.panel().controllerName() +
              (deviceName_.empty() ? " — assumed from the chip; --device overrides it\n" : "\n");
  }
  machine_.log().append("sim", banner.c_str(), banner.size());

  // Emulating an instruction set costs far more than one host instruction per
  // guest instruction, so an emulated image cannot keep up with wall time. In
  // realtime mode the clock would run ahead of the firmware and every piece of
  // modelled timing — panel waveforms, BUSY, input sampling — would be keyed to
  // a clock the firmware is behind. Virtual time is not a convenience here, it
  // is the only mode in which the machine stays coherent, so loading an image
  // switches to it and says so.
  if (machine_.clock().mode() != ClockMode::Virtual) {
    machine_.clock().setMode(ClockMode::Virtual);
    const char* note =
        "[sim] clock switched to virtual: emulated instructions set the pace, not wall time\n";
    machine_.log().append("sim", note, std::strlen(note));
  }
  // And paced, so the device can be used. An idle emulated machine runs its
  // clock forward as fast as the host allows, which means a firmware's own
  // inactivity timeout fires long before anyone can press a key; pacing holds
  // simulated time to the configured rate. `freeink-sim clock --rate 0` lifts
  // it for a test that only wants to reach the end.
  if (!machine_.clock().paced()) {
    machine_.clock().setPaced(true);
    const std::string note = "[sim] clock paced at " +
                             std::to_string(machine_.clock().rate()) +
                             "x wall time so the device can be driven; `clock --rate 0` runs it "
                             "flat out\n";
    machine_.log().append("sim", note.c_str(), note.size());
  }

  machine_.clearTeardown();
  running_ = true;
  thread_ = std::thread([this]() { runEmulator(); });
  return true;
}

void Runtime::runEmulator() {
  // The emulated CPU is just another firmware thread as far as the clock is
  // concerned: it runs a slice, then hands back exactly as much simulated time
  // as the slice consumed. That is what makes `pause`, `step`, `--clock
  // virtual` and the rate multiplier work on an emulated image without the
  // emulator knowing anything about them.
  machine_.clock().enterRunning();
  constexpr uint64_t kSliceInstructions = 20000;

  // An idle emulated machine fast-forwards to its next timer, so simulated
  // time can outrun the wall by orders of magnitude. The clock is what holds
  // that to the configured rate: handing it the elapsed time and waiting for
  // it to catch up is all the pacing this loop needs.
  while (!machine_.tearingDown() && !emulator_->stopped()) {
    const uint64_t before = emulator_->timeUs();
    if (emulator_->run(kSliceInstructions) == 0) break;
    const uint64_t elapsed = emulator_->timeUs() - before;
    if (elapsed && machine_.clock().sleepUs(elapsed) != 0) break;
  }
  machine_.clock().exitRunning();

  if (emulator_->asleep() && !machine_.tearingDown()) {
    // The chip is off. On the device it comes back through reset when a wake
    // source fires, and the emulated wake sources are the two the operator can
    // actually produce: the sleep timer the firmware programmed, and the power
    // key. Everything else a real chip can wake on — a touch, a charger — is
    // not modelled, so `freeink-sim wake` stands in for it.
    const fsim_board_desc& board = machine_.board();
    const int powerKey = machine_.boardKnown() ? board.buttons[6] : -1;
    if (powerKey >= 0) machine_.setSleepWakePin(powerKey, board.power_active_high ? 1 : 0);
    machine_.enterDeepSleep(emulator_->sleepRequestUs());
    std::string message = "[sim] the firmware went to sleep";
    if (emulator_->sleepRequestUs()) {
      message += " for " + std::to_string(emulator_->sleepRequestUs() / 1000) + " ms";
    }
    message += powerKey >= 0 ? "; the power key or `wake` brings it back\n"
                             : "; `wake` brings it back\n";
    machine_.log().append("sim", message.c_str(), message.size());
    running_ = false;
    return;
  }

  if (emulator_->stopped() && !machine_.tearingDown()) {
    const std::string message = "[sim] the emulated firmware stopped: " + emulator_->stopReason() + "\n";
    machine_.log().append("sim", message.c_str(), message.size());
    const std::string exception = emulator_->firstExceptionReport();
    if (!exception.empty()) {
      const std::string detail = "[sim] first exception: " + exception + "\n";
      machine_.log().append("sim", detail.c_str(), detail.size());
    }
  }
  running_ = false;
}

bool Runtime::loadBundle(const std::string& path, std::string* error) {

  // RTLD_LOCAL so two bundles never share symbols; RTLD_NOW so a missing fsim_*
  // symbol is reported here, by name, rather than as a crash mid-refresh.
  void* handle = ::dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
  if (!handle) {
    if (error) *error = ::dlerror() ? ::dlerror() : "dlopen failed";
    return false;
  }

  using InfoFn = const freeink_sim_bundle_info* (*)();
  auto info = reinterpret_cast<InfoFn>(::dlsym(handle, "freeink_sim_info"));
  auto entry = reinterpret_cast<void (*)()>(::dlsym(handle, "freeink_sim_main"));
  if (!entry) {
    ::dlclose(handle);
    if (error) {
      *error = "not a FreeInk firmware bundle: freeink_sim_main is missing (build it with build/build-firmware.sh)";
    }
    return false;
  }

  if (info) {
    const freeink_sim_bundle_info* bundle = info();
    if (bundle && bundle->abi_major != FREEINK_SIM_ABI_MAJOR) {
      ::dlclose(handle);
      if (error) {
        *error = "bundle ABI " + std::to_string(bundle->abi_major) + " does not match daemon ABI " +
                 std::to_string(FREEINK_SIM_ABI_MAJOR) + "; rebuild the firmware bundle";
      }
      return false;
    }
    if (bundle) {
      firmwareName_ = bundle->firmware_name ? bundle->firmware_name : "firmware";
      buildFlags_ = bundle->build_flags ? bundle->build_flags : "";
    }
  }

  handle_ = handle;
  entry_ = entry;
  bundlePath_ = path;
  // A bundle runs at host speed: there is nothing to pace, and a virtual clock
  // that waits on the wall would throw away the whole reason for having one.
  machine_.clock().setPaced(false);

  machine_.clearTeardown();
  running_ = true;
  thread_ = std::thread([this]() { runFirmware(); });
  return true;
}

void Runtime::runFirmware() {
  machine_.clock().enterRunning();
  entry_();
  machine_.clock().exitRunning();
  running_ = false;
}

void Runtime::joinFirmwareThread() {
  if (!thread_.joinable()) return;
  // Ask the firmware to unwind: its entry loop polls fsim_should_exit(), and
  // every blocking wait returns early once teardown is set.
  machine_.requestTeardown();

  // Bounded wait. Firmware can legitimately be inside a long call that does not
  // poll, and a simulator that hangs on shutdown is worse than one that says so
  // and lets the thread finish on its own.
  for (int waited = 0; waited < 2000 && running_.load(); waited += 10) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (running_.load()) {
    const char* message = "[sim] firmware did not unwind within 2s; detaching its thread\n";
    machine_.log().append("sim", message, std::strlen(message));
    thread_.detach();
    return;
  }
  thread_.join();
}

void Runtime::stop() {
  std::lock_guard<std::mutex> lock(lifecycle_);
  stopLocked();
}

void Runtime::stopLocked() {
  if (!thread_.joinable()) return;
  joinFirmwareThread();
  running_ = false;
  // The handle is intentionally kept mapped; see the note in Runtime.h.
}

bool Runtime::reset(std::string* error) {
  if (bundlePath_.empty()) {
    if (error) *error = "no firmware loaded";
    return false;
  }
  return load(bundlePath_, error);
}

void Runtime::servicePowerTransitions() {
  if (reloading_.load()) return;

  const bool wake = machine_.shouldWake();
  if (wake) {
    // ESP_SLEEP_WAKEUP_TIMER (4) or GPIO (7), matching esp_sleep.h.
    machine_.wakeNow(7);
  }

  if (!machine_.tearingDown()) return;

  reloading_ = true;
  std::string error;
  // Reloading on the clock thread would deadlock against the firmware thread's
  // own teardown, so do it on a detached thread and let the clock keep running.
  std::thread([this]() {
    std::string error;
    if (!load(bundlePath_, &error)) {
      const std::string message = "[sim] firmware reload failed: " + error + "\n";
      machine_.log().append("sim", message.c_str(), message.size());
    }
    reloading_ = false;
  }).detach();
}

}  // namespace freeink::sim
