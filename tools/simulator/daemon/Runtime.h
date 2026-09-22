#pragma once

// FreeInk simulator — firmware bundle lifecycle.
//
// A bundle is a shared library holding the firmware, the SDK and the platform
// shims. Loading one starts a thread that calls its freeink_sim_main(); a
// restart or a deep-sleep wake tears that thread down and re-enters, which is
// what makes reboot and wake paths genuinely testable rather than approximated.
//
// The daemon deliberately does not dlclose() a bundle it has run. Firmware and
// the SDK register plenty of file-static state and background threads;
// unloading the image underneath them is how a simulator turns a firmware bug
// into an unexplainable crash in the tool. Reloading maps a fresh copy.
//
// There are two kinds of firmware, told apart by the file itself:
//
//   * a **bundle** (.dylib/.so) — your source, compiled for the host. Fast,
//     debuggable with host tools, and what you want while writing firmware.
//   * an **image** (.bin) — the artefact that gets flashed to a device, run
//     instruction by instruction on an emulated SoC. Slower, and the only way
//     to run firmware you did not build.
//
// Both drive the same virtual machine, so every CLI command works either way.

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace freeink::sim {

class Machine;

class EmuBoard;

namespace emu {
class EmuMachine;
}

class Runtime {
 public:
  explicit Runtime(Machine& machine);
  ~Runtime();

  // Loads (or reloads) a bundle and starts the firmware. Any running firmware
  // is stopped first. Returns false with `error` set on failure.
  bool load(const std::string& path, std::string* error);
  void stop();
  // Tears the firmware down and re-enters setup() on a fresh bundle instance,
  // as a reset would.
  bool reset(std::string* error);

  bool running() const { return running_.load(); }
  const std::string& bundlePath() const { return bundlePath_; }
  std::string firmwareName() const { return firmwareName_; }
  std::string buildFlags() const { return buildFlags_; }

  // Non-null when the loaded firmware is a device image being emulated. The
  // control socket reports its state and reads its memory through this.
  emu::EmuMachine* emulator() const { return emulator_.get(); }
  // Where to find <chip>.romsyms. Set from the daemon's own location.
  void setRomSymbolDir(const std::string& dir) { romSymbolDir_ = dir; }
  // Which board a device image is running on. An image carries no profile of
  // its own, so without this the emulator can execute it and nothing else:
  // no panel, no buttons, no I2C. Empty means "infer from the image's chip",
  // and the log says which board that turned out to be.
  void setDeviceName(const std::string& name) { deviceName_ = name; }
  const std::string& deviceName() const { return deviceName_; }

  // Called from the daemon's clock thread: notices a firmware-requested
  // restart or a due deep-sleep wake and performs the reload.
  void servicePowerTransitions();

 private:
  bool loadBundle(const std::string& path, std::string* error);
  bool loadImage(const std::string& path, std::string* error);
  void runFirmware();
  void runEmulator();
  void joinFirmwareThread();
  // stop(), with `lifecycle_` already held.
  void stopLocked();

  Machine& machine_;
  std::string bundlePath_;
  std::string firmwareName_;
  std::string buildFlags_;
  void* handle_ = nullptr;
  void (*entry_)() = nullptr;
  std::unique_ptr<emu::EmuMachine> emulator_;
  // Lives as long as the emulator does: the emulated chip's pin-facing
  // peripherals hold a pointer to it.
  std::unique_ptr<EmuBoard> board_;
  std::string romSymbolDir_;
  std::string deviceName_;
  std::thread thread_;
  // Loading, resetting and stopping all take this. Without it two of them can
  // run at once — a `reset` over the socket and the reload the clock thread
  // starts when it notices the firmware asked to restart — and the second one
  // to reach joinFirmwareThread() joins a thread the first has already joined,
  // which throws out of a destructorless path and aborts the daemon.
  std::mutex lifecycle_;
  std::atomic<bool> running_{false};
  std::atomic<bool> reloading_{false};
};

}  // namespace freeink::sim
