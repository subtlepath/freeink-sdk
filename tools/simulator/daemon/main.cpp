// FreeInk simulator daemon.
//
// Owns the virtual machine, the control socket and (unless headless) the
// window. The main thread runs the clock and the window because macOS requires
// the SDL event loop there; the socket and the firmware get their own threads.

#include "../core/I2cDevices.h"
#include "../core/Machine.h"
#include "../core/Panel.h"
#include "../emu/EmuMachine.h"
#include "Runtime.h"
#include "Server.h"
#include "Window.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#include <pwd.h>
#include <sys/stat.h>
#include <unistd.h>

namespace {

std::atomic<bool> g_stop{false};

void onSignal(int) { g_stop = true; }

std::string defaultStateDir() {
  const char* home = ::getenv("HOME");
  if (!home) {
    const struct passwd* pw = ::getpwuid(::getuid());
    home = pw ? pw->pw_dir : "/tmp";
  }
  return std::string(home) + "/.freeink-sim";
}

bool ensureDirectory(const std::string& path) {
  struct stat st {};
  if (::stat(path.c_str(), &st) == 0) return S_ISDIR(st.st_mode);
  return ::mkdir(path.c_str(), 0700) == 0;
}

void printUsage() {
  std::cout << R"(freeink-simd — FreeInk device simulator daemon

Usage:
  freeink-simd [options] [firmware-bundle]

Options:
  --socket PATH       Control socket (default: $HOME/.freeink-sim/sim.sock)
  --state-dir PATH    Persistent state: NVS, virtual flash (default: $HOME/.freeink-sim)
  --sd PATH           Mount a host directory as the SD card at startup. A
                      bundle reads it as files; a device image gets a FAT32
                      volume built from it and mounts that through its own
                      driver
  --sd-capacity MB    Virtual card capacity in MB (default: 8192)
  --device NAME       Board a device image is running on (X3, X4, X4PRO,
                      X4CLASSIC, ...). A bundle reports its own and ignores
                      this; an image carries none, so without it the usual
                      board for the image's chip is assumed.
  --headless          Do not open a window; capture screens over the socket
  --clock MODE        realtime (default) or virtual — virtual advances only when
                      the firmware is idle, making runs fast and reproducible
  --rate N            Realtime clock multiplier (default: 1.0)
  --seed N            Seed the firmware's random source, for reproducible runs
  --rom-dir PATH      Where to find <chip>.romsyms for emulating device images
                      (default: alongside the daemon, in ../emu/rom)
  --paused            Start paused, so you can set state before setup() runs
  -h, --help          Show this help

The firmware is either a bundle — a shared library built by
build/build-firmware.sh from your own source — or a device image, the .bin that
would be flashed to the device. The daemon tells them apart by reading the
file. Either can also be loaded later with `freeink-sim load <path>`.
)";
}

}  // namespace

int main(int argc, char** argv) {
  using namespace freeink::sim;

  std::string socketPath;
  std::string stateDir = defaultStateDir();
  std::string sdPath;
  std::string deviceName;
  std::string bundlePath;
  long sdCapacityMb = 8192;
  std::string romDir;
  bool headless = false;
  bool startPaused = false;
  ClockMode clockMode = ClockMode::Realtime;
  double rate = 1.0;
  long seed = 0;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&](const char* name) -> std::string {
      if (i + 1 >= argc) {
        std::cerr << "freeink-simd: " << name << " needs a value\n";
        std::exit(2);
      }
      return argv[++i];
    };
    if (arg == "-h" || arg == "--help") {
      printUsage();
      return 0;
    } else if (arg == "--socket") {
      socketPath = next("--socket");
    } else if (arg == "--state-dir") {
      stateDir = next("--state-dir");
    } else if (arg == "--device") {
      deviceName = next("--device");
    } else if (arg == "--sd") {
      sdPath = next("--sd");
    } else if (arg == "--sd-capacity") {
      sdCapacityMb = std::stol(next("--sd-capacity"));
    } else if (arg == "--headless") {
      headless = true;
    } else if (arg == "--paused") {
      startPaused = true;
    } else if (arg == "--clock") {
      const std::string mode = next("--clock");
      if (mode != "realtime" && mode != "virtual") {
        std::cerr << "freeink-simd: --clock must be realtime or virtual\n";
        return 2;
      }
      clockMode = mode == "virtual" ? ClockMode::Virtual : ClockMode::Realtime;
    } else if (arg == "--rate") {
      rate = std::stod(next("--rate"));
    } else if (arg == "--seed") {
      seed = std::stol(next("--seed"));
    } else if (arg == "--rom-dir") {
      romDir = next("--rom-dir");
    } else if (arg.rfind("--", 0) == 0) {
      std::cerr << "freeink-simd: unknown option " << arg << "\n";
      return 2;
    } else {
      bundlePath = arg;
    }
  }

  if (!ensureDirectory(stateDir)) {
    std::cerr << "freeink-simd: cannot create state directory " << stateDir << "\n";
    return 1;
  }
  if (socketPath.empty()) socketPath = stateDir + "/sim.sock";

  Machine& machine = Machine::instance();
  machine.storage().setStateDir(stateDir);
  machine.storage().nvsLoad(stateDir + "/nvs.store");
  machine.clock().setMode(clockMode);
  machine.clock().setRate(rate);
  if (seed) machine.seedRandom(static_cast<uint32_t>(seed));
  if (startPaused) machine.clock().pause();

  if (!sdPath.empty()) {
    struct stat st {};
    if (::stat(sdPath.c_str(), &st) != 0 || !S_ISDIR(st.st_mode)) {
      std::cerr << "freeink-simd: --sd path is not a directory: " << sdPath << "\n";
      return 1;
    }
    std::string cardError;
    if (!machine.mountCard(sdPath, static_cast<uint64_t>(sdCapacityMb) * 1024 * 1024, &cardError)) {
      std::cerr << "freeink-simd: " << cardError << "\n";
      return 1;
    }
  }

  Runtime runtime(machine);
  if (romDir.empty()) {
    // The ROM symbol maps ship beside the emulator sources, which sit next to
    // the build directory the daemon is built into.
    const std::string self = argv[0];
    const size_t slash = self.find_last_of('/');
    romDir = (slash == std::string::npos ? std::string(".") : self.substr(0, slash)) + "/../emu/rom";
  }
  runtime.setRomSymbolDir(romDir);
  runtime.setDeviceName(deviceName);

  Server server(machine, runtime);

  std::string error;
  if (!server.listen(socketPath, &error)) {
    std::cerr << "freeink-simd: " << error << "\n";
    return 1;
  }
  server.start();

  Window window(machine, runtime);
  if (!headless) {
    std::string windowError;
    if (!window.open(&windowError)) {
      // A missing display is not a reason to lose the simulation: say so and
      // keep going headless, which is still fully drivable over the socket.
      std::cerr << "freeink-simd: continuing headless (" << windowError << ")\n";
      headless = true;
    } else {
      machine.setBoardChangedHook([&window]() { window.requestResize(); });
    }
  }

  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);
  // A client that disconnects mid-write must not kill the daemon.
  std::signal(SIGPIPE, SIG_IGN);

  std::cout << "freeink-simd listening on " << socketPath << (headless ? " (headless)" : "") << "\n";
  std::cout << "  freeink-sim --socket " << socketPath << " status\n";

  if (!bundlePath.empty()) {
    if (!runtime.load(bundlePath, &error)) {
      std::cerr << "freeink-simd: " << error << "\n";
    } else {
      std::cout << "loaded " << bundlePath << "\n";
    }
  }

  // Main loop: advance the clock, run the panel waveform, service power
  // transitions, and paint. 4 ms is well under the shortest waveform, so the
  // panel's BUSY timing stays accurate without spinning a core.
  while (!g_stop.load()) {
    const bool advanced = machine.clock().tick();
    machine.panel().tick();
    runtime.servicePowerTransitions();

    if (!headless && window.isOpen()) {
      if (!window.pump()) break;  // window closed
      continue;                   // vsync already paces this loop
    }

    // In virtual mode, keep going the moment time moved: the firmware's waits
    // are sliced, so the loop rate is what decides whether a simulated second
    // costs a millisecond of wall time or a second of it. Back off only when
    // nothing advanced (firmware is computing, or the machine is paused), so an
    // idle daemon does not spin a core.
    if (advanced && machine.clock().mode() == ClockMode::Virtual) continue;
    std::this_thread::sleep_for(std::chrono::microseconds(
        machine.clock().mode() == ClockMode::Virtual ? 50 : 2000));
  }

  std::cout << "\nfreeink-simd shutting down\n";
  machine.clock().shutdown();
  runtime.stop();
  machine.storage().nvsSave();
  server.stop();
  window.close();

  // _Exit, not return: firmware can leave threads running that this process
  // could not join (Runtime says so when it detaches one), and letting static
  // destructors tear the Machine down underneath such a thread turns a clean
  // shutdown into a crash. Everything that needed flushing is flushed above.
  std::_Exit(0);
}
