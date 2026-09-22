// freeink-emu — inspect and boot a device firmware image, without the daemon.
//
// The simulator daemon is the thing you drive; this is the thing you reach for
// when the question is about the image itself, or when a boot stops and you
// want the report without a socket in the way. The regression suite uses it
// for the same reason.

#include "EmuMachine.h"

#include "../boards/BoardTable.h"
#include "../core/EmuBoard.h"
#include "../core/Machine.h"
#include "../core/Panel.h"
#include "../core/VirtualCard.h"
#include "../daemon/Png.h"

#include <atomic>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using namespace freeink::sim::emu;
// `Machine` is spelled out: the emulator has a forward declaration of its own
// under that name, and the two are different things.
using Device = freeink::sim::Machine;

namespace {

void printUsage() {
  std::cout << R"(freeink-emu — inspect and run ESP32 firmware images

Usage:
  freeink-emu info <firmware.bin>
  freeink-emu boot <firmware.bin> [options]
  freeink-emu boards

Options:
  --limit N            Stop after N instructions (default: 200000000)
  --rom-dir PATH       Directory holding <chip>.romsyms
  --partitions PATH    Use a real partitions.bin instead of a synthesized table
  --quiet              Suppress the firmware's console output
  --registers N        After stopping, list the N most-touched unmodelled registers
  --seed N             Seed the hardware RNG (default: 1), for reproducible runs
  --registers-at ADDR  After stopping, dump every register touched in that
                       peripheral block (e.g. --registers-at 0x60023000)
  --break ADDR         Stop when execution reaches ADDR, with registers intact.
                       May be given more than once.
  --profile N          Sample the PC while running and list the N hottest
                       addresses afterwards. The tool for "it boots and then
                       just sits there".
  --device NAME        Board to run the image on (X3, X4, X4PRO, X4CLASSIC, ...).
                       An image carries no board profile, so without this the
                       machine is a bare chip: no panel, no buttons, no I2C.
                       Defaults to the usual board for the image's chip.
  --no-board           Run the bare chip deliberately, with every bus empty.
  --capture PATH       Write what the panel is showing to a PNG before exiting.
  --tasks              List the firmware's FreeRTOS tasks, and which one was
                       running when it stopped.
  --blocks             List every peripheral block the firmware touched, busiest
                       first, and whether the emulator models it.
  --trace-bus          Print every I2C and SPI transaction as it happens.
  --card DIR           Insert an SD card holding a FAT32 volume built from DIR.
                       A reader firmware with no card shows an error screen and
                       goes no further, so this is usually what separates a
                       boot from a session.
  --card-image PATH    Insert a raw card image instead, and write changes back.
  --touch X,Y          Hold a finger on the panel at that point for the whole
                       run, on a board with a digitizer. The counterpart of
                       --hold for a device whose keys are on the glass.
  --hold BUTTON        Hold a button down for the whole run (back, confirm,
                       left, right, up, down, power). Repeatable. A device that
                       is woken by its power key boots differently from one
                       that is not, so this is often what a firmware is
                       waiting for.
)";
}

std::string humanSize(uint32_t bytes) {
  char buf[32];
  if (bytes >= 1024 * 1024) {
    std::snprintf(buf, sizeof(buf), "%.2f MB", bytes / (1024.0 * 1024.0));
  } else {
    std::snprintf(buf, sizeof(buf), "%.1f KB", bytes / 1024.0);
  }
  return buf;
}

int doInfo(const std::string& path) {
  FlashImage flash;
  std::string error;
  if (!flash.load(path, &error)) {
    std::cerr << "freeink-emu: " << error << "\n";
    return 1;
  }
  const AppImage& app = flash.app();
  const SocDesc* soc = socForChip(app.chip);

  std::printf("%s\n", path.c_str());
  std::printf("  chip          %s%s\n", chipName(app.chip), soc ? "" : "  (no emulator model)");
  if (soc) {
    std::printf("  architecture  %s, %d core%s\n", soc->arch == Arch::RiscV ? "RISC-V" : "Xtensa",
                soc->cores, soc->cores == 1 ? "" : "s");
  }
  std::printf("  entry point   0x%08X\n", app.entry);
  std::printf("  image size    %s\n", humanSize(app.imageLength).c_str());
  std::printf("  flash         %s, %s @ %s\n", humanSize(app.flashSizeBytes).c_str(),
              spiModeName(app.spiMode), spiSpeedName(app.spiSpeedCode));
  if (app.desc.present) {
    std::printf("  project       %s\n", app.desc.projectName.c_str());
    std::printf("  version       %s\n", app.desc.version.c_str());
    std::printf("  built         %s %s with ESP-IDF %s\n", app.desc.date.c_str(), app.desc.time.c_str(),
                app.desc.idfVersion.c_str());
  }

  std::printf("\n  segments\n");
  for (size_t i = 0; i < app.segments.size(); ++i) {
    const ImageSegment& segment = app.segments[i];
    const char* where = "RAM";
    if (soc) {
      if (inRange(soc->iromCache, segment.addr)) where = "flash-mapped IROM";
      else if (inRange(soc->dromCache, segment.addr)) where = "flash-mapped DROM";
      else if (inRange(soc->rtcFast, segment.addr)) where = "RTC fast";
      else if (inRange(soc->rtcSlow, segment.addr)) where = "RTC slow";
    }
    std::printf("    %zu  0x%08X  %9s  %s\n", i, segment.addr, humanSize(segment.length).c_str(), where);
  }

  std::printf("\n  partitions%s\n", flash.synthesizedTable() ? " (synthesized — pass --partitions for the real table)" : "");
  for (const Partition& partition : flash.partitions()) {
    std::printf("    %-10s %-14s 0x%06X  %s\n", partition.label.c_str(),
                partitionTypeName(partition.type, partition.subtype), partition.offset,
                humanSize(partition.size).c_str());
  }
  return 0;
}

int doBoot(const std::string& path, int argc, char** argv) {
  EmuOptions options;
  options.instructionLimit = 200000000ULL;
  bool quiet = false;
  size_t registers = 0;
  size_t profile = 0;
  unsigned long registerBlock = 0;
  std::string deviceName;
  std::string capturePath;
  bool bareChip = false;
  bool listTasks = false;
  bool listBlocks = false;
  bool traceBus = false;
  std::vector<std::string> held;
  std::string cardDirectory;
  std::string cardImage;
  std::string touchAt;

  for (int i = 0; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&]() -> std::string { return i + 1 < argc ? argv[++i] : std::string(); };
    if (arg == "--limit") options.instructionLimit = std::stoull(next());
    else if (arg == "--rom-dir") options.romSymbolDir = next();
    else if (arg == "--partitions") options.partitionTablePath = next();
    else if (arg == "--quiet") quiet = true;
    else if (arg == "--registers") registers = std::stoul(next());
    else if (arg == "--seed") options.seed = static_cast<uint32_t>(std::stoul(next()));
    else if (arg == "--registers-at") registerBlock = std::stoul(next(), nullptr, 0);
    else if (arg == "--break") options.breakpoints.push_back(static_cast<uint32_t>(std::stoul(next(), nullptr, 0)));
    else if (arg == "--profile") { profile = std::stoul(next()); options.profile = true; }
    else if (arg == "--device") deviceName = next();
    else if (arg == "--no-board") bareChip = true;
    else if (arg == "--capture") capturePath = next();
    else if (arg == "--tasks") listTasks = true;
    else if (arg == "--blocks") listBlocks = true;
    else if (arg == "--trace-bus") traceBus = true;
    else if (arg == "--hold") held.push_back(next());
    else if (arg == "--card") cardDirectory = next();
    else if (arg == "--card-image") cardImage = next();
    else if (arg == "--touch") touchAt = next();
    else {
      std::cerr << "freeink-emu: unknown option " << arg << "\n";
      return 2;
    }
  }

  // An image is a `.bin` with no board profile in it, so the machine is told
  // which board it is on. Without one it can run the firmware's instructions
  // and nothing else — which is occasionally what you want, and never the
  // default.
  FlashImage probe;
  std::string probeError;
  const std::string chip = probe.load(path, &probeError) ? chipName(probe.app().chip) : "";

  std::string boardName = deviceName;
  const fsim_board_desc* board = nullptr;
  if (!bareChip) {
    if (!boardName.empty()) {
      board = freeink::sim::boards::byName(boardName);
      if (!board) {
        std::cerr << "freeink-emu: no board profile named " << boardName << " (try --boards)\n";
        return 2;
      }
    } else {
      board = freeink::sim::boards::defaultForChip(chip, &boardName);
    }
  }

  Device& device = Device::instance();
  std::unique_ptr<freeink::sim::EmuBoard> bridge;
  if (board) {
    device.describeBoard(*board);
    bridge = std::make_unique<freeink::sim::EmuBoard>(device, chip);
    if (traceBus) {
      bridge->setTrace([](const std::string& line) {
        // Flushed: a bus trace is most useful when the run ends badly, and an
        // unflushed buffer loses exactly the lines that would say why.
        std::printf("[bus] %s\n", line.c_str());
        std::fflush(stdout);
      });
    }
  }

  if (!cardDirectory.empty() || !cardImage.empty()) {
    std::string cardError;
    const bool mounted = cardImage.empty()
                             ? device.mountCard(cardDirectory, 0, &cardError)
                             : device.mountCardImage(cardImage, &cardError);
    if (!mounted) {
      std::cerr << "freeink-emu: " << cardError << "\n";
      return 1;
    }
    std::printf("[emu] card: %.1f MB from %s\n",
                device.card().capacityBytes() / (1024.0 * 1024.0),
                device.card().source().c_str());
  }

  for (const std::string& name : held) {
    const int index = Device::buttonIndexFromName(name);
    if (index < 0) {
      std::cerr << "freeink-emu: no button named " << name << "\n";
      return 2;
    }
    device.setButton(index, true);
  }

  if (!touchAt.empty()) {
    const size_t comma = touchAt.find(',');
    if (comma == std::string::npos) {
      std::cerr << "freeink-emu: --touch wants X,Y\n";
      return 2;
    }
    if (!board || board->touch_controller == FSIM_TOUCH_NONE) {
      // Refused rather than synthesized: a test should not be able to pass
      // against input the hardware cannot produce.
      std::cerr << "freeink-emu: " << boardName << " has no digitizer\n";
      return 2;
    }
    device.setTouch(std::stoi(touchAt.substr(0, comma)), std::stoi(touchAt.substr(comma + 1)), true);
  }

  EmuMachine machine;
  machine.setBoard(bridge.get());
  if (!quiet) {
    // One console here: the tool prints the primary stream, and leaves a
    // secondary USB copy out rather than printing every line twice.
    machine.setConsole([](const char* source, const char* text, size_t length) {
      if (std::strcmp(source, "usb") == 0) return;
      std::fwrite(text, 1, length, stdout);
      std::fflush(stdout);
    });
  }

  std::string error;
  if (!machine.load(path, options, &error)) {
    std::cerr << "freeink-emu: " << error << "\n";
    return 1;
  }

  std::printf("[emu] %s, entry 0x%08X, %zu ROM symbols\n", machine.soc().name, machine.flash().app().entry,
              machine.rom().symbolCount());
  if (board) {
    std::printf("[emu] board %s (%s), %dx%d %s\n", boardName.c_str(), board->board_name,
                board->panel_width, board->panel_height, device.panel().controllerName());
  } else {
    std::printf("[emu] no board: every bus is empty and the panel is not wired\n");
  }
  std::printf("[emu] ---- firmware output ----\n");

  // Simulated time is the emulator's to spend, and the panel's waveforms are
  // keyed to it: a refresh takes as long as the glass takes, and BUSY is held
  // for exactly that. So the machine's clock is driven from the instructions
  // the firmware retired, and a second thread advances it and runs the panel —
  // the same arrangement the daemon uses, without the socket.
  device.clock().setMode(freeink::sim::ClockMode::Virtual);
  // This is the batch tool: no ceiling on how fast simulated time may run.
  // The daemon is the thing a person drives, and it paces itself.
  device.clock().setRate(0.0);
  std::atomic<bool> stop{false};
  std::thread clockThread([&device, &stop]() {
    while (!stop.load()) {
      device.clock().tick();
      device.panel().tick();
      std::this_thread::yield();
    }
  });

  const uint64_t budget = options.instructionLimit ? options.instructionLimit : ~0ULL;
  device.clock().enterRunning();
  uint64_t done = 0;
  while (done < budget && !machine.stopped()) {
    const uint64_t before = machine.timeUs();
    const uint64_t ran = machine.run(std::min<uint64_t>(budget - done, 200000));
    if (ran == 0) break;
    done += ran;
    const uint64_t elapsed = machine.timeUs() - before;
    if (elapsed && device.clock().sleepUs(elapsed) != 0) break;
  }
  device.clock().exitRunning();
  stop = true;
  device.clock().shutdown();
  clockThread.join();

  std::printf("\n[emu] ---- stopped ----\n");
  std::printf("[emu] %s\n", machine.stopReason().c_str());
  std::printf("[emu] %llu instructions, %.3f ms of simulated time, %llu interrupts, %llu exceptions\n",
              static_cast<unsigned long long>(machine.cpu().retired()), machine.timeUs() / 1000.0,
              static_cast<unsigned long long>(machine.cpu().interruptsTaken()),
              static_cast<unsigned long long>(machine.cpu().exceptionsTaken()));

  const std::string firstException = machine.firstExceptionReport();
  if (!firstException.empty()) {
    std::printf("[emu] first exception the firmware took: %s\n", firstException.c_str());
  }

  const std::vector<std::pair<uint32_t, uint32_t>> calls = machine.cpu().recentCalls();
  if (!calls.empty()) {
    std::printf("[emu] last control transfers (most recent last):\n");
    for (const auto& call : calls) {
      std::printf("        0x%08X -> 0x%08X\n", call.first, call.second);
    }
  }

  const std::vector<std::string> gaps = machine.gaps();
  if (!gaps.empty()) {
    std::printf("[emu] gaps reached:\n");
    for (const std::string& gap : gaps) std::printf("        %s\n", gap.c_str());
  }
  if (registerBlock) {
    std::printf("[emu] registers touched at 0x%08lX:\n", registerBlock);
    for (const RegisterAccess& access : machine.registersAt(static_cast<uint32_t>(registerBlock))) {
      std::printf("        0x%08X  %-5s  %8llu  value=0x%08X\n", access.address,
                  access.write ? "write" : "read", static_cast<unsigned long long>(access.count),
                  access.value);
    }
  }
  if (profile) {
    std::printf("[emu] where it spent its time (%llu samples):\n",
                static_cast<unsigned long long>(machine.profileSamples()));
    for (const EmuMachine::HotSpot& spot : machine.hotSpots(profile)) {
      const double share = machine.profileSamples()
                               ? 100.0 * static_cast<double>(spot.samples) / machine.profileSamples()
                               : 0.0;
      std::printf("        %5.1f%%  pc=0x%08X  ra=0x%08X  %s%s\n", share, spot.pc, spot.returnAddress,
                  machine.bus().describe(spot.pc).c_str(),
                  machine.soc().cores > 1 ? (spot.core ? "  [APP]" : "  [PRO]") : "");
    }
  }
  if (registers) {
    std::printf("[emu] most-touched unmodelled registers:\n");
    for (const RegisterAccess& access : machine.unmodelledRegisters(registers)) {
      std::printf("        0x%08X  %-5s  %8llu  last=0x%08X\n", access.address,
                  access.write ? "write" : "read", static_cast<unsigned long long>(access.count),
                  access.value);
    }
  }
  if (listBlocks) {
    std::printf("[emu] peripheral blocks the firmware touched:\n");
    for (const EmuMachine::BlockUse& block : machine.blocksTouched()) {
      std::printf("        0x%08X  %-18s %10llu accesses  %s\n", block.base, block.name.c_str(),
                  static_cast<unsigned long long>(block.accesses),
                  block.modelled ? "modelled" : "recorded only");
    }
  }
  if (listBlocks) {
    std::printf("[emu] interrupts the firmware took:\n");
    for (const auto& source : machine.interruptsUsed()) {
      char where[96] = "";
      if (source.line < 0) {
        std::snprintf(where, sizeof(where), "(the firmware routed nothing to this slot)");
      } else {
        std::snprintf(where, sizeof(where), "on CPU line %d, taken %llu times", source.line,
                      static_cast<unsigned long long>(machine.cpu().interruptsOnLine(source.line)));
      }
      std::printf("        source %-4d raised %10llu times   %s%s\n", source.peripheral,
                  static_cast<unsigned long long>(source.raised), where,
                  source.asserted ? "   still asserted" : "");
    }
  }
  if (listTasks) {
    const std::vector<EmuMachine::Task> tasks = machine.tasks();
    std::printf("[emu] FreeRTOS tasks found in RAM (%zu):\n", tasks.size());
    for (const EmuMachine::Task& task : tasks) {
      std::printf("        %-18s prio=%-3u stack=0x%08X sp=0x%08X  resumes at 0x%08X %s%s\n",
                  task.name.c_str(), task.priority, task.stack, task.topOfStack, task.resumePc,
                  machine.bus().describe(task.resumePc).c_str(), task.running ? "  <- running" : "");
      if (!task.stackTrail.empty()) {
        std::printf("            on its stack:");
        for (uint32_t address : task.stackTrail) std::printf(" 0x%08X", address);
        std::printf("\n");
      }
    }
  }
  if (!capturePath.empty()) {
    const freeink::sim::Frame frame = device.panel().currentFrame();
    std::string pngError;
    if (freeink::sim::writeGrayPng(capturePath, frame.pixels.data(), frame.width, frame.height,
                                   &pngError)) {
      std::printf("[emu] captured %dx%d frame %llu to %s\n", frame.width, frame.height,
                  static_cast<unsigned long long>(frame.seq), capturePath.c_str());
    } else {
      std::printf("[emu] could not write %s: %s\n", capturePath.c_str(), pngError.c_str());
    }
  }

  // A machine that stopped because it was asked to — an ebreak with no
  // handler, or the instruction limit — is not a failure; a fault, a panic or
  // an unimplemented routine is.
  switch (machine.cpu().halted()) {
    case HaltReason::Fault:
    case HaltReason::Panic:
    case HaltReason::Unimplemented:
      return 1;
    default:
      return 0;
  }
}

}  // namespace

int main(int argc, char** argv) {
  if (argc == 2 && std::string(argv[1]) == "boards") {
    for (const auto& entry : freeink::sim::boards::all()) {
      std::printf("  %-12s %-20s %4dx%-4d\n", entry.first.c_str(), entry.second->board_name,
                  entry.second->panel_width, entry.second->panel_height);
    }
    return 0;
  }
  if (argc < 3) {
    printUsage();
    return argc < 2 ? 2 : 0;
  }
  const std::string command = argv[1];
  if (command == "boards") {
    for (const auto& entry : freeink::sim::boards::all()) {
      std::printf("  %-12s %-20s %4dx%-4d\n", entry.first.c_str(), entry.second->board_name,
                  entry.second->panel_width, entry.second->panel_height);
    }
    return 0;
  }
  if (command == "info") return doInfo(argv[2]);
  if (command == "boot") return doBoot(argv[2], argc - 3, argv + 3);
  printUsage();
  return 2;
}
