#pragma once

// FreeInk emulator — one emulated device.
//
// Assembles a chip from a firmware image: picks the SoC descriptor the image's
// header names, builds the address space, attaches the peripherals, loads the
// segments where the bootloader would have put them (RAM directly, cached
// flash through the MMU), and starts the core at the image's entry point.
//
// The second-stage bootloader is *not* executed. It is a program whose whole
// job is to do what this class does — verify the image, set the MMU up and
// jump — and emulating it would only add its own ROM dependencies. The
// emulator enters the app with the machine in the state the bootloader leaves
// it in, which is exactly the contract `call_start_cpu0` is written against.

#include "Board.h"
#include "BoardPeripherals.h"
#include "Bus.h"
#include "Cpu.h"
#include "Image.h"
#include "Peripherals.h"
#include "Rom.h"
#include "Soc.h"

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace freeink::sim::emu {

struct EmuOptions {
  // Seeds the hardware random number generator, so a run reproduces.
  uint32_t seed = 1;
  // Directory holding <chip>.romsyms. Defaults to the emu/rom directory next
  // to the binary.
  std::string romSymbolDir;
  std::string partitionTablePath;
  // Stop after this many instructions with a diagnostic, so a firmware that
  // spins forever fails a test rather than hanging it. Zero means no limit.
  uint64_t instructionLimit = 0;
  // Addresses to stop at, for inspecting a firmware that has no symbols.
  std::vector<uint32_t> breakpoints;
  // Sample the program counter while running. A firmware with no symbols that
  // "just sits there" is otherwise opaque; the histogram says which handful of
  // addresses it is sitting in, and the caller each was reached from.
  bool profile = false;
};

class EmuMachine {
 public:
  EmuMachine();
  ~EmuMachine();

  // Loads a firmware image and brings the machine up to its entry point.
  bool load(const std::string& imagePath, const EmuOptions& options, std::string* error);

  // Runs at most `budget` instructions. Returns the number retired; check
  // `stopped()` afterwards.
  uint64_t run(uint64_t budget);

  bool stopped() const;
  std::string stopReason() const;
  // True when the machine stopped because the firmware put the chip to sleep
  // rather than because anything went wrong. A device that sleeps is working;
  // what happens next is a question for whatever is driving it.
  bool asleep() const { return asleep_; }
  // What the firmware asked the wake-up timer for, in microseconds, or zero
  // when it armed no timer and is waiting on a pad.
  uint64_t sleepRequestUs() const { return sleepRequestUs_; }
  // Simulated microseconds since reset, charged at the chip's clock rate plus
  // whatever the firmware explicitly waited for.
  uint64_t timeUs() const { return timeUs_; }
  void advanceUs(uint64_t us) { timeUs_ += us; }

  const FlashImage& flash() const { return flash_; }
  const SocDesc& soc() const { return *soc_; }
  Bus& bus() { return *bus_; }
  Cpu& cpu() { return *cpu_; }
  const Cpu& cpu() const { return *cpu_; }
  // The second core, on a part that has one. It exists from reset but does
  // nothing until the firmware hands the ROM its entry point — which an IDF
  // app does during startup, and then waits for it to check in.
  Cpu* appCpu() { return cpu1_.get(); }
  const Cpu* appCpu() const { return cpu1_.get(); }
  Rom& rom() { return *rom_; }

  // What is wired to the chip's pins. Set before load(), because the pin-facing
  // peripherals bind to it as they are attached. Left unset, the machine models
  // a bare chip on a bench: every bus answers nothing, which is what a probe
  // for an absent peripheral should see.
  void setBoard(BoardBridge* board) { board_ = board; }
  BoardBridge* board() const { return board_; }

  // Console output. `source` names where the bytes came out of — "rom" for the
  // ROM's own printf, "uart0", "usb". A chip with both a UART and a USB serial
  // console enabled writes every line to both, exactly as the device does, so
  // keeping the source lets the reader filter rather than see everything twice.
  void setConsole(std::function<void(const char* source, const char*, size_t)> sink) {
    console_ = std::move(sink);
  }

  // A description of the first CPU exception the firmware took, with the
  // control transfers that led to it, or an empty string if it took none.
  std::string firstExceptionReport() const;

  // What the machine was asked for and could not provide, for the report a
  // failed boot prints: unimplemented ROM routines, unmapped accesses.
  std::vector<std::string> gaps() const;
  // Peripheral registers the firmware touched that no model claims, most-read
  // first. This is the list that says what to build next.
  std::vector<RegisterAccess> unmodelledRegisters(size_t limit) const;
  // Where the firmware spent its instructions, hottest first. Sampled, so the
  // counts are proportions rather than exact instruction tallies.
  struct HotSpot {
    uint32_t pc;
    uint32_t returnAddress;
    int core;
    uint64_t samples;
  };
  std::vector<HotSpot> hotSpots(size_t limit) const;
  uint64_t profileSamples() const { return profileSamples_; }

  // The firmware's FreeRTOS tasks, found by looking for their control blocks
  // in RAM.
  //
  // A firmware image has no symbols, so "it boots and then sits there" is
  // otherwise an opaque answer: the profile shows the scheduler, which is true
  // of every idle system and says nothing. What actually matters is *which
  // task* is running and which are blocked — and a task control block is
  // recognisable without symbols, because it carries its own name a fixed
  // distance after a pointer to its own stack.
  //
  // This is a heuristic over memory the firmware owns, so it is a diagnostic
  // and not a fact: a plausible-looking pair of words elsewhere in RAM will be
  // listed too. It is still the difference between "somewhere in FreeRTOS" and
  // "the task called `loopTask` is the one spinning".
  struct Task {
    std::string name;
    uint32_t tcb;
    uint32_t stack;
    uint32_t topOfStack;
    // Where the task will resume: the saved program counter at the top of its
    // stack. For a task that is blocked, this is the line the firmware is
    // waiting on — the single most useful number about a firmware that boots
    // and then does nothing.
    uint32_t resumePc;
    uint32_t priority;
    bool running;  // its stack is the one the current stack pointer is in
    // Code addresses still sitting on the task's stack, innermost first. With
    // no symbols and no frame pointers this is a scan rather than an unwind,
    // so it is a set of candidates and not a call stack — but it is what says
    // *which* of the firmware's own routines is the one blocked, which the
    // resume address alone (always the same scheduler primitive) never does.
    std::vector<uint32_t> stackTrail;
  };
  std::vector<Task> tasks() const;
  // Every register the firmware touched in one peripheral block, in address
  // order — how a driver's configuration of a block is inspected after a run.
  std::vector<RegisterAccess> registersAt(uint32_t base) const;
  // The peripheral blocks that have models, by base address and name.
  std::vector<std::pair<uint32_t, std::string>> modelledBlocks() const;
  // Every peripheral block the firmware touched, busiest first, with how many
  // register accesses it made and whether the block has a real model behind
  // it. The map of what a firmware actually uses.
  struct BlockUse {
    uint32_t base;
    std::string name;
    uint64_t accesses;
    bool modelled;
  };
  std::vector<BlockUse> blocksTouched() const;
  // Which interrupts the firmware actually took, busiest first.
  std::vector<InterruptMatrixDevice::SourceUse> interruptsUsed() const;

 private:
  bool buildMemoryMap(std::string* error);
  void attachPeripherals();
  bool loadSegments(std::string* error);

  FlashImage flash_;
  const SocDesc* soc_ = nullptr;
  std::unique_ptr<Bus> bus_;
  std::unique_ptr<Rom> rom_;
  std::unique_ptr<Cpu> cpu_;
  std::unique_ptr<Cpu> cpu1_;
  InterruptMatrixDevice* matrix_ = nullptr;
  SystimerDevice* systimer_ = nullptr;
  std::vector<GenericPeripheral*> generic_;
  // Every recording device, modelled or not, so a register trace can be taken
  // from any block.
  std::vector<std::pair<uint32_t, GenericPeripheral*>> recording_;

  std::function<void(const char*, const char*, size_t)> console_;
  bool profiling_ = false;
  uint64_t profileSamples_ = 0;
  std::map<uint64_t, HotSpot> profile_;  // (core<<32 | pc) -> sample
  BoardBridge* board_ = nullptr;
  GpioDevice* gpio_ = nullptr;
  GdmaDevice* gdma_ = nullptr;
  uint64_t timeUs_ = 0;
  bool asleep_ = false;
  uint64_t sleepRequestUs_ = 0;
  // Which core ran the last slice, so the two get the machine in turn.
  int nextCore_ = 0;
  uint64_t limit_ = 0;
  uint32_t seed_ = 1;
  std::string loadError_;
};

}  // namespace freeink::sim::emu
