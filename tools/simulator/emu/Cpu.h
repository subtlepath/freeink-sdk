#pragma once

// FreeInk emulator — what the ROM layer and the machine need from a CPU,
// without knowing which instruction set is underneath.
//
// The two cores here are very different animals (RV32IMC with a flat register
// file; Xtensa LX7 with register windows), but a ROM routine only ever needs
// the calling convention: arguments in, a result out, and a return to the
// caller. Expressing that as one interface is what lets Rom.cpp hold a single
// table of native implementations that serves both chips.

#include "Bus.h"

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace freeink::sim::emu {

enum class HaltReason {
  Running,
  Fault,          // a bus fault or an unhandled CPU exception
  Breakpoint,     // ebreak / break with no debugger attached
  WaitingForInterrupt,
  Panic,          // the firmware called a ROM abort/reset routine
  Unimplemented,  // an instruction or ROM routine the emulator does not model
  Stopped,        // asked to stop from outside
};

class Cpu {
 public:
  virtual ~Cpu() = default;

  virtual const char* isaName() const = 0;
  virtual void reset(uint32_t entry) = 0;
  // Runs at most `budget` instructions, returning how many it retired. Stops
  // early on a fault, a halt, or a wait-for-interrupt.
  virtual uint64_t run(uint64_t budget) = 0;

  virtual uint32_t pc() const = 0;
  virtual void setPc(uint32_t pc) = 0;
  virtual uint64_t retired() const = 0;
  virtual HaltReason halted() const = 0;
  virtual const std::string& haltDetail() const = 0;
  virtual void stop(HaltReason reason, const std::string& detail) = 0;
  // Stops the machine when execution reaches this address, with the registers
  // intact. The debugging primitive the emulator would otherwise lack: a
  // firmware image has no symbols, so being able to stop at an address found
  // in a disassembly is how a wrong result gets traced to its cause.
  virtual void addBreakpoint(uint32_t address) = 0;
  virtual void resume() = 0;

  // Interrupt lines, as driven by the interrupt matrix.
  virtual void setInterruptPending(int line, bool pending) = 0;
  virtual bool interruptsEnabled() const = 0;
  // Whether anything is pending that would wake a core sitting in a wait
  // instruction. The machine asks before resuming an idle core, so an idle
  // CPU costs no host time and does not run past its own wait.
  virtual bool hasPendingInterrupt() { return false; }
  // The ROM's own interrupt-masking calls — the `_xtos_` family on Xtensa —
  // work on whatever register the core masks with. They are expressed here
  // rather than reached into, so the ROM table stays one table.
  //
  // `maskInterruptsAbove` returns whatever `restoreInterruptMask` needs to
  // undo it; the caller treats it as opaque, which is exactly the contract
  // between _xtos_set_intlevel and _xtos_restore_intlevel.
  virtual uint32_t maskInterruptsAbove(uint32_t /*level*/) { return 0; }
  virtual void restoreInterruptMask(uint32_t /*state*/) {}
  virtual uint32_t interruptEnableMask() const { return 0; }
  virtual void setInterruptEnableMask(uint32_t /*mask*/) {}

  // Priority and threshold exist on the C3's matrix and not on the S3's, so
  // they default to doing nothing rather than forcing every core to carry them.
  virtual void setInterruptPriority(int /*line*/, uint32_t /*priority*/) {}
  virtual void setInterruptThreshold(uint32_t /*threshold*/) {}

  // ── Calling convention, for ROM interception ───────────────────────────────
  virtual uint32_t arg(int index) const = 0;
  virtual void setArg(int index, uint32_t value) = 0;
  virtual void setReturn(uint32_t value) = 0;
  virtual void setReturn64(uint64_t value) = 0;
  virtual uint32_t stackPointer() const = 0;
  // Where the current function will return to, as the ABI holds it (ra on
  // RISC-V, a0 on Xtensa). The profiler samples it so a hot leaf routine is
  // reported with the caller that keeps calling it, which is usually the more
  // useful half of the answer in a firmware with no symbols.
  virtual uint32_t returnAddress() const { return 0; }
  // Returns to whoever called the ROM routine.
  virtual void returnToCaller() = 0;

  // Calls a function *in the firmware* and returns its result. ROM routines
  // that take a callback — qsort's comparator, the console putc hook — cannot
  // be emulated without this: the code they call is the application's, and
  // there is nowhere else for it to run. The core saves its state, runs the
  // call to completion against a sentinel return address, and restores.
  // Returns false if the firmware faulted or ran away inside the callback.
  virtual bool callGuest(uint32_t function, const uint32_t* args, int argCount, uint32_t* result) = 0;

  // How many interrupts and exceptions the core has taken. A firmware that
  // sits idle with zero interrupts is waiting for a timer that never fires,
  // which is a very different problem from one that is busy computing.
  virtual uint64_t interruptsTaken() const = 0;
  virtual uint64_t exceptionsTaken() const = 0;
  // How many of them arrived on each CPU interrupt line. A firmware that is
  // busy but making no progress is usually taking one interrupt over and over,
  // and the line is the half of the answer the peripheral side cannot give:
  // several peripherals share a line, and a line nothing clears is taken again
  // the instant the handler returns.
  virtual uint64_t interruptsOnLine(int /*line*/) const { return 0; }

  // A one-line register dump for fault reports.
  virtual std::string describeState() const = 0;
  // The most recent calls and indirect jumps, oldest first, as (from, to).
  // When a firmware ends up executing something that is not code, this is the
  // only thing that says how it got there.
  virtual std::vector<std::pair<uint32_t, uint32_t>> recentCalls() const = 0;
};

}  // namespace freeink::sim::emu
