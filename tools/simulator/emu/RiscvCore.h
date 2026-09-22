#pragma once

// FreeInk emulator — RV32IMC core, as fitted to the ESP32-C3.
//
// Machine mode only, which is all an ESP-IDF app uses. The parts that are
// specific to the chip rather than to RISC-V are the interrupt model: the C3
// wires its interrupt matrix straight into `mie`/`mip` as 31 individually
// maskable lines, dispatches them through a vectored `mtvec`, and gates them
// with a priority threshold register that lives in the matrix rather than in a
// CSR. Everything else is stock.

#include "Cpu.h"

#include <array>
#include <utility>
#include <vector>
#include <cstdint>
#include <string>

namespace freeink::sim::emu {

class Rom;

class RiscvCore final : public Cpu {
 public:
  RiscvCore(Bus& bus, const SocDesc& soc, Rom* rom);

  const char* isaName() const override { return "rv32imc"; }
  void reset(uint32_t entry) override;
  uint64_t run(uint64_t budget) override;

  uint32_t pc() const override { return pc_; }
  void setPc(uint32_t pc) override { pc_ = pc; }
  uint64_t retired() const override { return retired_; }
  HaltReason halted() const override { return halt_; }
  const std::string& haltDetail() const override { return haltDetail_; }
  void stop(HaltReason reason, const std::string& detail) override;
  void addBreakpoint(uint32_t address) override { breakpoints_.push_back(address); }
  void resume() override;

  void setInterruptPending(int line, bool pending) override;
  bool interruptsEnabled() const override;
  bool hasPendingInterrupt() override { return (mie_ & mip_) != 0; }
  // The interrupt matrix owns the threshold; lines below it stay masked.
  void setInterruptThreshold(uint32_t threshold) override { threshold_ = threshold; }
  void setInterruptPriority(int line, uint32_t priority) override;

  uint32_t arg(int index) const override;
  void setArg(int index, uint32_t value) override;
  void setReturn(uint32_t value) override;
  void setReturn64(uint64_t value) override;
  uint32_t stackPointer() const override { return x_[2]; }
  uint32_t returnAddress() const override { return x_[1]; }
  void returnToCaller() override { pc_ = x_[1]; }
  bool callGuest(uint32_t function, const uint32_t* args, int argCount, uint32_t* result) override;

  uint64_t interruptsTaken() const override { return interrupts_; }
  uint64_t interruptsOnLine(int line) const override {
    return (line >= 0 && line < 32) ? lineCounts_[line] : 0;
  }
  uint64_t exceptionsTaken() const override { return exceptions_; }
  std::string describeState() const override;
  std::vector<std::pair<uint32_t, uint32_t>> recentCalls() const override;

  // The first CPU exception the firmware took, with the control transfers that
  // led to it. A firmware that crashes usually then runs its own panic handler,
  // which overwrites the trace; capturing it here keeps the thing that actually
  // went wrong rather than the tidy-up afterwards.
  struct FirstException {
    bool seen = false;
    uint32_t cause = 0;
    uint32_t pc = 0;
    uint32_t tval = 0;
    std::vector<std::pair<uint32_t, uint32_t>> trace;
  };
  const FirstException& firstException() const { return firstException_; }

  uint32_t reg(int index) const { return index == 0 ? 0 : x_[index]; }
  void setReg(int index, uint32_t value) {
    if (index != 0) x_[index] = value;
  }

 private:
  // One instruction, or one intercepted ROM call. Returns false when the core
  // has stopped or the bus has faulted.
  bool stepOnce();
  void execute(uint32_t instruction, uint32_t length);
  void executeCompressed(uint16_t instruction);
  void trap(uint32_t cause, uint32_t tval);
  bool serviceInterrupt();
  uint32_t readCsr(uint32_t csr, bool* ok);
  void writeCsr(uint32_t csr, uint32_t value, bool* ok);
  void illegal(uint32_t instruction);

  Bus& bus_;
  const SocDesc& soc_;
  Rom* rom_;

  uint32_t x_[32] = {};
  uint32_t pc_ = 0;
  uint64_t retired_ = 0;

  // Machine-mode CSRs that actually do something. Everything else the C3
  // exposes (performance counters, PMP, vendor CSRs) is stored and returned
  // unchanged: an app writes them at startup and never depends on the effect.
  uint32_t mstatus_ = 0;
  uint32_t mtvec_ = 0;
  uint32_t mepc_ = 0;
  uint32_t mcause_ = 0;
  uint32_t mtval_ = 0;
  uint32_t mscratch_ = 0;
  uint32_t mie_ = 0;
  uint32_t mip_ = 0;
  std::array<uint32_t, 32> intPriority_{};
  uint32_t threshold_ = 0;
  uint32_t cycleOffset_ = 0;
  std::array<uint32_t, 4096> otherCsr_{};

  // A short history of control transfers, so a fault inside something that is
  // not code can be traced back to the call that went wrong.
  static constexpr size_t kCallRing = 64;
  std::array<std::pair<uint32_t, uint32_t>, kCallRing> calls_{};
  size_t callCursor_ = 0;
  void noteTransfer(uint32_t from, uint32_t to) {
    calls_[callCursor_ % kCallRing] = {from, to};
    ++callCursor_;
  }

  std::vector<uint32_t> breakpoints_;
  uint64_t interrupts_ = 0;
  std::array<uint64_t, 32> lineCounts_{};
  uint64_t exceptions_ = 0;
  FirstException firstException_;
  HaltReason halt_ = HaltReason::Running;
  std::string haltDetail_;
};

}  // namespace freeink::sim::emu
