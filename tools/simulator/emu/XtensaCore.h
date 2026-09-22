#pragma once

// FreeInk emulator — Xtensa LX7 core, as fitted to the ESP32-S3.
//
// This is the other half of the "can it run a device image?" question: the X3
// and the C3-based X4 are RISC-V, but the X4 Pro and the X4 Classic are
// ESP32-S3, and an S3 image is Xtensa from its first instruction.
//
// Three things make this core different in kind from `RiscvCore`, rather than
// just different in encoding:
//
//   * **Registers move.** The LX7 has 64 physical address registers and shows
//     16 of them at a time through a window. A call does not save registers,
//     it rotates the window; when the window wraps onto a frame that is still
//     live the hardware raises an exception and the *firmware* spills that
//     frame to its stack. So a correct core cannot simply model a0-a15 — it
//     has to model WindowBase/WindowStart and raise overflow and underflow at
//     exactly the right moment, because the handlers that fix it up are the
//     application's own code, sitting at VECBASE.
//
//   * **Exceptions are the normal path, not the failure path.** Window spill,
//     FPU enable, syscalls and every level-1 interrupt arrive through the same
//     vector table. Dispatching them to the firmware's vectors is what makes
//     FreeRTOS's context switch work at all.
//
//   * **Interrupts have fixed levels.** The S3's matrix routes a peripheral to
//     one of 32 CPU interrupts, and the *hardware* decides which of the six
//     priority levels that interrupt belongs to — the table is part of the
//     core's configuration, not something software programs. `kInterruptLevel`
//     below mirrors XCHAL_INTn_LEVEL from ESP-IDF's esp32s3 core-isa.h; a
//     wrong entry sends an interrupt to a vector the firmware did not install
//     a handler at.
//
// Unmodelled by choice: MAC16 (no compiler emits it here), the MMU/TLB options
// the S3 does not fit, and the debug/OCD registers beyond storing them.

#include "Cpu.h"

#include <array>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace freeink::sim::emu {

class Rom;

class XtensaCore final : public Cpu {
 public:
  // `coreId` is the PRID the firmware reads to tell which CPU it is on. The S3
  // is dual-core and an IDF app will not finish booting until the second one
  // reports for duty, so both are real cores here, not a stub.
  XtensaCore(Bus& bus, const SocDesc& soc, Rom* rom, int coreId = 0);

  const char* isaName() const override { return "xtensa lx7"; }
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
  bool hasPendingInterrupt() override {
    refreshTimerInterrupts();
    return (interrupt_ & intenable_) != 0;
  }
  uint32_t maskInterruptsAbove(uint32_t level) override {
    const uint32_t previous = ps_;
    ps_ = (ps_ & ~kPsIntLevel) | (level & kPsIntLevel);
    return previous;
  }
  void restoreInterruptMask(uint32_t state) override { ps_ = state; }
  uint32_t interruptEnableMask() const override { return intenable_; }
  void setInterruptEnableMask(uint32_t mask) override { intenable_ = mask; }

  uint32_t arg(int index) const override;
  void setArg(int index, uint32_t value) override;
  void setReturn(uint32_t value) override;
  void setReturn64(uint64_t value) override;
  uint32_t stackPointer() const override { return ar(1); }
  // a0 carries only the low 30 bits of the return address; the top two come
  // from the PC of the call, which is the window the callee is running in.
  uint32_t returnAddress() const override { return (pc() & 0xC0000000u) | (ar(0) & 0x3FFFFFFFu); }
  void returnToCaller() override;
  bool callGuest(uint32_t function, const uint32_t* args, int argCount, uint32_t* result) override;

  uint64_t interruptsTaken() const override { return interrupts_; }
  uint64_t exceptionsTaken() const override { return exceptions_; }
  std::string describeState() const override;
  std::vector<std::pair<uint32_t, uint32_t>> recentCalls() const override;

  // The first exception the firmware took, kept for the same reason the RISC-V
  // core keeps one: the panic handler that runs afterwards overwrites the
  // evidence. Window overflow and underflow are excluded — they are hundreds
  // of times a second and entirely normal.
  struct FirstException {
    bool seen = false;
    uint32_t cause = 0;
    uint32_t pc = 0;
    uint32_t vaddr = 0;
    std::vector<std::pair<uint32_t, uint32_t>> trace;
  };
  const FirstException& firstException() const { return firstException_; }
  static const char* causeName(uint32_t cause);

  // A core that has not been started yet retires nothing and reports no fault.
  // The APP CPU sits like this until the firmware hands the ROM its entry
  // point, which is exactly how the silicon behaves.
  void park() { parked_ = true; }
  bool parked() const { return parked_; }
  void start(uint32_t entry);

  int coreId() const { return coreId_; }
  // The window as the firmware sees it: a0-a15 of the current frame.
  uint32_t ar(int index) const { return ar_[(windowBase_ * 4 + index) & 63]; }
  void setAr(int index, uint32_t value) { ar_[(windowBase_ * 4 + index) & 63] = value; }

  // Level of each of the 32 CPU interrupts, from XCHAL_INTn_LEVEL.
  static const uint8_t kInterruptLevel[32];
  // Interrupts whose bit is set by an edge and cleared by software, from
  // XCHAL_INTTYPE_MASK_EXTERN_EDGE. The rest follow the line.
  static constexpr uint32_t kEdgeTriggered = 0x50400400u;
  static constexpr uint32_t kSoftwareInterrupts = 0x20000080u;
  static constexpr uint32_t kTimerInterrupts = 0x00018040u;

 private:
  // PS fields, as bit positions rather than a struct: the firmware reads and
  // writes PS as a word constantly (RSIL, RFI, the context switch), so keeping
  // it packed avoids a pack/unpack on every one of those.
  static constexpr uint32_t kPsIntLevel = 0x0000000Fu;
  static constexpr uint32_t kPsExcm = 1u << 4;
  static constexpr uint32_t kPsUm = 1u << 5;
  static constexpr uint32_t kPsRing = 3u << 6;
  static constexpr uint32_t kPsOwbShift = 8;
  static constexpr uint32_t kPsOwb = 0xFu << kPsOwbShift;
  static constexpr uint32_t kPsCallIncShift = 16;
  static constexpr uint32_t kPsCallInc = 3u << kPsCallIncShift;
  static constexpr uint32_t kPsWoe = 1u << 18;

  static constexpr int kExcmLevel = 3;   // XCHAL_EXCM_LEVEL
  static constexpr int kNumLevels = 7;   // including NMI
  static constexpr int kWindows = 16;    // 64 address registers / 4

  bool stepOnce();
  void execute();
  void executeNarrow(uint32_t instruction);
  void executeWide(uint32_t instruction);

  // ── Exceptions and interrupts ──────────────────────────────────────────────
  void exception(uint32_t cause, uint32_t vaddr = 0);
  void windowException(uint32_t vectorOffset, uint32_t oldBase);
  bool serviceInterrupt();
  void illegal(uint32_t instruction);
  void unimplemented(const char* what, uint32_t instruction);
  void noteException(uint32_t cause, uint32_t vaddr);

  // ── The register window ────────────────────────────────────────────────────
  // `windowStart_` has one bit per 4-register window position; a set bit means
  // "a live frame starts here". Everything about spilling follows from it.
  void rotateWindow(int by);
  // WindowStart as seen from just above the current frame: bit k is the window
  // position WindowBase + 1 + k. Every window question is asked in these terms.
  uint32_t rotatedWindowStart() const;
  // How many further window positions the current frame may reach into before
  // it runs onto a live frame — 0 to 3, since the window shows 16 registers.
  int windowAvailable() const;
  // The check the architecture makes on *every* register operand: an access to
  // a register beyond what the frame owns spills the oldest live frame first.
  //
  // This is why it is an operand check and not something ENTRY does. A caller
  // writes its callee's registers — the return address and the arguments —
  // before the callee's ENTRY ever runs. If the spill waited for ENTRY, those
  // writes would already have landed on top of the frame about to be saved.
  int highestRegister(uint32_t instruction) const;
  void windowOverflow();

  uint32_t readSpecial(uint32_t which);
  void writeSpecial(uint32_t which, uint32_t value);
  uint32_t readUser(uint32_t which);
  void writeUser(uint32_t which, uint32_t value);
  uint32_t ps() const { return ps_; }
  void setPs(uint32_t value);
  uint32_t ccount() const;
  void refreshTimerInterrupts();
  bool coprocessorReady(int coprocessor);

  void noteTransfer(uint32_t from, uint32_t to) {
    calls_[callCursor_ % kCallRing] = {from, to};
    ++callCursor_;
  }

  Bus& bus_;
  const SocDesc& soc_;
  Rom* rom_;
  int coreId_;

  uint32_t ar_[64] = {};
  uint32_t pc_ = 0;
  uint64_t retired_ = 0;

  uint32_t ps_ = 0;
  uint32_t windowBase_ = 0;
  uint32_t windowStart_ = 1;
  uint32_t sar_ = 0;
  uint32_t br_ = 0;
  uint32_t litbase_ = 0;
  uint32_t scompare1_ = 0;
  uint32_t lbeg_ = 0, lend_ = 0, lcount_ = 0;
  uint32_t vecbase_ = 0;
  uint32_t exccause_ = 0;
  uint32_t excvaddr_ = 0;
  uint32_t depc_ = 0;
  uint32_t cpenable_ = 0;
  uint32_t atomctl_ = 0;
  uint32_t memctl_ = 0;
  uint32_t threadptr_ = 0;
  uint32_t fcr_ = 0, fsr_ = 0;
  uint32_t fr_[16] = {};
  std::array<uint32_t, 8> epc_{};
  std::array<uint32_t, 8> eps_{};
  std::array<uint32_t, 8> excsave_{};
  std::array<uint32_t, 4> misc_{};
  std::array<uint32_t, 3> ccompare_{};
  std::array<uint32_t, 256> otherSpecial_{};

  // INTERRUPT as the firmware reads it, plus the state behind it: a
  // level-triggered line's bit follows the line, an edge-triggered one latches.
  uint32_t interrupt_ = 0;
  uint32_t intenable_ = 0;
  uint32_t externalLines_ = 0;
  uint64_t cycleBase_ = 0;

  static constexpr size_t kCallRing = 64;
  std::array<std::pair<uint32_t, uint32_t>, kCallRing> calls_{};
  size_t callCursor_ = 0;

  std::vector<uint32_t> breakpoints_;
  uint64_t interrupts_ = 0;
  uint64_t exceptions_ = 0;
  FirstException firstException_;
  HaltReason halt_ = HaltReason::Running;
  std::string haltDetail_;
  bool parked_ = false;
  // Set by the instruction just executed when it wrote the PC itself, so the
  // zero-overhead loop does not fire on a branch that happens to land on LEND.
  bool tookBranch_ = false;
  // The window increment of the call that entered mask ROM, so an intercepted
  // ROM routine reads its arguments from the registers the caller put them in.
  int romCallInc_ = 0;
  int callGuestDepth_ = 0;
};

}  // namespace freeink::sim::emu
