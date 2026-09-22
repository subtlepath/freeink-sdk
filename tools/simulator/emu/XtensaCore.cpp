// FreeInk emulator — Xtensa LX7 core, as fitted to the ESP32-S3.

#include "XtensaCore.h"

#include "Rom.h"

#include <cmath>
#include <cstdio>
#include <cstring>

namespace freeink::sim::emu {
namespace {

// EXCCAUSE values an ESP32-S3 app can actually produce. The names are the
// architecture's own, so a fault report can be matched against the firmware's
// panic output without a decoder ring.
constexpr uint32_t kIllegalInstruction = 0;
constexpr uint32_t kSyscall = 1;
constexpr uint32_t kInstructionFetchError = 2;
constexpr uint32_t kLoadStoreError = 3;
constexpr uint32_t kLevel1Interrupt = 4;
constexpr uint32_t kAlloca = 5;
constexpr uint32_t kIntegerDivideByZero = 6;
constexpr uint32_t kPrivileged = 8;
constexpr uint32_t kLoadStoreAlignment = 9;
constexpr uint32_t kCoprocessor0Disabled = 32;

// Vector offsets from VECBASE, from XCHAL_*_VECOFS in esp32s3's core-isa.h.
constexpr uint32_t kWindowOverflow4 = 0x000;
constexpr uint32_t kWindowUnderflow4 = 0x040;
constexpr uint32_t kWindowOverflow8 = 0x080;
constexpr uint32_t kWindowUnderflow8 = 0x0C0;
constexpr uint32_t kWindowOverflow12 = 0x100;
constexpr uint32_t kWindowUnderflow12 = 0x140;
constexpr uint32_t kLevel2Vector = 0x180;  // levels 3..7 follow at 0x40 apart
constexpr uint32_t kKernelVector = 0x300;
constexpr uint32_t kUserVector = 0x340;
constexpr uint32_t kDoubleVector = 0x3C0;

// Special register numbers. Only the ones with behaviour are named; the rest
// are stored and read back, which is what an app that saves and restores a
// register it never otherwise uses actually needs.
enum Special {
  SR_LBEG = 0,
  SR_LEND = 1,
  SR_LCOUNT = 2,
  SR_SAR = 3,
  SR_BR = 4,
  SR_LITBASE = 5,
  SR_SCOMPARE1 = 12,
  SR_WINDOWBASE = 72,
  SR_WINDOWSTART = 73,
  SR_MEMCTL = 97,
  SR_ATOMCTL = 99,
  SR_EPC1 = 177,
  SR_DEPC = 192,
  SR_EPS2 = 194,
  SR_EXCSAVE1 = 209,
  SR_CPENABLE = 224,
  SR_INTERRUPT = 226,  // INTSET on write
  SR_INTCLEAR = 227,
  SR_INTENABLE = 228,
  SR_PS = 230,
  SR_VECBASE = 231,
  SR_EXCCAUSE = 232,
  SR_DEBUGCAUSE = 233,
  SR_CCOUNT = 234,
  SR_PRID = 235,
  SR_EXCVADDR = 238,
  SR_CCOMPARE0 = 240,
  SR_MISC0 = 244,
};

// User registers reachable with RUR/WUR.
enum User {
  UR_THREADPTR = 231,
  UR_FCR = 232,
  UR_FSR = 233,
};

// The immediates BEQI and friends compare against, from the B4CONST table.
constexpr int32_t kB4Const[16] = {-1, 1, 2, 3, 4, 5, 6, 7, 8, 10, 12, 16, 32, 64, 128, 256};
constexpr uint32_t kB4ConstU[16] = {32768, 65536, 2, 3, 4, 5, 6, 7, 8, 10, 12, 16, 32, 64, 128, 256};

int32_t signExtend(uint32_t value, int bits) {
  const uint32_t sign = 1u << (bits - 1);
  return static_cast<int32_t>((value ^ sign) - sign);
}

float bitsToFloat(uint32_t bits) {
  float value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

uint32_t floatToBits(float value) {
  uint32_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

}  // namespace

// XCHAL_INT<n>_LEVEL for esp32s3. Interrupt 14 is the NMI and 6/15/16 are the
// core's own timers; everything else is a matrix line.
const uint8_t XtensaCore::kInterruptLevel[32] = {
    1, 1, 1, 1, 1, 1, 1, 1,  //  0- 7
    1, 1, 1, 3, 1, 1, 7, 3,  //  8-15
    5, 1, 1, 2, 2, 2, 3, 3,  // 16-23
    4, 4, 5, 3, 4, 3, 4, 5,  // 24-31
};

XtensaCore::XtensaCore(Bus& bus, const SocDesc& soc, Rom* rom, int coreId)
    : bus_(bus), soc_(soc), rom_(rom), coreId_(coreId) {
  reset(0);
}

void XtensaCore::reset(uint32_t entry) {
  std::memset(ar_, 0, sizeof(ar_));
  pc_ = entry;
  retired_ = 0;
  // Not the state at reset, but the state the second-stage bootloader hands
  // over in — which is the contract an application's entry point is compiled
  // against, and the same choice the rest of the emulator makes by entering
  // the app directly. Windowed calls on (its first instruction is ENTRY),
  // user mode on, and interrupts masked up to the level the app will lower
  // itself once it has installed its vectors.
  // CALLINC of 2 completes the picture: the application was reached by a
  // CALL8, and its first instruction is the ENTRY that rotates onto it.
  ps_ = kPsWoe | kPsUm | kExcmLevel | (2u << kPsCallIncShift);
  windowBase_ = 0;
  windowStart_ = 1;
  // VECBASE still points at the ROM's vectors; the app moves it to its own
  // within a handful of instructions.
  vecbase_ = soc_.rom.base;
  interrupt_ = 0;
  intenable_ = 0;
  externalLines_ = 0;
  cycleBase_ = 0;
  lcount_ = 0;
  cpenable_ = 0;
  halt_ = HaltReason::Running;
  haltDetail_.clear();
  parked_ = false;
  firstException_ = FirstException{};
  // The ROM leaves the app on its own stack; starting anywhere else puts the
  // earliest frames on top of memory the ROM owns.
  //
  // What is set up here is a caller's frame, not just a stack pointer, because
  // the application is entered as the bootloader would enter it: with a CALL8
  // behind it. That frame is what the window spill handler needs when the
  // register file eventually fills and the application's own first frame has
  // to be written out — the handler reads the caller's stack pointer from the
  // frame's save area, and with no caller it reads zero and stores through it.
  // Every later frame's save area is filled in by the spill itself, so this
  // one frame is all the emulator has to fabricate.
  if (soc_.romStack.size) {
    // Each core gets its own half of the ROM stack region. They overlap for
    // only a few instructions on real silicon — the APP CPU's entry point
    // switches to a stack of its own almost immediately — but "only a few
    // instructions" is not a thing to model by letting two cores write the
    // same words.
    const uint32_t perCore = soc_.cores > 1 ? soc_.romStack.size / 2 : soc_.romStack.size;
    const uint32_t stackTop =
        soc_.romStack.base + soc_.romStack.size - static_cast<uint32_t>(coreId_) * perCore;
    const uint32_t callerStack = stackTop - 16;
    const uint32_t stack = callerStack - 64;
    setAr(1, stack);
    bus_.write32(stack - 12, callerStack);
    bus_.write32(stack - 16, 0);
    // The link register a CALL8 would have left, with its window increment in
    // the top two bits and an address the application never returns to.
    setAr(8, 2u << 30);
  }
}

void XtensaCore::start(uint32_t entry) {
  reset(entry);
  parked_ = false;
}

void XtensaCore::stop(HaltReason reason, const std::string& detail) {
  if (halt_ != HaltReason::Running && reason != HaltReason::Stopped) return;
  halt_ = reason;
  haltDetail_ = detail;
}

void XtensaCore::resume() {
  if (halt_ == HaltReason::WaitingForInterrupt || halt_ == HaltReason::Breakpoint ||
      halt_ == HaltReason::Stopped) {
    halt_ = HaltReason::Running;
    haltDetail_.clear();
  }
}

// ── The register window ──────────────────────────────────────────────────────

void XtensaCore::rotateWindow(int by) {
  windowBase_ = static_cast<uint32_t>((static_cast<int>(windowBase_) + by) & (kWindows - 1));
}

uint32_t XtensaCore::rotatedWindowStart() const {
  const uint32_t doubled = windowStart_ | (windowStart_ << kWindows);
  return (doubled >> (((windowBase_ + 1) & (kWindows - 1)))) & 0xFFFFu;
}

int XtensaCore::windowAvailable() const {
  // The distance to the next live frame, capped at 3: the window shows four
  // positions, so nothing beyond that is reachable anyway.
  return __builtin_ctz(rotatedWindowStart() | 0x8u);
}

void XtensaCore::windowOverflow() {
  // Rotate onto the oldest live frame — the one whose registers the current
  // frame is about to need — and hand it to the firmware's spill handler. The
  // distance to the frame *after* that one is its size, and picks the vector:
  // a 4-register frame spills through a different handler than a 12-register
  // one because it has different registers to write.
  const uint32_t above = rotatedWindowStart();
  const int distance = __builtin_ctz(above | 0x10000u) + 1;
  const uint32_t oldBase = windowBase_;
  rotateWindow(distance);
  const uint32_t beyond = above >> distance;
  const int size = beyond ? __builtin_ctz(beyond) : 2;
  static const uint32_t kOverflow[3] = {kWindowOverflow4, kWindowOverflow8, kWindowOverflow12};
  windowException(kOverflow[size < 2 ? size : 2], oldBase);
}

// ── Exceptions ───────────────────────────────────────────────────────────────

void XtensaCore::noteException(uint32_t cause, uint32_t vaddr) {
  ++exceptions_;
  if (firstException_.seen) return;
  firstException_.seen = true;
  firstException_.cause = cause;
  firstException_.pc = pc_;
  firstException_.vaddr = vaddr;
  firstException_.trace = recentCalls();
}

void XtensaCore::exception(uint32_t cause, uint32_t vaddr) {
  // Level-1 interrupts arrive here too, which is the architecture's own design
  // rather than a shortcut: PS.EXCM masks level 1, so the general exception
  // vector is where the firmware handles them.
  if (cause != kLevel1Interrupt) noteException(cause, vaddr);

  if (ps_ & kPsExcm) {
    // An exception while already handling one. The firmware's double-exception
    // vector is the only thing that can say anything useful about it.
    depc_ = pc_;
    exccause_ = cause;
    excvaddr_ = vaddr;
    const uint32_t target = vecbase_ + kDoubleVector;
    noteTransfer(pc_, target);
    pc_ = target;
    tookBranch_ = true;
    return;
  }

  epc_[1] = pc_;
  exccause_ = cause;
  if (vaddr) excvaddr_ = vaddr;
  ps_ |= kPsExcm;
  const uint32_t target = vecbase_ + ((ps_ & kPsUm) ? kUserVector : kKernelVector);
  noteTransfer(pc_, target);
  pc_ = target;
  tookBranch_ = true;
}

void XtensaCore::windowException(uint32_t vectorOffset, uint32_t oldBase) {
  // Window spill is normal traffic, not a fault, so it is deliberately not
  // recorded as the firmware's first exception.
  ++exceptions_;
  epc_[1] = pc_;
  ps_ = (ps_ & ~kPsOwb) | ((oldBase << kPsOwbShift) & kPsOwb) | kPsExcm;
  pc_ = vecbase_ + vectorOffset;
  tookBranch_ = true;
}

void XtensaCore::illegal(uint32_t instruction) {
  char buf[160];
  std::snprintf(buf, sizeof(buf), "illegal instruction 0x%06X at 0x%08X", instruction & 0xFFFFFF, pc_);
  // Trap first when the firmware has vectors of its own: its panic handler
  // prints a backtrace, which beats anything this layer can say.
  if (vecbase_ != soc_.rom.base) {
    exception(kIllegalInstruction);
  } else {
    stop(HaltReason::Unimplemented, buf);
  }
}

void XtensaCore::unimplemented(const char* what, uint32_t instruction) {
  char buf[200];
  std::snprintf(buf, sizeof(buf), "%s (0x%06X) at 0x%08X is not implemented by the Xtensa core", what,
                instruction & 0xFFFFFF, pc_);
  stop(HaltReason::Unimplemented, buf);
}

// ── Interrupts ───────────────────────────────────────────────────────────────

void XtensaCore::setInterruptPending(int line, bool pending) {
  if (line < 0 || line >= 32) return;
  const uint32_t bit = 1u << line;
  const bool wasAsserted = (externalLines_ & bit) != 0;
  if (pending) {
    externalLines_ |= bit;
  } else {
    externalLines_ &= ~bit;
  }
  if (kEdgeTriggered & bit) {
    // An edge-triggered line latches on the rising edge and is cleared by the
    // handler writing INTCLEAR; dropping the line does not clear it.
    if (pending && !wasAsserted) interrupt_ |= bit;
  } else {
    if (pending) {
      interrupt_ |= bit;
    } else {
      interrupt_ &= ~bit;
    }
  }
}

bool XtensaCore::interruptsEnabled() const { return (ps_ & kPsIntLevel) < kNumLevels; }

uint32_t XtensaCore::ccount() const { return static_cast<uint32_t>(retired_ + cycleBase_); }

void XtensaCore::refreshTimerInterrupts() {
  // CCOMPARE fires when CCOUNT passes it. The comparison is on the difference
  // so that it stays right across the counter's 32-bit wrap.
  static constexpr int kTimerLine[3] = {6, 15, 16};
  const uint32_t now = ccount();
  for (int i = 0; i < 3; ++i) {
    if (!ccompare_[i]) continue;
    if (static_cast<int32_t>(now - ccompare_[i]) >= 0) interrupt_ |= 1u << kTimerLine[i];
  }
}

bool XtensaCore::serviceInterrupt() {
  const uint32_t ready = interrupt_ & intenable_;
  if (!ready) return false;

  const uint32_t current = ps_ & kPsIntLevel;
  int bestLine = -1;
  int bestLevel = 0;
  for (int line = 0; line < 32; ++line) {
    if (!(ready & (1u << line))) continue;
    const int level = kInterruptLevel[line];
    if (static_cast<uint32_t>(level) <= current) continue;
    // PS.EXCM masks everything up to EXCM_LEVEL, which is what stops an
    // exception handler being interrupted before it has saved anything.
    if ((ps_ & kPsExcm) && level <= kExcmLevel) continue;
    if (level > bestLevel) {
      bestLevel = level;
      bestLine = line;
    }
  }
  if (bestLine < 0) return false;

  ++interrupts_;
  if (bestLevel == 1) {
    // Level 1 goes through the general exception vector with EXCCAUSE set,
    // because PS.EXCM is what masks it.
    exception(kLevel1Interrupt);
    return true;
  }

  epc_[bestLevel] = pc_;
  eps_[bestLevel] = ps_;
  ps_ = (ps_ & ~(kPsIntLevel | kPsUm)) | static_cast<uint32_t>(bestLevel) | kPsExcm;
  const uint32_t target = vecbase_ + kLevel2Vector + static_cast<uint32_t>(bestLevel - 2) * 0x40;
  noteTransfer(pc_, target);
  pc_ = target;
  return true;
}

// ── Special and user registers ───────────────────────────────────────────────

void XtensaCore::setPs(uint32_t value) { ps_ = value & 0x0007FF3Fu; }

uint32_t XtensaCore::readSpecial(uint32_t which) {
  switch (which) {
    case SR_LBEG: return lbeg_;
    case SR_LEND: return lend_;
    case SR_LCOUNT: return lcount_;
    case SR_SAR: return sar_;
    case SR_BR: return br_;
    case SR_LITBASE: return litbase_;
    case SR_SCOMPARE1: return scompare1_;
    case SR_WINDOWBASE: return windowBase_;
    case SR_WINDOWSTART: return windowStart_;
    case SR_MEMCTL: return memctl_;
    case SR_ATOMCTL: return atomctl_;
    case SR_DEPC: return depc_;
    case SR_CPENABLE: return cpenable_;
    case SR_INTERRUPT: return interrupt_;
    case SR_INTENABLE: return intenable_;
    case SR_PS: return ps_;
    case SR_VECBASE: return vecbase_;
    case SR_EXCCAUSE: return exccause_;
    case SR_CCOUNT: return ccount();
    // PRID is how an IDF app knows which core it is running on: the S3 reports
    // 0xCDCD for the PRO CPU and 0xABAB for the APP CPU.
    case SR_PRID: return coreId_ == 0 ? 0xCDCDu : 0xABABu;
    case SR_EXCVADDR: return excvaddr_;
    default: break;
  }
  if (which >= SR_EPC1 && which <= SR_EPC1 + 6) return epc_[which - SR_EPC1 + 1];
  if (which >= SR_EPS2 && which <= SR_EPS2 + 5) return eps_[which - SR_EPS2 + 2];
  if (which >= SR_EXCSAVE1 && which <= SR_EXCSAVE1 + 6) return excsave_[which - SR_EXCSAVE1 + 1];
  if (which >= SR_CCOMPARE0 && which <= SR_CCOMPARE0 + 2) return ccompare_[which - SR_CCOMPARE0];
  if (which >= SR_MISC0 && which <= SR_MISC0 + 3) return misc_[which - SR_MISC0];
  return otherSpecial_[which & 0xFF];
}

void XtensaCore::writeSpecial(uint32_t which, uint32_t value) {
  switch (which) {
    case SR_LBEG: lbeg_ = value; return;
    case SR_LEND: lend_ = value; return;
    case SR_LCOUNT: lcount_ = value; return;
    case SR_SAR: sar_ = value & 0x3F; return;
    case SR_BR: br_ = value & 0xFFFF; return;
    case SR_LITBASE: litbase_ = value; return;
    case SR_SCOMPARE1: scompare1_ = value; return;
    case SR_WINDOWBASE: windowBase_ = value & (kWindows - 1); return;
    case SR_WINDOWSTART: windowStart_ = value & 0xFFFF; return;
    case SR_MEMCTL: memctl_ = value; return;
    case SR_ATOMCTL: atomctl_ = value; return;
    case SR_DEPC: depc_ = value; return;
    case SR_CPENABLE: cpenable_ = value & 0xFF; return;
    case SR_INTERRUPT:  // INTSET
      // Only software and timer interrupts can be raised from software; the
      // hardware ignores the rest, and so does an app that writes them.
      interrupt_ |= value & (kSoftwareInterrupts | kTimerInterrupts);
      return;
    case SR_INTCLEAR:
      // A level-triggered line cannot be cleared this way — it follows its
      // source, and a handler that thinks otherwise would spin here.
      interrupt_ &= ~(value & (kEdgeTriggered | kSoftwareInterrupts | kTimerInterrupts));
      return;
    case SR_INTENABLE: intenable_ = value; return;
    case SR_PS: setPs(value); return;
    case SR_VECBASE: vecbase_ = value & 0xFFFFFC00u; return;
    case SR_EXCCAUSE: exccause_ = value; return;
    case SR_EXCVADDR: excvaddr_ = value; return;
    case SR_CCOUNT: cycleBase_ = value - retired_; return;
    case SR_DEBUGCAUSE: return;
    case SR_PRID: return;
    default: break;
  }
  if (which >= SR_EPC1 && which <= SR_EPC1 + 6) {
    epc_[which - SR_EPC1 + 1] = value;
    return;
  }
  if (which >= SR_EPS2 && which <= SR_EPS2 + 5) {
    eps_[which - SR_EPS2 + 2] = value;
    return;
  }
  if (which >= SR_EXCSAVE1 && which <= SR_EXCSAVE1 + 6) {
    excsave_[which - SR_EXCSAVE1 + 1] = value;
    return;
  }
  if (which >= SR_CCOMPARE0 && which <= SR_CCOMPARE0 + 2) {
    static constexpr int kTimerLine[3] = {6, 15, 16};
    const int index = static_cast<int>(which - SR_CCOMPARE0);
    ccompare_[index] = value;
    // Writing a comparator acknowledges its interrupt, which is how the
    // firmware's tick handler rearms without a separate clear.
    interrupt_ &= ~(1u << kTimerLine[index]);
    return;
  }
  if (which >= SR_MISC0 && which <= SR_MISC0 + 3) {
    misc_[which - SR_MISC0] = value;
    return;
  }
  otherSpecial_[which & 0xFF] = value;
}

uint32_t XtensaCore::readUser(uint32_t which) {
  switch (which) {
    case UR_THREADPTR: return threadptr_;
    case UR_FCR: return fcr_;
    case UR_FSR: return fsr_;
    default: return 0;
  }
}

void XtensaCore::writeUser(uint32_t which, uint32_t value) {
  switch (which) {
    case UR_THREADPTR: threadptr_ = value; return;
    case UR_FCR: fcr_ = value; return;
    case UR_FSR: fsr_ = value; return;
    default: return;
  }
}

bool XtensaCore::coprocessorReady(int coprocessor) {
  if (cpenable_ & (1u << coprocessor)) return true;
  // FreeRTOS on the S3 switches FPU context lazily: the first float a task
  // executes traps here, and the handler saves whoever had the coprocessor
  // before handing it over. Skipping the trap would silently share one FPU
  // state between every task.
  exception(kCoprocessor0Disabled + static_cast<uint32_t>(coprocessor));
  return false;
}

// ── Calling convention, for ROM interception ─────────────────────────────────

uint32_t XtensaCore::arg(int index) const {
  // A windowed call has not rotated the window yet when the ROM routine is
  // intercepted at its entry point — ENTRY, which would have done it, is code
  // this emulator never runs. So the callee's a2..a7 are still the caller's
  // a(2 + 4*CALLINC) .. a(7 + 4*CALLINC), which is where the caller put them.
  const int base = romCallInc_ * 4 + 2 + index;
  if (index < 6) return ar(base);
  // Arguments past the sixth are in the caller's outgoing-argument area, at
  // the bottom of its own frame.
  return const_cast<Bus&>(bus_).read32(ar(1) + static_cast<uint32_t>(index - 6) * 4);
}

void XtensaCore::setArg(int index, uint32_t value) {
  if (index < 6) setAr(romCallInc_ * 4 + 2 + index, value);
}

void XtensaCore::setReturn(uint32_t value) { setAr(romCallInc_ * 4 + 2, value); }

void XtensaCore::setReturn64(uint64_t value) {
  setAr(romCallInc_ * 4 + 2, static_cast<uint32_t>(value));
  setAr(romCallInc_ * 4 + 3, static_cast<uint32_t>(value >> 32));
}

void XtensaCore::returnToCaller() {
  const uint32_t link = ar(romCallInc_ * 4);
  // The top two bits of a windowed return address hold the call's window
  // increment, not address bits; the address itself comes from the PC's region.
  pc_ = romCallInc_ ? ((pc_ & 0xC0000000u) | (link & 0x3FFFFFFFu)) : link;
  tookBranch_ = true;
}

bool XtensaCore::callGuest(uint32_t function, const uint32_t* args, int argCount, uint32_t* result) {
  if (callGuestDepth_ > 4) return false;
  ++callGuestDepth_;

  // A sentinel return address inside the ROM window that no routine occupies:
  // running back to it is how the core knows the guest function has returned.
  const uint32_t sentinel = soc_.rom.base + soc_.rom.size - 4;
  const uint32_t savedPc = pc_;
  const uint32_t savedPs = ps_;
  const uint32_t savedBase = windowBase_;
  const uint32_t savedStart = windowStart_;
  const int savedInc = romCallInc_;
  uint32_t savedAr[64];
  std::memcpy(savedAr, ar_, sizeof(savedAr));
  const HaltReason savedHalt = halt_;
  const std::string savedDetail = haltDetail_;
  halt_ = HaltReason::Running;

  // Enter as a CALL8 would: the callee's ENTRY rotates two window positions
  // on, finds its arguments in a2..a7, and RETW rotates back to the sentinel.
  ps_ = (ps_ & ~kPsCallInc) | (2u << kPsCallIncShift) | kPsWoe;
  ps_ &= ~kPsExcm;
  setAr(8, (2u << 30) | (sentinel & 0x3FFFFFFFu));
  for (int i = 0; i < argCount && i < 6; ++i) setAr(10 + i, args[i]);
  pc_ = function;

  bool ok = false;
  for (uint64_t guard = 0; guard < 200000000ULL; ++guard) {
    if (pc_ == sentinel) {
      ok = true;
      break;
    }
    if (!stepOnce()) break;
  }
  if (ok && result) *result = ar(10);

  std::memcpy(ar_, savedAr, sizeof(ar_));
  pc_ = savedPc;
  ps_ = savedPs;
  windowBase_ = savedBase;
  windowStart_ = savedStart;
  romCallInc_ = savedInc;
  if (ok) {
    halt_ = savedHalt;
    haltDetail_ = savedDetail;
  }
  --callGuestDepth_;
  return ok;
}

// ── Reporting ────────────────────────────────────────────────────────────────

const char* XtensaCore::causeName(uint32_t cause) {
  switch (cause) {
    case kIllegalInstruction: return "illegal instruction";
    case kSyscall: return "syscall";
    case kInstructionFetchError: return "instruction fetch error";
    case kLoadStoreError: return "load/store error";
    case kLevel1Interrupt: return "level 1 interrupt";
    case kAlloca: return "alloca (movsp)";
    case kIntegerDivideByZero: return "integer divide by zero";
    case kPrivileged: return "privileged instruction";
    case kLoadStoreAlignment: return "load/store alignment";
    case 20: return "instruction fetch prohibited";
    case 28: return "load prohibited";
    case 29: return "store prohibited";
    default: break;
  }
  if (cause >= kCoprocessor0Disabled && cause <= kCoprocessor0Disabled + 7) return "coprocessor disabled";
  return "exception";
}

std::string XtensaCore::describeState() const {
  // All sixteen, because on a windowed machine the interesting register is
  // rarely one of the first few: arguments to a call live in a8-a15 until the
  // callee's ENTRY moves the window under them.
  char buf[768];
  std::snprintf(buf, sizeof(buf),
                "cpu%d pc=0x%08X ps=0x%05X (intlevel %u%s%s callinc %u) wb=%u ws=0x%04X\n"
                "       a0-a7  0x%08X 0x%08X 0x%08X 0x%08X 0x%08X 0x%08X 0x%08X 0x%08X\n"
                "       a8-a15 0x%08X 0x%08X 0x%08X 0x%08X 0x%08X 0x%08X 0x%08X 0x%08X\n"
                "       exccause=%u (%s) excvaddr=0x%08X epc1=0x%08X intenable=0x%08X",
                coreId_, pc_, ps_, ps_ & kPsIntLevel, (ps_ & kPsExcm) ? " excm" : "",
                (ps_ & kPsWoe) ? " woe" : "", (ps_ & kPsCallInc) >> kPsCallIncShift, windowBase_,
                windowStart_, ar(0), ar(1), ar(2), ar(3), ar(4), ar(5), ar(6), ar(7), ar(8), ar(9),
                ar(10), ar(11), ar(12), ar(13), ar(14), ar(15), exccause_, causeName(exccause_),
                excvaddr_, epc_[1], intenable_);
  return buf;
}

std::vector<std::pair<uint32_t, uint32_t>> XtensaCore::recentCalls() const {
  std::vector<std::pair<uint32_t, uint32_t>> out;
  const size_t count = callCursor_ < kCallRing ? callCursor_ : kCallRing;
  for (size_t i = 0; i < count; ++i) {
    const size_t index = (callCursor_ + kCallRing - count + i) % kCallRing;
    if (calls_[index].first || calls_[index].second) out.push_back(calls_[index]);
  }
  return out;
}

// ── Running ──────────────────────────────────────────────────────────────────

bool XtensaCore::stepOnce() {
  if (parked_) return false;
  if (halt_ != HaltReason::Running) return false;
  if (bus_.faulted()) return false;

  if (!breakpoints_.empty()) {
    for (uint32_t address : breakpoints_) {
      if (address != pc_) continue;
      char buf[64];
      std::snprintf(buf, sizeof(buf), "breakpoint at 0x%08X", pc_);
      stop(HaltReason::Breakpoint, buf);
      return false;
    }
  }

  refreshTimerInterrupts();
  serviceInterrupt();
  if (halt_ != HaltReason::Running) return false;

  // A call into mask ROM is intercepted and run natively — the emulator has no
  // ROM bytes, by design (see Rom.cpp).
  if (inRange(soc_.rom, pc_)) {
    if (!rom_ || !rom_->call(*this, pc_)) {
      char buf[192];
      std::snprintf(buf, sizeof(buf), "call into unimplemented mask ROM at 0x%08X (a0=0x%08X)", pc_,
                    ar(romCallInc_ * 4));
      stop(HaltReason::Unimplemented, buf);
      return false;
    }
    ++retired_;
    return true;
  }

  const uint32_t at = pc_;
  tookBranch_ = false;
  execute();
  ++retired_;

  // The zero-overhead loop closes here, and only on a straight-line fall
  // through: a branch that happens to land on LEND is not the loop ending.
  if (!tookBranch_ && lcount_ != 0 && pc_ == lend_ && at < lend_) {
    --lcount_;
    pc_ = lbeg_;
  }
  return halt_ == HaltReason::Running && !bus_.faulted();
}

uint64_t XtensaCore::run(uint64_t budget) {
  uint64_t executed = 0;
  while (executed < budget) {
    if (!stepOnce()) break;
    ++executed;
  }
  if (bus_.faulted() && halt_ == HaltReason::Running) {
    stop(HaltReason::Fault, bus_.fault().detail);
  }
  return executed;
}

// ── Decode ───────────────────────────────────────────────────────────────────
//
// Xtensa instructions are 24 bits, little-endian in memory, with a 16-bit
// "density" encoding for the common ones. op0 — the low nibble of the first
// byte — says which it is and which group the rest belongs to.

// The highest address register this instruction names, or 0 if it names none
// above a0. Only the fields that really are register operands count: the same
// four bits hold a shift amount in one instruction and a register number in
// the next, and treating an immediate as a register number would spill the
// window for no reason.
int XtensaCore::highestRegister(uint32_t inst) const {
  const uint32_t op0 = inst & 0xF;
  const int t = static_cast<int>((inst >> 4) & 0xF);
  const int s = static_cast<int>((inst >> 8) & 0xF);
  const int r = static_cast<int>((inst >> 12) & 0xF);
  const uint32_t op1 = (inst >> 16) & 0xF;
  const uint32_t op2 = (inst >> 20) & 0xF;
  auto top = [](int a, int b) { return a > b ? a : b; };
  auto top3 = [&](int a, int b, int c) { return top(a, top(b, c)); };

  switch (op0) {
    case 0x0:
      switch (op1) {
        case 0x0:
          switch (op2) {
            case 0x0:  // ST0
              switch (r) {
                case 0x0: {
                  const int m = t >> 2;
                  const int n = t & 3;
                  if (m == 2) return n == 2 ? s : 0;      // JX, or RET/RETW on a0
                  if (m == 3) return top(s, n * 4);       // CALLX: target, and the link
                  return 0;
                }
                case 0x1: return top(t, s);               // MOVSP
                case 0x6: return t;                       // RSIL
                default: return 0;
              }
            case 0x1:
            case 0x2:
            case 0x3: return top3(r, s, t);               // AND, OR, XOR
            case 0x4:                                     // ST1
              switch (r) {
                case 0x0:
                case 0x1:
                case 0x2:
                case 0x3: return s;                       // SSR, SSL, SSA8L, SSA8B
                case 0x6:
                case 0x7:
                case 0xE:
                case 0xF: return top(t, s);               // RER, WER, NSA, NSAU
                default: return 0;                        // SSAI, ROTW
              }
            case 0x5: return top(t, s);                   // the TLB group
            case 0x6: return top(r, t);                   // NEG, ABS
            default: return top3(r, s, t);                // ADD, ADDX, SUB, SUBX
          }
        case 0x1:
          switch (op2) {
            case 0x0:
            case 0x1: return top(r, s);                   // SLLI
            case 0x2:
            case 0x3:
            case 0x4: return top(r, t);                   // SRAI, SRLI
            case 0x6: return t;                           // XSR
            case 0x8: return top3(r, s, t);               // SRC
            case 0x9:
            case 0xB: return top(r, t);                   // SRL, SRA
            case 0xA: return top(r, s);                   // SLL
            default: return top3(r, s, t);                // MUL16
          }
        case 0x2: return op2 >= 0x8 ? top3(r, s, t) : 0;  // multiply/divide vs booleans
        case 0x3:
          switch (op2) {
            case 0x0:
            case 0x1:
            case 0xF: return t;                           // RSR, WSR, WUR
            case 0xE: return r;                           // RUR names its destination in r
            case 0x2:
            case 0x3:
            case 0xC:
            case 0xD: return top(r, s);                   // SEXT, CLAMPS, MOVF, MOVT
            default: return top3(r, s, t);                // MIN/MAX, MOVEQZ family
          }
        case 0x4:
        case 0x5: return top(r, t);                       // EXTUI
        case 0x8: return top(s, t);                       // LSX family: the float is in r
        case 0x9: return top(t, s);                       // L32E, S32E
        case 0xA:                                         // FP0
          if (op2 == 0xC || op2 == 0xD) return s;         // FLOAT.S, UFLOAT.S
          if (op2 == 0x8 || op2 == 0x9 || op2 == 0xA || op2 == 0xB || op2 == 0xE) return r;
          if (op2 == 0xF) return t == 4 ? r : (t == 5 ? s : 0);  // RFR, WFR
          return 0;
        case 0xB: return (op2 >= 0x8 && op2 <= 0xB) ? t : 0;     // MOVEQZ.S family
        default: return 0;
      }
    case 0x1: return t;                                   // L32R
    case 0x2:                                             // LSAI
      switch (r) {
        case 0x7: return s;                               // cache hints
        case 0xA: return t;                               // MOVI
        default: return top(t, s);
      }
    case 0x3: return s;                                   // LSI family: the float is in t
    case 0x4: return top3(r, s, t);                       // MAC16
    case 0x5: return static_cast<int>((inst >> 4) & 3) * 4;  // CALLn writes the link
    case 0x6: {
      const uint32_t n = static_cast<uint32_t>(t) & 3;
      const uint32_t m = static_cast<uint32_t>(t) >> 2;
      if (n == 0) return 0;                               // J
      if (n == 3) {
        if (m == 0) {                                     // ENTRY
          const int inc = static_cast<int>((ps_ & kPsCallInc) >> kPsCallIncShift);
          return top(s, inc * 4 + (s & 3));
        }
        if (m == 1) return (r == 0 || r == 1) ? 0 : s;    // BF/BT take a boolean
      }
      return s;
    }
    case 0x7:                                             // conditional branches
      return (r == 0x6 || r == 0x7 || r == 0xE || r == 0xF) ? s : top(s, t);  // BBCI/BBSI
    case 0x8:
    case 0x9: return top(t, s);                           // L32I.N, S32I.N
    case 0xA: return top3(r, s, t);                       // ADD.N
    case 0xB: return top(r, s);                           // ADDI.N
    case 0xC: return s;                                   // MOVI.N, BEQZ.N, BNEZ.N
    case 0xD: return r == 0 ? top(t, s) : 0;              // MOV.N
    default: return 0;
  }
}

void XtensaCore::execute() {
  bus_.setPc(pc_);
  // Instructions are byte-aligned, so one straddles two words about half the
  // time. Fetching the word the PC sits in and, only when it has to, the one
  // after keeps that to one bus lookup in the common case.
  const uint32_t aligned = pc_ & ~3u;
  const uint32_t within = pc_ & 3u;
  uint32_t bytes = bus_.read32(aligned) >> (within * 8);
  if (bus_.faulted()) return;
  if (within > 1) {
    bytes |= bus_.read32(aligned + 4) << ((4 - within) * 8);
    if (bus_.faulted()) return;
  }
  const bool narrow = (bytes & 0xF) >= 8;
  const uint32_t instruction = narrow ? (bytes & 0xFFFF) : (bytes & 0xFFFFFF);

  // Windowed calls only: inside an exception handler (PS.EXCM) the whole
  // register file is addressable, which is what lets the spill handlers reach
  // the frame they are saving.
  // Three free window positions is as far as any register can reach — a15 is
  // in the fourth — so the common case needs no per-operand decode at all.
  if ((ps_ & (kPsWoe | kPsExcm)) == kPsWoe && windowAvailable() < 3) {
    const int highest = highestRegister(instruction);
    if (highest >= 4 && highest / 4 > windowAvailable()) {
      windowOverflow();
      return;
    }
  }

  if (narrow) {
    executeNarrow(instruction);
    return;
  }
  executeWide(instruction);
}

void XtensaCore::executeNarrow(uint32_t inst) {
  const uint32_t op0 = inst & 0xF;
  const uint32_t t = (inst >> 4) & 0xF;
  const uint32_t s = (inst >> 8) & 0xF;
  const uint32_t r = (inst >> 12) & 0xF;
  const uint32_t next = pc_ + 2;

  auto jump = [&](uint32_t target) {
    pc_ = target;
    tookBranch_ = true;
    if (inRange(soc_.rom, target)) romCallInc_ = 0;
  };

  switch (op0) {
    case 0x8:  // L32I.N
      setAr(static_cast<int>(t), bus_.read32(ar(static_cast<int>(s)) + r * 4));
      break;
    case 0x9:  // S32I.N
      bus_.write32(ar(static_cast<int>(s)) + r * 4, ar(static_cast<int>(t)));
      break;
    case 0xA:  // ADD.N
      setAr(static_cast<int>(r), ar(static_cast<int>(s)) + ar(static_cast<int>(t)));
      break;
    case 0xB:  // ADDI.N — an immediate of 0 encodes -1, so the useful range is
               // -1..15 rather than 0..15.
      setAr(static_cast<int>(r),
            ar(static_cast<int>(s)) + (t == 0 ? 0xFFFFFFFFu : t));
      break;
    case 0xC: {  // ST2: MOVI.N, BEQZ.N, BNEZ.N
      // These two formats put their register in `s` and scatter the immediate
      // across the word, with bit 7 choosing between them and bit 6 choosing
      // the branch sense.
      const uint32_t low = (inst >> 12) & 0xF;
      if (!((inst >> 7) & 1)) {  // MOVI.N as, imm7 — range -32..95
        const uint32_t imm7 = low | (((inst >> 4) & 7) << 4);
        setAr(static_cast<int>(s), imm7 >= 96 ? imm7 | 0xFFFFFF80u : imm7);
        break;
      }
      const uint32_t offset = low | (((inst >> 4) & 3) << 4);
      const bool zero = ar(static_cast<int>(s)) == 0;
      const bool taken = ((inst >> 6) & 1) ? !zero : zero;
      if (taken) {
        jump(pc_ + 4 + offset);
        return;
      }
      break;
    }
    case 0xD: {  // ST3: MOV.N and the RET/NOP family
      if (r == 0) {
        setAr(static_cast<int>(t), ar(static_cast<int>(s)));
        break;
      }
      if (r != 0xF) {
        illegal(inst);
        return;
      }
      switch (t) {
        case 0:  // RET.N
          jump(ar(0));
          return;
        case 1: {  // RETW.N
          executeWide(0x000090);  // same semantics as the 24-bit RETW
          return;
        }
        case 2:  // BREAK.N
          stop(HaltReason::Breakpoint, "break.n");
          return;
        case 3:  // NOP.N
          break;
        default:
          illegal(inst);
          return;
      }
      break;
    }
    default:
      illegal(inst);
      return;
  }
  if (!tookBranch_) pc_ = next;
}

void XtensaCore::executeWide(uint32_t inst) {
  const uint32_t op0 = inst & 0xF;
  const uint32_t t = (inst >> 4) & 0xF;
  const uint32_t s = (inst >> 8) & 0xF;
  const uint32_t r = (inst >> 12) & 0xF;
  const uint32_t op1 = (inst >> 16) & 0xF;
  const uint32_t op2 = (inst >> 20) & 0xF;
  const uint32_t imm8 = (inst >> 16) & 0xFF;
  const uint32_t next = pc_ + 3;

  auto A = [&](uint32_t index) { return ar(static_cast<int>(index)); };
  auto setA = [&](uint32_t index, uint32_t value) { setAr(static_cast<int>(index), value); };
  auto jump = [&](uint32_t target) {
    pc_ = target;
    tookBranch_ = true;
    if (inRange(soc_.rom, target)) romCallInc_ = 0;
  };
  // A call carries its window increment into mask ROM: an intercepted routine
  // needs it to find the arguments the caller placed.
  auto call = [&](uint32_t target, int inc) {
    noteTransfer(pc_, target);
    pc_ = target;
    tookBranch_ = true;
    if (inRange(soc_.rom, target)) romCallInc_ = inc;
  };
  auto branchIf = [&](bool taken, int32_t offset) {
    if (taken) {
      pc_ = pc_ + 4 + static_cast<uint32_t>(offset);
      tookBranch_ = true;
    }
  };

  switch (op0) {
    // ── QRST: the register-to-register world ─────────────────────────────────
    case 0x0:
      switch (op1) {
        case 0x0:
          switch (op2) {
            case 0x0:  // ST0
              switch (r) {
                case 0x0: {  // SNM0: RET/RETW/JX/CALLX
                  const uint32_t m = t >> 2;
                  const uint32_t n = t & 3;
                  if (m == 2) {
                    if (n == 0) {  // RET
                      jump(A(0));
                      return;
                    }
                    if (n == 1) {  // RETW
                      const uint32_t link = A(0);
                      const uint32_t decrement = link >> 30;
                      const uint32_t target = (pc_ & 0xC0000000u) | (link & 0x3FFFFFFFu);
                      if (!(ps_ & kPsWoe) || (ps_ & kPsExcm)) {
                        jump(target);
                        return;
                      }
                      if (decrement == 0) {
                        illegal(inst);
                        return;
                      }
                      const uint32_t oldBase = windowBase_;
                      const uint32_t returning =
                          (windowBase_ - decrement) & (kWindows - 1);
                      if (!(windowStart_ & (1u << returning))) {
                        // The caller's registers are on its stack, not in the
                        // file. The frame stays marked live while the
                        // firmware's underflow handler puts them back; RFWU
                        // then re-runs this RETW, which finds them there.
                        rotateWindow(-static_cast<int>(decrement));
                        static const uint32_t kUnderflow[4] = {0, kWindowUnderflow4,
                                                               kWindowUnderflow8, kWindowUnderflow12};
                        windowException(kUnderflow[decrement], oldBase);
                        return;
                      }
                      windowStart_ &= ~(1u << windowBase_);
                      rotateWindow(-static_cast<int>(decrement));
                      noteTransfer(pc_, target);
                      pc_ = target;
                      tookBranch_ = true;
                      if (inRange(soc_.rom, target)) romCallInc_ = 0;
                      return;
                    }
                    if (n == 2) {  // JX
                      const uint32_t target = A(s);
                      noteTransfer(pc_, target);
                      jump(target);
                      return;
                    }
                    illegal(inst);
                    return;
                  }
                  if (m == 3) {  // CALLX0/4/8/12
                    const uint32_t target = A(s);
                    const uint32_t returnTo = next;
                    if (n == 0) {
                      setA(0, returnTo);
                      ps_ &= ~kPsCallInc;
                    } else {
                      ps_ = (ps_ & ~kPsCallInc) | (n << kPsCallIncShift);
                      setA(n * 4, (n << 30) | (returnTo & 0x3FFFFFFFu));
                    }
                    call(target, static_cast<int>(n));
                    return;
                  }
                  illegal(inst);
                  return;
                }
                case 0x1: {  // MOVSP
                  // Moving the stack pointer is only safe while the caller's
                  // registers are still in the file; if they have been spilled
                  // the firmware has to fix the saved copy up instead.
                  const bool live = (windowStart_ & (1u << ((windowBase_ - 1) & 15))) ||
                                    (windowStart_ & (1u << ((windowBase_ - 2) & 15))) ||
                                    (windowStart_ & (1u << ((windowBase_ - 3) & 15)));
                  if (!live) {
                    exception(kAlloca);
                    return;
                  }
                  setA(t, A(s));
                  break;
                }
                case 0x2:  // SYNC: ISYNC/RSYNC/MEMW/EXTW/NOP and friends
                  break;
                case 0x3:  // RFEI
                  if (t == 0) {
                    switch (s) {
                      case 0:  // RFE
                      case 1:  // RFUE
                        ps_ &= ~kPsExcm;
                        jump(epc_[1]);
                        return;
                      case 2:  // RFDE
                        jump(depc_);
                        return;
                      case 4: {  // RFWO — the spilled frame is no longer live
                        windowStart_ &= ~(1u << windowBase_);
                        windowBase_ = (ps_ & kPsOwb) >> kPsOwbShift;
                        ps_ &= ~kPsExcm;
                        jump(epc_[1]);
                        return;
                      }
                      case 5: {  // RFWU — the restored frame is live again
                        windowStart_ |= 1u << windowBase_;
                        windowBase_ = (ps_ & kPsOwb) >> kPsOwbShift;
                        ps_ &= ~kPsExcm;
                        jump(epc_[1]);
                        return;
                      }
                      default:
                        illegal(inst);
                        return;
                    }
                  }
                  if (t == 1) {  // RFI level
                    const uint32_t level = s & 7;
                    ps_ = eps_[level ? level : 1];
                    jump(epc_[level ? level : 1]);
                    return;
                  }
                  illegal(inst);
                  return;
                case 0x4: {  // BREAK
                  char buf[96];
                  std::snprintf(buf, sizeof(buf), "break %u, %u at 0x%08X", s, t, pc_);
                  stop(HaltReason::Breakpoint, buf);
                  return;
                }
                case 0x5:  // SYSCALL
                  exception(kSyscall);
                  return;
                case 0x6:  // RSIL
                  setA(t, ps_);
                  ps_ = (ps_ & ~kPsIntLevel) | s;
                  break;
                case 0x7:  // WAITI
                  ps_ = (ps_ & ~kPsIntLevel) | s;
                  pc_ = next;
                  tookBranch_ = true;
                  // Nothing else will happen until an interrupt arrives; the
                  // machine skips simulated time forward to the next one
                  // rather than spinning here.
                  stop(HaltReason::WaitingForInterrupt, "waiti");
                  return;
                case 0x8:  // ANY4
                case 0x9:  // ALL4
                case 0xA:  // ANY8
                case 0xB: {  // ALL8
                  const int width = (r < 0xA) ? 4 : 8;
                  const uint32_t mask = ((1u << width) - 1) << s;
                  const bool any = (br_ & mask) != 0;
                  const bool all = (br_ & mask) == mask;
                  const bool value = (r == 0x8 || r == 0xA) ? any : all;
                  br_ = value ? (br_ | (1u << t)) : (br_ & ~(1u << t));
                  break;
                }
                default:
                  illegal(inst);
                  return;
              }
              break;
            case 0x1: setA(r, A(s) & A(t)); break;  // AND
            case 0x2: setA(r, A(s) | A(t)); break;  // OR
            case 0x3: setA(r, A(s) ^ A(t)); break;  // XOR
            case 0x4:  // ST1: the shift-amount register, window rotation, NSA
              switch (r) {
                case 0x0: sar_ = A(s) & 0x1F; break;                       // SSR
                case 0x1: sar_ = 32 - (A(s) & 0x1F); break;                // SSL
                case 0x2: sar_ = (A(s) & 3) * 8; break;                    // SSA8L
                case 0x3: sar_ = 32 - (A(s) & 3) * 8; break;               // SSA8B
                case 0x4: sar_ = s | ((t & 1) << 4); break;                // SSAI
                case 0x8: rotateWindow(signExtend(t, 4)); break;           // ROTW
                case 0xE: {                                                // NSA
                  const int32_t value = static_cast<int32_t>(A(s));
                  const uint32_t folded = value < 0 ? ~static_cast<uint32_t>(value)
                                                    : static_cast<uint32_t>(value);
                  setA(t, folded == 0 ? 31 : static_cast<uint32_t>(__builtin_clz(folded)) - 1);
                  break;
                }
                case 0xF: {  // NSAU
                  const uint32_t value = A(s);
                  setA(t, value == 0 ? 32 : static_cast<uint32_t>(__builtin_clz(value)));
                  break;
                }
                default:
                  illegal(inst);
                  return;
              }
              break;
            case 0x5:  // TLB: the region-protection entries
              // The S3's MMU is eight fixed regions whose cache and access
              // attributes these instructions set — and this emulator models
              // neither caching nor protection, so setting them changes
              // nothing and reading one back reports the address it was asked
              // about. Firmware that expects a protection *fault* will not get
              // one here; nothing else notices.
              switch (r) {
                case 0x3:  // RITLB0
                case 0x7:  // RITLB1
                case 0xB:  // RDTLB0
                case 0xF:  // RDTLB1
                  setA(t, A(s) & 0xE0000000u);
                  break;
                case 0x5:  // PITLB
                case 0xD:  // PDTLB
                  // "Hit, in the way that covers this address."
                  setA(t, (A(s) & 0xE0000000u) | 0x10u);
                  break;
                case 0x4:  // IITLB
                case 0x6:  // WITLB
                case 0xC:  // IDTLB
                case 0xE:  // WDTLB
                  break;
                default:
                  illegal(inst);
                  return;
              }
              break;
            case 0x6:  // RT0: NEG, ABS
              if (s == 0) {
                setA(r, 0u - A(t));
              } else if (s == 1) {
                const int32_t value = static_cast<int32_t>(A(t));
                setA(r, value < 0 ? static_cast<uint32_t>(-static_cast<int64_t>(value))
                                  : static_cast<uint32_t>(value));
              } else {
                illegal(inst);
                return;
              }
              break;
            case 0x8: setA(r, A(s) + A(t)); break;         // ADD
            case 0x9: setA(r, (A(s) << 1) + A(t)); break;  // ADDX2
            case 0xA: setA(r, (A(s) << 2) + A(t)); break;  // ADDX4
            case 0xB: setA(r, (A(s) << 3) + A(t)); break;  // ADDX8
            case 0xC: setA(r, A(s) - A(t)); break;         // SUB
            case 0xD: setA(r, (A(s) << 1) - A(t)); break;  // SUBX2
            case 0xE: setA(r, (A(s) << 2) - A(t)); break;  // SUBX4
            case 0xF: setA(r, (A(s) << 3) - A(t)); break;  // SUBX8
            default:
              illegal(inst);
              return;
          }
          break;

        case 0x1:  // RST1: shifts, XSR, 16-bit multiply
          switch (op2) {
            case 0x0:
            case 0x1: {  // SLLI — its shift is in `t`, and the field holds
                         // 32 - shift, which is why a left shift by 32 has no
                         // encoding.
              const uint32_t amount = 32 - (t | ((op2 & 1) << 4));
              setA(r, amount >= 32 ? A(s) : A(s) << amount);
              break;
            }
            case 0x2:
            case 0x3: {  // SRAI
              const uint32_t amount = s | ((op2 & 1) << 4);
              setA(r, static_cast<uint32_t>(static_cast<int32_t>(A(t)) >> amount));
              break;
            }
            case 0x4: setA(r, A(t) >> s); break;  // SRLI
            case 0x6: {                           // XSR
              const uint32_t which = (inst >> 8) & 0xFF;
              const uint32_t old = readSpecial(which);
              writeSpecial(which, A(t));
              setA(t, old);
              break;
            }
            case 0x8: {  // SRC — funnel shift, the one that makes 64-bit
                         // shifts a pair of instructions
              const uint64_t wide = (static_cast<uint64_t>(A(s)) << 32) | A(t);
              setA(r, static_cast<uint32_t>(wide >> (sar_ & 0x3F)));
              break;
            }
            case 0x9: setA(r, (sar_ & 0x3F) >= 32 ? 0 : A(t) >> sar_); break;  // SRL
            case 0xA: {                                                        // SLL
              const uint32_t amount = 32 - (sar_ & 0x3F);
              setA(r, amount >= 32 ? 0 : A(s) << amount);
              break;
            }
            case 0xB:  // SRA
              setA(r, static_cast<uint32_t>(static_cast<int32_t>(A(t)) >>
                                            ((sar_ & 0x3F) >= 32 ? 31 : sar_)));
              break;
            case 0xC:  // MUL16U
              setA(r, (A(s) & 0xFFFF) * (A(t) & 0xFFFF));
              break;
            case 0xD:  // MUL16S
              setA(r, static_cast<uint32_t>(signExtend(A(s) & 0xFFFF, 16) *
                                            signExtend(A(t) & 0xFFFF, 16)));
              break;
            default:
              unimplemented("RST1 instruction", inst);
              return;
          }
          break;

        case 0x2:  // RST2: the 32-bit multiply and divide package, boolean ops
          switch (op2) {
            case 0x0: br_ = (br_ & ~(1u << r)) | ((((br_ >> s) & (br_ >> t)) & 1) << r); break;  // ANDB
            case 0x1: br_ = (br_ & ~(1u << r)) | ((((br_ >> s) & ~(br_ >> t)) & 1) << r); break; // ANDBC
            case 0x2: br_ = (br_ & ~(1u << r)) | ((((br_ >> s) | (br_ >> t)) & 1) << r); break;  // ORB
            case 0x3: br_ = (br_ & ~(1u << r)) | ((((br_ >> s) | ~(br_ >> t)) & 1) << r); break; // ORBC
            case 0x4: br_ = (br_ & ~(1u << r)) | ((((br_ >> s) ^ (br_ >> t)) & 1) << r); break;  // XORB
            case 0x6:  // SALTU — set if less than, unsigned
              setA(r, A(s) < A(t) ? 1 : 0);
              break;
            case 0x7:  // SALT — set if less than, signed
              setA(r, static_cast<int32_t>(A(s)) < static_cast<int32_t>(A(t)) ? 1 : 0);
              break;
            case 0x8: setA(r, A(s) * A(t)); break;                                               // MULL
            case 0xA:  // MULUH
              setA(r, static_cast<uint32_t>((static_cast<uint64_t>(A(s)) * A(t)) >> 32));
              break;
            case 0xB:  // MULSH
              setA(r, static_cast<uint32_t>(
                          (static_cast<int64_t>(static_cast<int32_t>(A(s))) *
                           static_cast<int64_t>(static_cast<int32_t>(A(t)))) >> 32));
              break;
            case 0xC:  // QUOU
              if (A(t) == 0) {
                exception(kIntegerDivideByZero);
                return;
              }
              setA(r, A(s) / A(t));
              break;
            case 0xD: {  // QUOS
              if (A(t) == 0) {
                exception(kIntegerDivideByZero);
                return;
              }
              const int32_t a = static_cast<int32_t>(A(s));
              const int32_t b = static_cast<int32_t>(A(t));
              // The one case where the quotient is not representable; the
              // hardware wraps rather than trapping.
              setA(r, (a == INT32_MIN && b == -1) ? static_cast<uint32_t>(a)
                                                  : static_cast<uint32_t>(a / b));
              break;
            }
            case 0xE:  // REMU
              if (A(t) == 0) {
                exception(kIntegerDivideByZero);
                return;
              }
              setA(r, A(s) % A(t));
              break;
            case 0xF: {  // REMS
              if (A(t) == 0) {
                exception(kIntegerDivideByZero);
                return;
              }
              const int32_t a = static_cast<int32_t>(A(s));
              const int32_t b = static_cast<int32_t>(A(t));
              setA(r, (a == INT32_MIN && b == -1) ? 0 : static_cast<uint32_t>(a % b));
              break;
            }
            default:
              unimplemented("RST2 instruction", inst);
              return;
          }
          break;

        case 0x3:  // RST3: the special registers, conditional moves, min/max
          switch (op2) {
            case 0x0: setA(t, readSpecial((inst >> 8) & 0xFF)); break;  // RSR
            case 0x1: writeSpecial((inst >> 8) & 0xFF, A(t)); break;    // WSR
            case 0x2: {  // SEXT — sign-extend from a bit the immediate names
              const int bit = static_cast<int>(t) + 7;
              setA(r, static_cast<uint32_t>(signExtend(A(s), bit + 1)));
              break;
            }
            case 0x3: {  // CLAMPS
              const int32_t value = static_cast<int32_t>(A(s));
              const int32_t limit = (1 << (static_cast<int>(t) + 7)) - 1;
              setA(r, static_cast<uint32_t>(value > limit ? limit
                                            : value < -limit - 1 ? -limit - 1
                                                                 : value));
              break;
            }
            case 0x4:  // MIN
              setA(r, static_cast<int32_t>(A(s)) < static_cast<int32_t>(A(t)) ? A(s) : A(t));
              break;
            case 0x5:  // MAX
              setA(r, static_cast<int32_t>(A(s)) > static_cast<int32_t>(A(t)) ? A(s) : A(t));
              break;
            case 0x6: setA(r, A(s) < A(t) ? A(s) : A(t)); break;  // MINU
            case 0x7: setA(r, A(s) > A(t) ? A(s) : A(t)); break;  // MAXU
            case 0x8: if (A(t) == 0) setA(r, A(s)); break;        // MOVEQZ
            case 0x9: if (A(t) != 0) setA(r, A(s)); break;        // MOVNEZ
            case 0xA: if (static_cast<int32_t>(A(t)) < 0) setA(r, A(s)); break;   // MOVLTZ
            case 0xB: if (static_cast<int32_t>(A(t)) >= 0) setA(r, A(s)); break;  // MOVGEZ
            case 0xC: if (!((br_ >> t) & 1)) setA(r, A(s)); break;                // MOVF
            case 0xD: if ((br_ >> t) & 1) setA(r, A(s)); break;                   // MOVT
            // RUR names its user register in s:t and its destination in r;
            // WUR names it in r:s with the source in t. They are mirror
            // images, not the same layout twice.
            case 0xE: setA(r, readUser((inst >> 4) & 0xFF)); break;               // RUR
            case 0xF: writeUser((inst >> 8) & 0xFF, A(t)); break;                 // WUR
            default:
              unimplemented("RST3 instruction", inst);
              return;
          }
          break;

        case 0x4:
        case 0x5: {  // EXTUI — the bitfield extract every compiler leans on.
          // The shift amount is split across two fields: four bits in `s` and
          // its top bit in op1, which is why this group covers two op1 values.
          const uint32_t shift = s | ((op1 & 1) << 4);
          const uint32_t width = op2 + 1;
          const uint32_t mask = width >= 32 ? 0xFFFFFFFFu : ((1u << width) - 1);
          setA(r, (A(t) >> shift) & mask);
          break;
        }

        case 0x8:  // LSCX: indexed float load/store
          if (!coprocessorReady(0)) return;
          switch (op2) {
            case 0x0: fr_[r] = bus_.read32(A(s) + A(t)); break;   // LSX
            case 0x4: bus_.write32(A(s) + A(t), fr_[r]); break;   // SSX
            case 0x1:                                             // LSXU
              fr_[r] = bus_.read32(A(s) + A(t));
              setA(s, A(s) + A(t));
              break;
            case 0x5:  // SSXU
              bus_.write32(A(s) + A(t), fr_[r]);
              setA(s, A(s) + A(t));
              break;
            default:
              unimplemented("LSCX instruction", inst);
              return;
          }
          break;

        case 0x9: {  // LSC4: the window spill instructions
          // L32E/S32E address relative to a frame that is *not* the current
          // window's — which is the whole point: the overflow handler runs in
          // the frame being spilled and reaches its caller's stack with these.
          const uint32_t address = A(s) + (static_cast<uint32_t>(r) << 2) - 64;
          if (op2 == 0x0) {
            setA(t, bus_.read32(address));
          } else if (op2 == 0x4) {
            bus_.write32(address, A(t));
          } else {
            unimplemented("LSC4 instruction", inst);
            return;
          }
          break;
        }

        case 0xA:  // FP0
        case 0xB:  // FP1
          if (!coprocessorReady(0)) return;
          if (op1 == 0xA) {
            switch (op2) {
              case 0x0: fr_[r] = floatToBits(bitsToFloat(fr_[s]) + bitsToFloat(fr_[t])); break;
              case 0x1: fr_[r] = floatToBits(bitsToFloat(fr_[s]) - bitsToFloat(fr_[t])); break;
              case 0x2: fr_[r] = floatToBits(bitsToFloat(fr_[s]) * bitsToFloat(fr_[t])); break;
              case 0x4:  // MADD.S
                fr_[r] = floatToBits(bitsToFloat(fr_[r]) + bitsToFloat(fr_[s]) * bitsToFloat(fr_[t]));
                break;
              case 0x5:  // MSUB.S
                fr_[r] = floatToBits(bitsToFloat(fr_[r]) - bitsToFloat(fr_[s]) * bitsToFloat(fr_[t]));
                break;
              case 0x6:  // MADDN.S — multiply-add without the final rounding
              case 0x7:  // DIVN.S — the last step of a compiled divide
                fr_[r] = floatToBits(bitsToFloat(fr_[r]) + bitsToFloat(fr_[s]) * bitsToFloat(fr_[t]));
                break;
              case 0x8:  // ROUND.S
              case 0x9:  // TRUNC.S
              case 0xA:  // FLOOR.S
              case 0xB:  // CEIL.S
              case 0xE: {  // UTRUNC.S
                const double scaled = static_cast<double>(bitsToFloat(fr_[s])) *
                                      static_cast<double>(1u << t);
                double rounded = scaled;
                if (op2 == 0x8) rounded = std::nearbyint(scaled);
                if (op2 == 0x9 || op2 == 0xE) rounded = std::trunc(scaled);
                if (op2 == 0xA) rounded = std::floor(scaled);
                if (op2 == 0xB) rounded = std::ceil(scaled);
                setA(r, op2 == 0xE ? static_cast<uint32_t>(rounded)
                                   : static_cast<uint32_t>(static_cast<int32_t>(rounded)));
                break;
              }
              case 0xC:  // FLOAT.S
                fr_[r] = floatToBits(static_cast<float>(static_cast<int32_t>(A(s))) /
                                     static_cast<float>(1u << t));
                break;
              case 0xD:  // UFLOAT.S
                fr_[r] = floatToBits(static_cast<float>(A(s)) / static_cast<float>(1u << t));
                break;
              case 0xF:
                switch (t) {
                  case 0x0: fr_[r] = fr_[s]; break;                            // MOV.S
                  case 0x1: fr_[r] = fr_[s] & 0x7FFFFFFFu; break;              // ABS.S
                  case 0x3:  // CONST.S — 0.0, then the powers of two
                    fr_[r] = s == 0 ? 0u : floatToBits(std::ldexp(1.0f, static_cast<int>(s) - 1));
                    break;
                  case 0x4: setA(r, fr_[s]); break;                            // RFR
                  case 0x5: fr_[r] = A(s); break;                              // WFR
                  case 0x6: fr_[r] = fr_[s] ^ 0x80000000u; break;              // NEG.S
                  // The seeds a compiled divide or square root starts from.
                  // The hardware produces a low-precision estimate that the
                  // Newton steps around it refine; producing the exact value
                  // here lands in the same place, to within the last bit the
                  // refinement would have settled anyway.
                  case 0x7:  // DIV0.S
                  case 0x8:  // RECIP0.S
                    fr_[r] = floatToBits(1.0f / bitsToFloat(fr_[s]));
                    break;
                  case 0x9:  // SQRT0.S
                    fr_[r] = floatToBits(std::sqrt(bitsToFloat(fr_[s])));
                    break;
                  case 0xA:  // RSQRT0.S
                    fr_[r] = floatToBits(1.0f / std::sqrt(bitsToFloat(fr_[s])));
                    break;
                  default:
                    unimplemented("FP1OP instruction", inst);
                    return;
                }
                break;
              default:
                unimplemented("FP0 instruction", inst);
                return;
            }
          } else {
            const float a = bitsToFloat(fr_[s]);
            const float b = bitsToFloat(fr_[t]);
            const bool unordered = std::isnan(a) || std::isnan(b);
            auto setBool = [&](bool value) {
              br_ = value ? (br_ | (1u << r)) : (br_ & ~(1u << r));
            };
            switch (op2) {
              case 0x1: setBool(unordered); break;                              // UN.S
              case 0x2: setBool(!unordered && a == b); break;                   // OEQ.S
              case 0x3: setBool(unordered || a == b); break;                    // UEQ.S
              case 0x4: setBool(!unordered && a < b); break;                    // OLT.S
              case 0x5: setBool(unordered || a < b); break;                     // ULT.S
              case 0x6: setBool(!unordered && a <= b); break;                   // OLE.S
              case 0x7: setBool(unordered || a <= b); break;                    // ULE.S
              case 0x8: if (A(t) == 0) fr_[r] = fr_[s]; break;                  // MOVEQZ.S
              case 0x9: if (A(t) != 0) fr_[r] = fr_[s]; break;                  // MOVNEZ.S
              case 0xA: if (static_cast<int32_t>(A(t)) < 0) fr_[r] = fr_[s]; break;   // MOVLTZ.S
              case 0xB: if (static_cast<int32_t>(A(t)) >= 0) fr_[r] = fr_[s]; break;  // MOVGEZ.S
              case 0xC: if (!((br_ >> t) & 1)) fr_[r] = fr_[s]; break;          // MOVF.S
              case 0xD: if ((br_ >> t) & 1) fr_[r] = fr_[s]; break;             // MOVT.S
              default:
                unimplemented("FP1 instruction", inst);
                return;
            }
          }
          break;

        default:
          unimplemented("QRST instruction", inst);
          return;
      }
      break;

    // ── Everything else ──────────────────────────────────────────────────────
    case 0x1: {  // L32R — the PC-relative literal load
      const uint32_t offset = (inst >> 8) & 0xFFFF;
      const uint32_t address = ((pc_ + 3) & 0xFFFFFFFCu) + ((offset | 0xFFFF0000u) << 2);
      setA(t, bus_.read32(address));
      break;
    }

    case 0x2:  // LSAI: loads, stores and the immediate arithmetic
      switch (r) {
        case 0x0: setA(t, bus_.read8(A(s) + imm8)); break;                        // L8UI
        case 0x1: setA(t, bus_.read16(A(s) + imm8 * 2)); break;                   // L16UI
        case 0x2: setA(t, bus_.read32(A(s) + imm8 * 4)); break;                   // L32I
        case 0x4: bus_.write8(A(s) + imm8, static_cast<uint8_t>(A(t))); break;    // S8I
        case 0x5: bus_.write16(A(s) + imm8 * 2, static_cast<uint16_t>(A(t))); break;  // S16I
        case 0x6: bus_.write32(A(s) + imm8 * 4, A(t)); break;                     // S32I
        case 0x7: break;  // the cache hint family: nothing here has a cache
        case 0x9:  // L16SI
          setA(t, static_cast<uint32_t>(signExtend(bus_.read16(A(s) + imm8 * 2), 16)));
          break;
        case 0xA:  // MOVI
          setA(t, static_cast<uint32_t>(signExtend((s << 8) | imm8, 12)));
          break;
        case 0xB: setA(t, bus_.read32(A(s) + imm8 * 4)); break;  // L32AI
        case 0xC:  // ADDI
          setA(t, A(s) + static_cast<uint32_t>(signExtend(imm8, 8)));
          break;
        case 0xD:  // ADDMI
          setA(t, A(s) + (static_cast<uint32_t>(signExtend(imm8, 8)) << 8));
          break;
        case 0xE: {  // S32C1I — compare and swap, against SCOMPARE1
          const uint32_t address = A(s) + imm8 * 4;
          const uint32_t current = bus_.read32(address);
          if (current == scompare1_) bus_.write32(address, A(t));
          setA(t, current);
          break;
        }
        case 0xF: bus_.write32(A(s) + imm8 * 4, A(t)); break;  // S32RI
        default:
          unimplemented("LSAI instruction", inst);
          return;
      }
      break;

    case 0x3:  // LSCI: the float loads and stores
      if (!coprocessorReady(0)) return;
      switch (r) {
        case 0x0: fr_[t] = bus_.read32(A(s) + imm8 * 4); break;  // LSI
        case 0x4: bus_.write32(A(s) + imm8 * 4, fr_[t]); break;  // SSI
        case 0x8:                                                // LSIU
          fr_[t] = bus_.read32(A(s) + imm8 * 4);
          setA(s, A(s) + imm8 * 4);
          break;
        case 0xC:  // SSIU
          bus_.write32(A(s) + imm8 * 4, fr_[t]);
          setA(s, A(s) + imm8 * 4);
          break;
        default:
          unimplemented("LSCI instruction", inst);
          return;
      }
      break;

    case 0x4:
      unimplemented("MAC16 instruction", inst);
      return;

    case 0x5: {  // CALL0/CALL4/CALL8/CALL12
      const uint32_t n = (inst >> 4) & 3;
      const int32_t offset = signExtend(inst >> 6, 18);
      const uint32_t target = ((pc_ + 4) & 0xFFFFFFFCu) + (static_cast<uint32_t>(offset) << 2);
      if (n == 0) {
        setA(0, next);
        ps_ &= ~kPsCallInc;
      } else {
        ps_ = (ps_ & ~kPsCallInc) | (n << kPsCallIncShift);
        setA(n * 4, (n << 30) | (next & 0x3FFFFFFFu));
      }
      call(target, static_cast<int>(n));
      return;
    }

    case 0x6: {  // SI: jumps, compare-with-zero and compare-with-immediate
      // `n` picks the subgroup and `m` the instruction within it; both live in
      // the `t` field, n in its low two bits.
      const uint32_t n = t & 3;
      const uint32_t m = t >> 2;
      if (n == 0) {  // J
        jump(pc_ + 4 + static_cast<uint32_t>(signExtend(inst >> 6, 18)));
        return;
      }
      if (n == 1) {  // BEQZ/BNEZ/BLTZ/BGEZ
        const int32_t offset = signExtend(inst >> 12, 12);
        const int32_t value = static_cast<int32_t>(A(s));
        const bool taken = m == 0 ? value == 0 : m == 1 ? value != 0 : m == 2 ? value < 0 : value >= 0;
        branchIf(taken, offset);
        break;
      }
      if (n == 2) {  // BEQI/BNEI/BLTI/BGEI
        const int32_t value = static_cast<int32_t>(A(s));
        const int32_t immediate = kB4Const[r];
        const bool taken = m == 0   ? value == immediate
                           : m == 1 ? value != immediate
                           : m == 2 ? value < immediate
                                    : value >= immediate;
        branchIf(taken, signExtend(imm8, 8));
        break;
      }
      // n == 3
      if (m == 0) {  // ENTRY
        const uint32_t frame = ((inst >> 12) & 0xFFF) << 3;
        if (!(ps_ & kPsWoe) || (ps_ & kPsExcm)) {
          setA(s, A(s) - frame);
          break;
        }
        if (s > 3) {
          illegal(inst);
          return;
        }
        // No overflow check here: writing the new frame's stack pointer is
        // itself a register access, and the check that runs before every
        // instruction has already spilled whatever was in the way.
        const uint32_t callInc = (ps_ & kPsCallInc) >> kPsCallIncShift;
        setA(callInc * 4 + (s & 3), A(s) - frame);
        rotateWindow(static_cast<int>(callInc));
        windowStart_ |= 1u << windowBase_;
        break;
      }
      if (m == 1) {  // BF/BT and the zero-overhead loops
        if (r == 0 || r == 1) {
          const bool value = ((br_ >> s) & 1) != 0;
          branchIf(value == (r == 1), signExtend(imm8, 8));
          break;
        }
        if (r >= 8 && r <= 10) {
          const uint32_t count = A(s);
          lbeg_ = next;
          lend_ = pc_ + 4 + imm8;
          const bool skip = (r == 9 && count == 0) ||
                            (r == 10 && static_cast<int32_t>(count) <= 0);
          lcount_ = skip ? 0 : count - 1;
          if (skip) {
            jump(lend_);
            return;
          }
          break;
        }
        illegal(inst);
        return;
      }
      // m == 2 or 3: BLTUI / BGEUI
      {
        const uint32_t value = A(s);
        const uint32_t immediate = kB4ConstU[r];
        branchIf(m == 2 ? value < immediate : value >= immediate, signExtend(imm8, 8));
      }
      break;
    }

    case 0x7: {  // B: the register-to-register branches
      const uint32_t a = A(s);
      const uint32_t b = A(t);
      const int32_t offset = signExtend(imm8, 8);
      bool taken = false;
      switch (r) {
        case 0x0: taken = (a & b) == 0; break;                                        // BNONE
        case 0x1: taken = a == b; break;                                              // BEQ
        case 0x2: taken = static_cast<int32_t>(a) < static_cast<int32_t>(b); break;   // BLT
        case 0x3: taken = a < b; break;                                               // BLTU
        case 0x4: taken = (a & b) == b; break;                                        // BALL
        case 0x5: taken = (a & (1u << (b & 31))) == 0; break;                         // BBC
        case 0x6:
        case 0x7: taken = (a & (1u << (((r & 1) << 4) | t))) == 0; break;              // BBCI
        case 0x8: taken = (a & b) != 0; break;                                        // BANY
        case 0x9: taken = a != b; break;                                              // BNE
        case 0xA: taken = static_cast<int32_t>(a) >= static_cast<int32_t>(b); break;  // BGE
        case 0xB: taken = a >= b; break;                                              // BGEU
        case 0xC: taken = (a & b) != b; break;                                        // BNALL
        case 0xD: taken = (a & (1u << (b & 31))) != 0; break;                         // BBS
        case 0xE:
        case 0xF: taken = (a & (1u << (((r & 1) << 4) | t))) != 0; break;              // BBSI
        default: break;
      }
      branchIf(taken, offset);
      break;
    }

    default:
      illegal(inst);
      return;
  }

  if (!tookBranch_) pc_ = next;
}

}  // namespace freeink::sim::emu
