// FreeInk emulator — RV32IMC core, as fitted to the ESP32-C3.

#include "RiscvCore.h"

#include "Rom.h"

#include <algorithm>
#include <cstdio>
#include <cstring>

namespace freeink::sim::emu {
namespace {

// Trap causes (RISC-V privileged spec, machine mode).
constexpr uint32_t kIllegalInstruction = 2;
constexpr uint32_t kBreakpoint = 3;
constexpr uint32_t kEcallFromMachine = 11;

constexpr uint32_t kMstatusMie = 1u << 3;
constexpr uint32_t kMstatusMpie = 1u << 7;
constexpr uint32_t kMstatusMpp = 3u << 11;

inline int32_t sext(uint32_t value, int bits) {
  const uint32_t sign = 1u << (bits - 1);
  return static_cast<int32_t>((value ^ sign) - sign);
}

inline uint32_t bits(uint32_t value, int high, int low) {
  return (value >> low) & ((1u << (high - low + 1)) - 1);
}

}  // namespace

RiscvCore::RiscvCore(Bus& bus, const SocDesc& soc, Rom* rom) : bus_(bus), soc_(soc), rom_(rom) {}

void RiscvCore::reset(uint32_t entry) {
  std::memset(x_, 0, sizeof(x_));
  pc_ = entry;
  retired_ = 0;
  mstatus_ = 0;
  mtvec_ = 0;
  mepc_ = 0;
  mcause_ = 0;
  mtval_ = 0;
  mie_ = 0;
  mip_ = 0;
  threshold_ = 0;
  cycleOffset_ = 0;
  intPriority_.fill(0);
  otherCsr_.fill(0);
  halt_ = HaltReason::Running;
  haltDetail_.clear();
  // The app is entered the way the bootloader enters it: running on the mask
  // ROM's stack. It stays there until FreeRTOS starts and moves each task onto
  // its own, so this is not a placeholder — early startup really does put its
  // frames here, and anywhere else would overlap memory the ROM owns.
  x_[2] = soc_.romStack.base + soc_.romStack.size;
}

void RiscvCore::stop(HaltReason reason, const std::string& detail) {
  if (halt_ != HaltReason::Running && halt_ != HaltReason::WaitingForInterrupt) return;
  halt_ = reason;
  haltDetail_ = detail;
}

void RiscvCore::resume() {
  if (halt_ == HaltReason::WaitingForInterrupt) {
    halt_ = HaltReason::Running;
    haltDetail_.clear();
  }
}

uint32_t RiscvCore::arg(int index) const {
  if (index < 0) return 0;
  // a0-a7, then the caller's outgoing stack area — the same order the ABI
  // spills in, so a ROM routine with many arguments reads them correctly.
  if (index < 8) return x_[10 + index];
  return const_cast<Bus&>(bus_).read32(x_[2] + static_cast<uint32_t>(index - 8) * 4);
}
void RiscvCore::setArg(int index, uint32_t value) {
  if (index >= 0 && index < 8) x_[10 + index] = value;
}
void RiscvCore::setReturn(uint32_t value) { x_[10] = value; }
void RiscvCore::setReturn64(uint64_t value) {
  x_[10] = static_cast<uint32_t>(value);
  x_[11] = static_cast<uint32_t>(value >> 32);
}

void RiscvCore::setInterruptPending(int line, bool pending) {
  if (line <= 0 || line >= 32) return;
  const uint32_t mask = 1u << line;
  if (pending) {
    mip_ |= mask;
  } else {
    mip_ &= ~mask;
  }
  if (pending && halt_ == HaltReason::WaitingForInterrupt) resume();
}

void RiscvCore::setInterruptPriority(int line, uint32_t priority) {
  if (line > 0 && line < 32) intPriority_[line] = priority;
}

bool RiscvCore::interruptsEnabled() const { return (mstatus_ & kMstatusMie) != 0; }

void RiscvCore::trap(uint32_t cause, uint32_t tval) {
  if (cause & 0x80000000u) {
    ++interrupts_;
  } else {
    ++exceptions_;
  }
  if (!firstException_.seen && !(cause & 0x80000000u)) {
    firstException_.seen = true;
    firstException_.cause = cause;
    firstException_.pc = pc_;
    firstException_.tval = tval;
    firstException_.trace = recentCalls();
  }
  mepc_ = pc_;
  mcause_ = cause;
  mtval_ = tval;
  mstatus_ = (mstatus_ & ~kMstatusMpie) | ((mstatus_ & kMstatusMie) ? kMstatusMpie : 0);
  mstatus_ &= ~kMstatusMie;
  mstatus_ |= kMstatusMpp;  // we only ever run in machine mode

  const uint32_t base = mtvec_ & ~3u;
  const bool vectored = (mtvec_ & 3u) == 1;
  const bool isInterrupt = (cause & 0x80000000u) != 0;
  pc_ = (vectored && isInterrupt) ? base + 4 * (cause & 0x7FFFFFFFu) : base;

  if (base == 0) {
    stop(HaltReason::Fault,
         "trap with mtvec unset (cause " + std::to_string(cause & 0x7FFFFFFFu) +
             (isInterrupt ? ", interrupt" : ", exception") + ") — the firmware trapped before it "
             "installed a handler");
  }
}

bool RiscvCore::serviceInterrupt() {
  if (!(mstatus_ & kMstatusMie)) return false;
  // On the C3 the per-line enable is not the `mie` CSR — it is the interrupt
  // matrix's own CPU_INT_ENABLE register, which the matrix model has already
  // applied before asserting a line here. Requiring `mie` bits as well is what
  // a stock RISC-V core would do, and it would mask every interrupt this chip
  // ever raises: IDF never sets them.
  uint32_t ready = mip_;
  if (!ready) return false;
  // Highest line number first, then priority against the matrix threshold.
  // A line is allowed through when its priority is at or above the threshold:
  // IDF enables interrupts globally by setting the threshold to 1 and gives
  // almost everything — the scheduler tick included — priority 1, so a
  // strictly-greater test would mask the entire system.
  for (int line = 31; line >= 1; --line) {
    if (!(ready & (1u << line))) continue;
    if (intPriority_[line] < threshold_) continue;
    ++lineCounts_[line];
    trap(0x80000000u | static_cast<uint32_t>(line), 0);
    return true;
  }
  return false;
}

void RiscvCore::illegal(uint32_t instruction) {
  char buf[128];
  std::snprintf(buf, sizeof(buf), "illegal instruction 0x%08X at 0x%08X", instruction, pc_);
  // Trap first: an IDF app installs a handler that panics with a backtrace,
  // which is far more useful than the emulator's own message.
  if ((mtvec_ & ~3u) != 0) {
    trap(kIllegalInstruction, instruction);
  } else {
    stop(HaltReason::Unimplemented, buf);
  }
}

bool RiscvCore::stepOnce() {
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

  serviceInterrupt();
  if (halt_ != HaltReason::Running) return false;

  // A call into mask ROM is intercepted and run natively — the emulator has
  // no ROM bytes, by design (see Rom.cpp).
  if (inRange(soc_.rom, pc_)) {
    if (!rom_ || !rom_->call(*this, pc_)) {
      char buf[160];
      std::snprintf(buf, sizeof(buf), "call into unimplemented mask ROM at 0x%08X (ra=0x%08X)", pc_, x_[1]);
      stop(HaltReason::Unimplemented, buf);
      return false;
    }
    ++retired_;
    return true;
  }

  bus_.setPc(pc_);
  const uint32_t lowHalf = bus_.read16(pc_);
  if (bus_.faulted()) return false;

  if ((lowHalf & 3u) != 3u) {
    executeCompressed(static_cast<uint16_t>(lowHalf));
  } else {
    const uint32_t instruction = lowHalf | (static_cast<uint32_t>(bus_.read16(pc_ + 2)) << 16);
    if (bus_.faulted()) return false;
    execute(instruction, 4);
  }
  ++retired_;
  return true;
}

uint64_t RiscvCore::run(uint64_t budget) {
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

bool RiscvCore::callGuest(uint32_t function, const uint32_t* args, int argCount, uint32_t* result) {
  // The return address is a word inside the ROM window that no routine lives
  // at. Reaching it is how the core knows the callback has returned, and it
  // cannot collide with real code because the whole window is intercepted.
  const uint32_t sentinel = soc_.rom.base + soc_.rom.size - 4;

  uint32_t saved[32];
  std::memcpy(saved, x_, sizeof(saved));
  const uint32_t savedPc = pc_;

  for (int i = 0; i < argCount && i < 8; ++i) x_[10 + i] = args[i];
  x_[1] = sentinel;
  pc_ = function;

  // Generous, but bounded: a callback that never returns is a firmware bug,
  // and hanging the emulator would report it as a hang in the tool.
  constexpr uint64_t kMaxCallbackInstructions = 20000000;
  uint64_t steps = 0;
  bool ok = true;
  while (pc_ != sentinel) {
    if (steps++ >= kMaxCallbackInstructions) {
      stop(HaltReason::Fault, "a firmware callback invoked from a ROM routine did not return");
      ok = false;
      break;
    }
    if (!stepOnce()) {
      ok = false;
      break;
    }
  }

  if (result) *result = x_[10];
  std::memcpy(x_, saved, sizeof(saved));
  pc_ = savedPc;
  return ok;
}

void RiscvCore::execute(uint32_t instruction, uint32_t length) {
  const uint32_t opcode = instruction & 0x7F;
  const uint32_t rd = bits(instruction, 11, 7);
  const uint32_t rs1 = bits(instruction, 19, 15);
  const uint32_t rs2 = bits(instruction, 24, 20);
  const uint32_t funct3 = bits(instruction, 14, 12);
  const uint32_t funct7 = bits(instruction, 31, 25);
  const uint32_t next = pc_ + length;

  auto set = [&](uint32_t value) {
    if (rd) x_[rd] = value;
  };

  switch (opcode) {
    case 0x37:  // LUI
      set(instruction & 0xFFFFF000u);
      pc_ = next;
      return;
    case 0x17:  // AUIPC
      set(pc_ + (instruction & 0xFFFFF000u));
      pc_ = next;
      return;
    case 0x6F: {  // JAL
      const uint32_t imm = (bits(instruction, 31, 31) << 20) | (bits(instruction, 19, 12) << 12) |
                           (bits(instruction, 20, 20) << 11) | (bits(instruction, 30, 21) << 1);
      set(next);
      noteTransfer(pc_, pc_ + static_cast<uint32_t>(sext(imm, 21)));
      pc_ = pc_ + static_cast<uint32_t>(sext(imm, 21));
      return;
    }
    case 0x67: {  // JALR
      const uint32_t target = (x_[rs1] + static_cast<uint32_t>(sext(bits(instruction, 31, 20), 12))) & ~1u;
      set(next);
      noteTransfer(pc_, target);
      pc_ = target;
      return;
    }
    case 0x63: {  // branches
      const uint32_t imm = (bits(instruction, 31, 31) << 12) | (bits(instruction, 7, 7) << 11) |
                           (bits(instruction, 30, 25) << 5) | (bits(instruction, 11, 8) << 1);
      const int32_t a = static_cast<int32_t>(x_[rs1]);
      const int32_t b = static_cast<int32_t>(x_[rs2]);
      bool taken = false;
      switch (funct3) {
        case 0: taken = a == b; break;
        case 1: taken = a != b; break;
        case 4: taken = a < b; break;
        case 5: taken = a >= b; break;
        case 6: taken = x_[rs1] < x_[rs2]; break;
        case 7: taken = x_[rs1] >= x_[rs2]; break;
        default: illegal(instruction); return;
      }
      pc_ = taken ? pc_ + static_cast<uint32_t>(sext(imm, 13)) : next;
      return;
    }
    case 0x03: {  // loads
      const uint32_t addr = x_[rs1] + static_cast<uint32_t>(sext(bits(instruction, 31, 20), 12));
      switch (funct3) {
        case 0: set(static_cast<uint32_t>(sext(bus_.read8(addr), 8))); break;
        case 1: set(static_cast<uint32_t>(sext(bus_.read16(addr), 16))); break;
        case 2: set(bus_.read32(addr)); break;
        case 4: set(bus_.read8(addr)); break;
        case 5: set(bus_.read16(addr)); break;
        default: illegal(instruction); return;
      }
      pc_ = next;
      return;
    }
    case 0x23: {  // stores
      const uint32_t imm = (funct7 << 5) | rd;
      const uint32_t addr = x_[rs1] + static_cast<uint32_t>(sext(imm, 12));
      switch (funct3) {
        case 0: bus_.write8(addr, static_cast<uint8_t>(x_[rs2])); break;
        case 1: bus_.write16(addr, static_cast<uint16_t>(x_[rs2])); break;
        case 2: bus_.write32(addr, x_[rs2]); break;
        default: illegal(instruction); return;
      }
      pc_ = next;
      return;
    }
    case 0x13: {  // register-immediate
      const int32_t imm = sext(bits(instruction, 31, 20), 12);
      const uint32_t shamt = rs2;
      switch (funct3) {
        case 0: set(x_[rs1] + static_cast<uint32_t>(imm)); break;
        case 1:
          if (funct7 != 0) { illegal(instruction); return; }
          set(x_[rs1] << shamt);
          break;
        case 2: set(static_cast<int32_t>(x_[rs1]) < imm ? 1 : 0); break;
        case 3: set(x_[rs1] < static_cast<uint32_t>(imm) ? 1 : 0); break;
        case 4: set(x_[rs1] ^ static_cast<uint32_t>(imm)); break;
        case 5:
          if (funct7 == 0) {
            set(x_[rs1] >> shamt);
          } else if (funct7 == 0x20) {
            set(static_cast<uint32_t>(static_cast<int32_t>(x_[rs1]) >> shamt));
          } else {
            illegal(instruction);
            return;
          }
          break;
        case 6: set(x_[rs1] | static_cast<uint32_t>(imm)); break;
        case 7: set(x_[rs1] & static_cast<uint32_t>(imm)); break;
        default: illegal(instruction); return;
      }
      pc_ = next;
      return;
    }
    case 0x33: {  // register-register, including M
      const uint32_t a = x_[rs1];
      const uint32_t b = x_[rs2];
      if (funct7 == 0x01) {
        const int32_t sa = static_cast<int32_t>(a);
        const int32_t sb = static_cast<int32_t>(b);
        switch (funct3) {
          case 0: set(static_cast<uint32_t>(sa * sb)); break;
          case 1: set(static_cast<uint32_t>((static_cast<int64_t>(sa) * sb) >> 32)); break;
          case 2:
            // MULHSU: signed times unsigned. Widening the unsigned operand
            // through uint64 before making it signed keeps its value; letting
            // the mixed expression convert to uint64 would lose the sign of
            // the first.
            set(static_cast<uint32_t>(
                static_cast<uint64_t>(static_cast<int64_t>(sa) *
                                      static_cast<int64_t>(static_cast<uint64_t>(b))) >>
                32));
            break;
          case 3:
            set(static_cast<uint32_t>((static_cast<uint64_t>(a) * b) >> 32));
            break;
          // Division by zero and the signed overflow case are defined by the
          // spec to produce specific values rather than trap.
          case 4: set(b == 0 ? 0xFFFFFFFFu : (sa == INT32_MIN && sb == -1 ? a : static_cast<uint32_t>(sa / sb))); break;
          case 5: set(b == 0 ? 0xFFFFFFFFu : a / b); break;
          case 6: set(b == 0 ? a : (sa == INT32_MIN && sb == -1 ? 0 : static_cast<uint32_t>(sa % sb))); break;
          case 7: set(b == 0 ? a : a % b); break;
          default: illegal(instruction); return;
        }
        pc_ = next;
        return;
      }
      switch (funct3) {
        case 0: set(funct7 == 0x20 ? a - b : a + b); break;
        case 1: set(a << (b & 31)); break;
        case 2: set(static_cast<int32_t>(a) < static_cast<int32_t>(b) ? 1 : 0); break;
        case 3: set(a < b ? 1 : 0); break;
        case 4: set(a ^ b); break;
        case 5:
          set(funct7 == 0x20 ? static_cast<uint32_t>(static_cast<int32_t>(a) >> (b & 31)) : a >> (b & 31));
          break;
        case 6: set(a | b); break;
        case 7: set(a & b); break;
        default: illegal(instruction); return;
      }
      pc_ = next;
      return;
    }
    case 0x0F:  // fence / fence.i — nothing to order in an in-order interpreter
      pc_ = next;
      return;
    case 0x73: {  // system
      if (funct3 == 0) {
        const uint32_t imm = bits(instruction, 31, 20);
        if (imm == 0) {  // ecall
          trap(kEcallFromMachine, 0);
          return;
        }
        if (imm == 1) {  // ebreak
          if ((mtvec_ & ~3u) != 0) {
            trap(kBreakpoint, pc_);
          } else {
            stop(HaltReason::Breakpoint, "ebreak with no handler installed");
          }
          return;
        }
        if (imm == 0x302) {  // mret
          mstatus_ = (mstatus_ & ~kMstatusMie) | ((mstatus_ & kMstatusMpie) ? kMstatusMie : 0);
          mstatus_ |= kMstatusMpie;
          pc_ = mepc_;
          return;
        }
        if (imm == 0x105) {  // wfi
          if (!(mip_ & mie_)) {
            halt_ = HaltReason::WaitingForInterrupt;
            haltDetail_ = "wfi";
          }
          pc_ = next;
          return;
        }
        if (imm == 0x120 || imm == 0x104) {  // sfence.vma / sret — no S-mode here
          pc_ = next;
          return;
        }
        illegal(instruction);
        return;
      }

      const uint32_t csr = bits(instruction, 31, 20);
      bool ok = true;
      const uint32_t operand = (funct3 & 4) ? rs1 : x_[rs1];
      // CSRRS/CSRRC with rs1 = x0 (or a zero immediate) must not write at all:
      // the distinction matters for registers whose write has a side effect,
      // such as the ones a peripheral clears on write.
      const bool writes = (funct3 & 3) == 1 || rs1 != 0;
      const uint32_t old = rd || (funct3 & 3) != 1 ? readCsr(csr, &ok) : 0;
      if (!ok) {
        illegal(instruction);
        return;
      }
      uint32_t updated = old;
      switch (funct3 & 3) {
        case 1: updated = operand; break;
        case 2: updated = old | operand; break;
        case 3: updated = old & ~operand; break;
        default: illegal(instruction); return;
      }
      if (writes) {
        writeCsr(csr, updated, &ok);
        if (!ok) {
          illegal(instruction);
          return;
        }
      }
      set(old);
      pc_ = next;
      return;
    }
    default:
      illegal(instruction);
      return;
  }
}

void RiscvCore::executeCompressed(uint16_t instruction) {
  const uint32_t op = instruction & 3u;
  const uint32_t funct3 = bits(instruction, 15, 13);
  const uint32_t next = pc_ + 2;
  auto rdPrime = [&]() { return 8 + bits(instruction, 4, 2); };
  auto rs1Prime = [&]() { return 8 + bits(instruction, 9, 7); };

  if (instruction == 0) {
    illegal(instruction);  // an all-zero halfword is never a valid instruction
    return;
  }

  if (op == 0) {
    switch (funct3) {
      case 0: {  // C.ADDI4SPN
        const uint32_t imm = (bits(instruction, 10, 7) << 6) | (bits(instruction, 12, 11) << 4) |
                             (bits(instruction, 5, 5) << 3) | (bits(instruction, 6, 6) << 2);
        if (imm == 0) {
          illegal(instruction);
          return;
        }
        x_[rdPrime()] = x_[2] + imm;
        pc_ = next;
        return;
      }
      case 2: {  // C.LW
        const uint32_t imm = (bits(instruction, 5, 5) << 6) | (bits(instruction, 12, 10) << 3) |
                             (bits(instruction, 6, 6) << 2);
        x_[rdPrime()] = bus_.read32(x_[rs1Prime()] + imm);
        pc_ = next;
        return;
      }
      case 6: {  // C.SW
        const uint32_t imm = (bits(instruction, 5, 5) << 6) | (bits(instruction, 12, 10) << 3) |
                             (bits(instruction, 6, 6) << 2);
        bus_.write32(x_[rs1Prime()] + imm, x_[rdPrime()]);
        pc_ = next;
        return;
      }
      default:
        illegal(instruction);  // the float forms; the C3 has no FPU
        return;
    }
  }

  if (op == 1) {
    switch (funct3) {
      case 0: {  // C.NOP / C.ADDI
        const uint32_t rd = bits(instruction, 11, 7);
        const int32_t imm = sext((bits(instruction, 12, 12) << 5) | bits(instruction, 6, 2), 6);
        if (rd) x_[rd] += static_cast<uint32_t>(imm);
        pc_ = next;
        return;
      }
      case 1: {  // C.JAL (RV32 only)
        const uint32_t imm = (bits(instruction, 12, 12) << 11) | (bits(instruction, 8, 8) << 10) |
                             (bits(instruction, 10, 9) << 8) | (bits(instruction, 6, 6) << 7) |
                             (bits(instruction, 7, 7) << 6) | (bits(instruction, 2, 2) << 5) |
                             (bits(instruction, 11, 11) << 4) | (bits(instruction, 5, 3) << 1);
        x_[1] = next;
        noteTransfer(pc_, pc_ + static_cast<uint32_t>(sext(imm, 12)));
        pc_ = pc_ + static_cast<uint32_t>(sext(imm, 12));
        return;
      }
      case 2: {  // C.LI
        const uint32_t rd = bits(instruction, 11, 7);
        const int32_t imm = sext((bits(instruction, 12, 12) << 5) | bits(instruction, 6, 2), 6);
        if (rd) x_[rd] = static_cast<uint32_t>(imm);
        pc_ = next;
        return;
      }
      case 3: {  // C.ADDI16SP / C.LUI
        const uint32_t rd = bits(instruction, 11, 7);
        if (rd == 2) {
          const uint32_t imm = (bits(instruction, 12, 12) << 9) | (bits(instruction, 4, 3) << 7) |
                               (bits(instruction, 5, 5) << 6) | (bits(instruction, 2, 2) << 5) |
                               (bits(instruction, 6, 6) << 4);
          if (imm == 0) {
            illegal(instruction);
            return;
          }
          x_[2] += static_cast<uint32_t>(sext(imm, 10));
        } else if (rd != 0) {
          const uint32_t imm = (bits(instruction, 12, 12) << 5) | bits(instruction, 6, 2);
          if (imm == 0) {
            illegal(instruction);
            return;
          }
          x_[rd] = static_cast<uint32_t>(sext(imm, 6)) << 12;
        }
        pc_ = next;
        return;
      }
      case 4: {  // C.SRLI / C.SRAI / C.ANDI / register forms
        const uint32_t which = bits(instruction, 11, 10);
        const uint32_t reg = rs1Prime();
        const uint32_t shamt = (bits(instruction, 12, 12) << 5) | bits(instruction, 6, 2);
        if (which == 0) {
          x_[reg] >>= shamt;
        } else if (which == 1) {
          x_[reg] = static_cast<uint32_t>(static_cast<int32_t>(x_[reg]) >> shamt);
        } else if (which == 2) {
          x_[reg] &= static_cast<uint32_t>(sext(shamt, 6));
        } else {
          const uint32_t other = rdPrime();
          switch (bits(instruction, 6, 5)) {
            case 0: x_[reg] -= x_[other]; break;
            case 1: x_[reg] ^= x_[other]; break;
            case 2: x_[reg] |= x_[other]; break;
            case 3: x_[reg] &= x_[other]; break;
            default: break;
          }
        }
        pc_ = next;
        return;
      }
      case 5: {  // C.J
        const uint32_t imm = (bits(instruction, 12, 12) << 11) | (bits(instruction, 8, 8) << 10) |
                             (bits(instruction, 10, 9) << 8) | (bits(instruction, 6, 6) << 7) |
                             (bits(instruction, 7, 7) << 6) | (bits(instruction, 2, 2) << 5) |
                             (bits(instruction, 11, 11) << 4) | (bits(instruction, 5, 3) << 1);
        pc_ = pc_ + static_cast<uint32_t>(sext(imm, 12));
        return;
      }
      case 6:
      case 7: {  // C.BEQZ / C.BNEZ
        const uint32_t imm = (bits(instruction, 12, 12) << 8) | (bits(instruction, 6, 5) << 6) |
                             (bits(instruction, 2, 2) << 5) | (bits(instruction, 11, 10) << 3) |
                             (bits(instruction, 4, 3) << 1);
        const bool zero = x_[rs1Prime()] == 0;
        const bool taken = funct3 == 6 ? zero : !zero;
        pc_ = taken ? pc_ + static_cast<uint32_t>(sext(imm, 9)) : next;
        return;
      }
      default:
        illegal(instruction);
        return;
    }
  }

  // op == 2
  switch (funct3) {
    case 0: {  // C.SLLI
      const uint32_t rd = bits(instruction, 11, 7);
      const uint32_t shamt = (bits(instruction, 12, 12) << 5) | bits(instruction, 6, 2);
      if (rd) x_[rd] <<= shamt;
      pc_ = next;
      return;
    }
    case 2: {  // C.LWSP
      const uint32_t rd = bits(instruction, 11, 7);
      if (rd == 0) {
        illegal(instruction);
        return;
      }
      const uint32_t imm = (bits(instruction, 3, 2) << 6) | (bits(instruction, 12, 12) << 5) |
                           (bits(instruction, 6, 4) << 2);
      x_[rd] = bus_.read32(x_[2] + imm);
      pc_ = next;
      return;
    }
    case 4: {  // C.JR / C.MV / C.EBREAK / C.JALR / C.ADD
      const uint32_t rd = bits(instruction, 11, 7);
      const uint32_t rs2 = bits(instruction, 6, 2);
      const bool high = bits(instruction, 12, 12) != 0;
      if (!high) {
        if (rs2 == 0) {
          if (rd == 0) {
            illegal(instruction);
            return;
          }
          noteTransfer(pc_, x_[rd] & ~1u);  // C.JR
          pc_ = x_[rd] & ~1u;
          return;
        }
        if (rd) x_[rd] = x_[rs2];  // C.MV
        pc_ = next;
        return;
      }
      if (rs2 == 0) {
        if (rd == 0) {  // C.EBREAK
          if ((mtvec_ & ~3u) != 0) {
            trap(kBreakpoint, pc_);
          } else {
            stop(HaltReason::Breakpoint, "c.ebreak with no handler installed");
          }
          return;
        }
        const uint32_t target = x_[rd] & ~1u;  // C.JALR
        x_[1] = next;
        noteTransfer(pc_, target);
        pc_ = target;
        return;
      }
      if (rd) x_[rd] += x_[rs2];  // C.ADD
      pc_ = next;
      return;
    }
    case 6: {  // C.SWSP
      const uint32_t imm = (bits(instruction, 8, 7) << 6) | (bits(instruction, 12, 9) << 2);
      bus_.write32(x_[2] + imm, x_[bits(instruction, 6, 2)]);
      pc_ = next;
      return;
    }
    default:
      illegal(instruction);
      return;
  }
}

uint32_t RiscvCore::readCsr(uint32_t csr, bool* ok) {
  *ok = true;
  switch (csr) {
    case 0x300: return mstatus_;
    case 0x301: return (1u << 30) | (1u << 8) | (1u << 12) | (1u << 2);  // misa: RV32IMC
    case 0x304: return mie_;
    case 0x305: return mtvec_;
    case 0x340: return mscratch_;
    case 0x341: return mepc_;
    case 0x342: return mcause_;
    case 0x343: return mtval_;
    case 0x344: return mip_;
    case 0xF14: return 0;  // mhartid — single core
    // The C3's machine performance counter. IDF reads it as the CPU cycle
    // count — `esp_cpu_get_cycle_count()` — and firmware busy-waits on it
    // advancing, so it has to be a running counter rather than storage.
    case 0x7E2: return static_cast<uint32_t>(retired_) + cycleOffset_;
    case 0xB00:            // mcycle
    case 0xC00:            // cycle
    case 0xC01:            // time
    case 0xC02:            // instret
    case 0xB02:
      return static_cast<uint32_t>(retired_);
    case 0xB80:
    case 0xC80:
    case 0xC81:
    case 0xC82:
    case 0xB82:
      return static_cast<uint32_t>(retired_ >> 32);
    default:
      // Everything else — PMP, performance counters, the C3's vendor CSRs —
      // reads back what was written. An app configures these at startup and
      // never depends on the side effect, and refusing them would turn a
      // working boot into a spurious illegal-instruction panic.
      return otherCsr_[csr & 0xFFF];
  }
}

void RiscvCore::writeCsr(uint32_t csr, uint32_t value, bool* ok) {
  *ok = true;
  switch (csr) {
    case 0x300: mstatus_ = value; return;
    case 0x304: mie_ = value; return;
    case 0x305: mtvec_ = value; return;
    case 0x340: mscratch_ = value; return;
    case 0x341: mepc_ = value; return;
    case 0x342: mcause_ = value; return;
    case 0x343: mtval_ = value; return;
    case 0x344: mip_ = value; return;
    case 0x7E2:
      // Writing the counter rebases it rather than stopping it.
      cycleOffset_ = value - static_cast<uint32_t>(retired_);
      return;
    case 0x301:
    case 0xF14:
      return;  // read-only
    default:
      otherCsr_[csr & 0xFFF] = value;
      return;
  }
}

std::vector<std::pair<uint32_t, uint32_t>> RiscvCore::recentCalls() const {
  std::vector<std::pair<uint32_t, uint32_t>> out;
  const size_t count = std::min(callCursor_, kCallRing);
  for (size_t i = 0; i < count; ++i) {
    out.push_back(calls_[(callCursor_ - count + i) % kCallRing]);
  }
  return out;
}

std::string RiscvCore::describeState() const {
  char buf[512];
  std::snprintf(buf, sizeof(buf),
                "pc=0x%08X ra=0x%08X sp=0x%08X gp=0x%08X\n  "
                "a0=0x%08X a1=0x%08X a2=0x%08X a3=0x%08X a4=0x%08X a5=0x%08X a6=0x%08X a7=0x%08X\n  "
                "s0=0x%08X s1=0x%08X mstatus=0x%08X mcause=0x%08X mepc=0x%08X mtvec=0x%08X",
                pc_, x_[1], x_[2], x_[3], x_[10], x_[11], x_[12], x_[13], x_[14], x_[15], x_[16], x_[17],
                x_[8], x_[9], mstatus_, mcause_, mepc_, mtvec_);
  return buf;
}

}  // namespace freeink::sim::emu
