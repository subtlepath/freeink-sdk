// FreeInk emulator — the SPI memory controller the flash hangs off.

#include "FlashController.h"

#include <algorithm>
#include <cstring>

namespace freeink::sim::emu {
namespace {

constexpr uint32_t kCmd = 0x000;
constexpr uint32_t kAddr = 0x004;
constexpr uint32_t kUser = 0x018;
constexpr uint32_t kUser1 = 0x01C;
constexpr uint32_t kUser2 = 0x020;
constexpr uint32_t kMosiDlen = 0x024;
constexpr uint32_t kMisoDlen = 0x028;
constexpr uint32_t kRdStatus = 0x02C;
constexpr uint32_t kFsm = 0x054;
constexpr uint32_t kW0 = 0x058;
constexpr uint32_t kWordCount = 16;

constexpr uint32_t kUsr = 1u << 18;
constexpr uint32_t kFlashRead = 1u << 31;
constexpr uint32_t kFlashWren = 1u << 30;
constexpr uint32_t kFlashWrdi = 1u << 29;
constexpr uint32_t kFlashRdid = 1u << 28;
constexpr uint32_t kFlashRdsr = 1u << 27;
constexpr uint32_t kFlashWrsr = 1u << 26;
constexpr uint32_t kFlashPp = 1u << 25;
constexpr uint32_t kFlashSe = 1u << 24;
constexpr uint32_t kFlashBe = 1u << 23;
constexpr uint32_t kFlashCe = 1u << 22;

constexpr uint32_t kUsrCommand = 1u << 31;
constexpr uint32_t kUsrAddr = 1u << 30;
constexpr uint32_t kUsrMiso = 1u << 28;
constexpr uint32_t kUsrMosi = 1u << 27;

// NOR command opcodes, as any SPI flash datasheet lists them.
constexpr uint8_t kOpWriteEnable = 0x06;
constexpr uint8_t kOpWriteDisable = 0x04;
constexpr uint8_t kOpReadStatus1 = 0x05;
constexpr uint8_t kOpReadStatus2 = 0x35;
constexpr uint8_t kOpWriteStatus1 = 0x01;
constexpr uint8_t kOpWriteStatus2 = 0x31;
constexpr uint8_t kOpReadId = 0x9F;
constexpr uint8_t kOpReadSfdp = 0x5A;
constexpr uint8_t kOpPageProgram = 0x02;
constexpr uint8_t kOpSectorErase = 0x20;
constexpr uint8_t kOpBlockErase32 = 0x52;
constexpr uint8_t kOpBlockErase64 = 0xD8;
constexpr uint8_t kOpChipErase = 0xC7;
constexpr uint8_t kOpChipEraseAlt = 0x60;
constexpr uint8_t kOpReleasePowerDown = 0xAB;
constexpr uint8_t kOpPowerDown = 0xB9;
constexpr uint8_t kOpEnableReset = 0x66;
constexpr uint8_t kOpReset = 0x99;
constexpr uint8_t kOpEnterQpi = 0x38;

bool isReadOpcode(uint8_t opcode) {
  switch (opcode) {
    case 0x03:  // read
    case 0x0B:  // fast read
    case 0x3B:  // dual output
    case 0x6B:  // quad output
    case 0xBB:  // dual I/O
    case 0xEB:  // quad I/O
    case 0x1B:
    case 0x0C:
    case 0x3C:
    case 0x6C:
    case 0xEC:
      return true;
    default:
      return false;
  }
}

}  // namespace

FlashControllerDevice::FlashControllerDevice(std::string name, uint32_t base, FlashImage& flash)
    : GenericPeripheral(std::move(name), base, 0x1000), flash_(flash) {}

uint32_t FlashControllerDevice::read(uint32_t offset) {
  if (offset >= kW0 && offset < kW0 + kWordCount * 4) return words_[(offset - kW0) / 4];
  if (offset == kRdStatus) return status_;
  if (offset == kFsm) return 0;  // the state machine is always idle by now
  if (offset == kCmd) {
    // Every command completes within the write that started it, so the start
    // bit is already clear. That is what lets the firmware's polling loop end.
    return GenericPeripheral::read(offset) &
           ~(kUsr | kFlashRead | kFlashWren | kFlashWrdi | kFlashRdid | kFlashRdsr | kFlashWrsr |
             kFlashPp | kFlashSe | kFlashBe | kFlashCe);
  }
  return GenericPeripheral::read(offset);
}

void FlashControllerDevice::write(uint32_t offset, uint32_t value, uint32_t mask) {
  if (offset >= kW0 && offset < kW0 + kWordCount * 4) {
    const uint32_t index = (offset - kW0) / 4;
    words_[index] = (words_[index] & ~mask) | (value & mask);
    return;
  }
  GenericPeripheral::write(offset, value, mask);
  if (offset != kCmd) return;

  const uint32_t command = GenericPeripheral::read(kCmd);
  if (command & kUsr) {
    runUserTransaction();
  } else {
    runDedicatedCommand(command);
  }
  // Clear the start bits: the operation is done.
  GenericPeripheral::write(kCmd, 0, 0xFFFFFFFFu);
}

void FlashControllerDevice::writeDataWords(const uint8_t* data, size_t length) {
  std::memset(words_, 0, sizeof(words_));
  std::memcpy(words_, data, std::min<size_t>(length, sizeof(words_)));
}

void FlashControllerDevice::readDataWords(uint8_t* data, size_t length) {
  std::memcpy(data, words_, std::min<size_t>(length, sizeof(words_)));
}

void FlashControllerDevice::runUserTransaction() {
  const uint32_t user = GenericPeripheral::read(kUser);
  const uint32_t user2 = GenericPeripheral::read(kUser2);
  const uint32_t address = GenericPeripheral::read(kAddr);
  const uint8_t opcode = static_cast<uint8_t>(user2 & 0xFF);

  // Address phase: the controller puts the address in the high bits of
  // SPI_MEM_ADDR_REG, left-justified by the configured address length.
  const uint32_t addressBits = ((GenericPeripheral::read(kUser1) >> 26) & 0x3F) + 1;
  const uint32_t flashAddress =
      (user & kUsrAddr) ? (addressBits >= 32 ? address : address >> (32 - addressBits)) : 0;

  const uint32_t misoBytes = (user & kUsrMiso) ? ((GenericPeripheral::read(kMisoDlen) & 0x3FF) + 1 + 7) / 8 : 0;
  const uint32_t mosiBytes = (user & kUsrMosi) ? ((GenericPeripheral::read(kMosiDlen) & 0x3FF) + 1 + 7) / 8 : 0;

  if (!(user & kUsrCommand)) return;

  switch (opcode) {
    case kOpReadId: {
      // Manufacturer, memory type, capacity — most significant byte first on
      // the wire, which lands in the low byte of W0.
      const uint8_t id[4] = {static_cast<uint8_t>(jedecId_ >> 16), static_cast<uint8_t>(jedecId_ >> 8),
                             static_cast<uint8_t>(jedecId_), 0};
      writeDataWords(id, sizeof(id));
      return;
    }
    case kOpReadStatus1: {
      const uint8_t value = static_cast<uint8_t>(status_ & 0xFF);
      writeDataWords(&value, 1);
      status_ = value;
      return;
    }
    case kOpReadStatus2: {
      // QE set: these boards run their flash in quad mode.
      const uint8_t value = 0x02;
      writeDataWords(&value, 1);
      return;
    }
    case kOpWriteStatus1:
    case kOpWriteStatus2:
      return;
    case kOpWriteEnable:
      writeEnabled_ = true;
      status_ |= 0x02;
      return;
    case kOpWriteDisable:
      writeEnabled_ = false;
      status_ &= ~0x02u;
      return;
    case kOpSectorErase:
      flash_.erase(flashAddress & ~0xFFFu, 4096);
      writeEnabled_ = false;
      return;
    case kOpBlockErase32:
      flash_.erase(flashAddress & ~0x7FFFu, 32 * 1024);
      writeEnabled_ = false;
      return;
    case kOpBlockErase64:
      flash_.erase(flashAddress & ~0xFFFFu, 64 * 1024);
      writeEnabled_ = false;
      return;
    case kOpChipErase:
    case kOpChipEraseAlt:
      flash_.erase(0, flash_.size());
      writeEnabled_ = false;
      return;
    case kOpPageProgram: {
      uint8_t buffer[kWordCount * 4];
      readDataWords(buffer, sizeof(buffer));
      flash_.write(flashAddress, buffer, std::min<size_t>(mosiBytes, sizeof(buffer)));
      writeEnabled_ = false;
      return;
    }
    case kOpReadSfdp: {
      // No SFDP table: a driver that probes for one falls back to its own
      // defaults, which is the same thing it does on a part without one.
      uint8_t buffer[kWordCount * 4] = {};
      writeDataWords(buffer, std::min<size_t>(misoBytes, sizeof(buffer)));
      return;
    }
    case kOpReleasePowerDown:
    case kOpPowerDown:
    case kOpEnableReset:
    case kOpReset:
    case kOpEnterQpi:
      return;
    default:
      if (isReadOpcode(opcode)) {
        uint8_t buffer[kWordCount * 4] = {};
        const size_t length = std::min<size_t>(misoBytes, sizeof(buffer));
        flash_.read(flashAddress, buffer, length);
        writeDataWords(buffer, length);
        return;
      }
      // An opcode with no model: the transaction "succeeds" and reads back
      // erased bytes, which is what an unimplemented command looks like on a
      // part that does not support it.
      if (misoBytes) {
        uint8_t buffer[kWordCount * 4];
        std::memset(buffer, 0xFF, sizeof(buffer));
        writeDataWords(buffer, std::min<size_t>(misoBytes, sizeof(buffer)));
      }
      return;
  }
}

void FlashControllerDevice::runDedicatedCommand(uint32_t command) {
  const uint32_t address = GenericPeripheral::read(kAddr) & 0x00FFFFFF;
  if (command & kFlashRdid) {
    status_ = jedecId_;
    const uint8_t id[4] = {static_cast<uint8_t>(jedecId_ >> 16), static_cast<uint8_t>(jedecId_ >> 8),
                           static_cast<uint8_t>(jedecId_), 0};
    writeDataWords(id, sizeof(id));
    return;
  }
  if (command & kFlashRdsr) {
    status_ = 0;
    return;
  }
  if (command & kFlashWren) {
    writeEnabled_ = true;
    return;
  }
  if (command & kFlashWrdi) {
    writeEnabled_ = false;
    return;
  }
  if (command & kFlashWrsr) return;
  if (command & kFlashRead) {
    uint8_t buffer[kWordCount * 4];
    flash_.read(address, buffer, sizeof(buffer));
    writeDataWords(buffer, sizeof(buffer));
    return;
  }
  if (command & kFlashPp) {
    uint8_t buffer[kWordCount * 4];
    readDataWords(buffer, sizeof(buffer));
    flash_.write(address, buffer, sizeof(buffer));
    return;
  }
  if (command & kFlashSe) {
    flash_.erase(address & ~0xFFFu, 4096);
    return;
  }
  if (command & kFlashBe) {
    flash_.erase(address & ~0xFFFFu, 64 * 1024);
    return;
  }
  if (command & kFlashCe) flash_.erase(0, flash_.size());
}

}  // namespace freeink::sim::emu
