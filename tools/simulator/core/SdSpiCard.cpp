// FreeInk simulator — an SD card answering in SPI mode.

#include "SdSpiCard.h"

#include <cstring>

namespace freeink::sim {
namespace {

constexpr uint8_t kR1Idle = 0x01;
constexpr uint8_t kR1Ready = 0x00;
constexpr uint8_t kR1IllegalCommand = 0x04;
constexpr uint8_t kDataStartToken = 0xFE;
constexpr uint8_t kMultiWriteToken = 0xFC;
constexpr uint8_t kStopTranToken = 0xFD;
constexpr uint8_t kDataAccepted = 0x05;

uint32_t argumentOf(const std::vector<uint8_t>& command) {
  return (static_cast<uint32_t>(command[1]) << 24) | (static_cast<uint32_t>(command[2]) << 16) |
         (static_cast<uint32_t>(command[3]) << 8) | command[4];
}

}  // namespace

void SdSpiCard::reset() {
  phase_ = Phase::Command;
  command_.clear();
  outgoing_.clear();
  outgoingCursor_ = 0;
  incoming_.clear();
  pendingWrite_ = false;
  idle_ = true;
  appCommand_ = false;
  initialised_ = false;
  multiRead_ = false;
  multiWrite_ = false;
}

void SdSpiCard::deselect() {
  // A command in the middle of being clocked is abandoned; a queued response
  // is not, because a host is entitled to raise the select again and keep
  // reading it. Only the command framing resets.
  command_.clear();
  if (phase_ == Phase::Command) return;
}

void SdSpiCard::queue(const uint8_t* bytes, size_t length) {
  outgoing_.assign(bytes, bytes + length);
  outgoingCursor_ = 0;
  phase_ = Phase::Response;
}

void SdSpiCard::queueR1(uint8_t status) {
  // The card takes a byte or two to answer; a host clocks 0xFF until the top
  // bit clears, so leading 0xFF is part of the protocol rather than padding.
  const uint8_t response[2] = {0xFF, status};
  queue(response, sizeof(response));
}

uint8_t SdSpiCard::transfer(uint8_t mosi) {
  switch (phase_) {
    case Phase::Response: {
      const uint8_t value = outgoing_[outgoingCursor_++];
      if (outgoingCursor_ >= outgoing_.size()) {
        outgoing_.clear();
        outgoingCursor_ = 0;
        phase_ = pendingWrite_ ? Phase::WriteWaiting : Phase::Command;
        pendingWrite_ = false;
      }
      return value;
    }
    case Phase::ReadData: {
      // A host stops a multi-block read by sending CMD12 *while the card is
      // still streaming* — it does not wait for a gap, and there is none. So
      // the command framing runs in parallel with the data, exactly as the
      // card's own does.
      if (multiRead_) {
        if (command_.empty() && (mosi & 0xC0) == 0x40) command_.push_back(mosi);
        else if (!command_.empty()) command_.push_back(mosi);
        if (command_.size() >= 6) {
          execute();
          return 0xFF;
        }
      }
      const uint8_t value = outgoing_[outgoingCursor_++];
      if (outgoingCursor_ >= outgoing_.size()) {
        outgoing_.clear();
        outgoingCursor_ = 0;
        if (multiRead_) {
          // A multi-block read keeps streaming until the host sends CMD12.
          std::vector<uint8_t> block(2 + VirtualCard::kSectorSize + 2, 0xFF);
          block[0] = 0xFF;
          block[1] = kDataStartToken;
          card_.readBlocks(++readLba_, 1, block.data() + 2);
          outgoing_ = std::move(block);
          outgoingCursor_ = 0;
        } else {
          phase_ = Phase::Command;
        }
      }
      return value;
    }
    case Phase::WriteWaiting: {
      if (mosi == kDataStartToken || mosi == kMultiWriteToken) {
        incoming_.clear();
        incoming_.reserve(VirtualCard::kSectorSize + 2);
        phase_ = Phase::WriteData;
      } else if (mosi == kStopTranToken) {
        multiWrite_ = false;
        phase_ = Phase::Command;
      }
      return 0xFF;
    }
    case Phase::WriteData: {
      incoming_.push_back(mosi);
      if (incoming_.size() < VirtualCard::kSectorSize + 2) return 0xFF;
      card_.writeBlocks(writeLba_++, 1, incoming_.data());
      incoming_.clear();
      // The data-response token, then "not busy" — the host clocks until it
      // sees a non-zero byte. A multi-block write then waits for the next
      // token rather than for a command.
      const uint8_t response[3] = {kDataAccepted, 0x00, 0xFF};
      queue(response, sizeof(response));
      pendingWrite_ = multiWrite_;
      return 0xFF;
    }
    case Phase::Command:
      break;
  }

  // Idle clocking: the host sends 0xFF between commands, and a command always
  // starts with the two top bits 01.
  if (command_.empty() && (mosi & 0xC0) != 0x40) return 0xFF;
  command_.push_back(mosi);
  if (command_.size() < 6) return 0xFF;
  execute();
  return 0xFF;
}

void SdSpiCard::execute() {
  const uint8_t index = command_[0] & 0x3F;
  const uint32_t argument = argumentOf(command_);
  const bool app = appCommand_;
  appCommand_ = false;
  command_.clear();

  if (!card_.present()) {
    // No card: the bus floats high and the host times out, which is what the
    // firmware's "no card" path is written against.
    const uint8_t none[1] = {0xFF};
    queue(none, sizeof(none));
    return;
  }

  if (app) {
    switch (index) {
      case 41:  // ACMD41: initialise, and report when it is done
        idle_ = false;
        initialised_ = true;
        queueR1(kR1Ready);
        return;
      case 51: {  // ACMD51: the SCR register, 8 bytes in a data block
        std::vector<uint8_t> out = {0xFF, kR1Ready, 0xFF, kDataStartToken,
                                    0x02, 0x35, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF};
        queue(out.data(), out.size());
        phase_ = Phase::Response;
        return;
      }
      case 13:  // ACMD13: SD status
        queueR1(kR1Ready);
        return;
      default:
        queueR1(kR1IllegalCommand);
        return;
    }
  }

  switch (index) {
    case 0:  // GO_IDLE_STATE
      idle_ = true;
      initialised_ = false;
      multiRead_ = multiWrite_ = false;
      queueR1(kR1Idle);
      return;
    case 8: {  // SEND_IF_COND — an R7: the R1 byte then the echoed argument
      const uint8_t out[6] = {0xFF, kR1Idle, 0x00, 0x00,
                              static_cast<uint8_t>((argument >> 8) & 0xFF),
                              static_cast<uint8_t>(argument & 0xFF)};
      queue(out, sizeof(out));
      return;
    }
    case 9:    // SEND_CSD
    case 10: { // SEND_CID
      std::vector<uint8_t> out = {0xFF, kR1Ready, 0xFF, kDataStartToken};
      const uint8_t* source = index == 9 ? card_.csd() : card_.cid();
      out.insert(out.end(), source, source + 16);
      out.push_back(0xFF);
      out.push_back(0xFF);
      queue(out.data(), out.size());
      return;
    }
    case 12:  // STOP_TRANSMISSION
      multiRead_ = false;
      outgoing_.clear();
      outgoingCursor_ = 0;
      queueR1(kR1Ready);
      return;
    case 13:  // SEND_STATUS — an R2: two bytes, both clear
    case 58: {  // READ_OCR — an R3: the R1 byte then the OCR
      if (index == 13) {
        const uint8_t out[3] = {0xFF, kR1Ready, 0x00};
        queue(out, sizeof(out));
        return;
      }
      const uint32_t ocr = card_.ocr();
      const uint8_t out[6] = {0xFF, static_cast<uint8_t>(idle_ ? kR1Idle : kR1Ready),
                              static_cast<uint8_t>(ocr >> 24), static_cast<uint8_t>(ocr >> 16),
                              static_cast<uint8_t>(ocr >> 8), static_cast<uint8_t>(ocr)};
      queue(out, sizeof(out));
      return;
    }
    case 16:  // SET_BLOCKLEN — always 512 on a high-capacity card
    case 55:  // APP_CMD
      if (index == 55) appCommand_ = true;
      queueR1(idle_ && index == 55 ? kR1Idle : kR1Ready);
      return;
    case 59:  // CRC_ON_OFF
      queueR1(kR1Ready);
      return;
    case 17:    // READ_SINGLE_BLOCK
    case 18: {  // READ_MULTIPLE_BLOCK
      readLba_ = argument;
      multiRead_ = index == 18;
      std::vector<uint8_t> out(2 + 2 + VirtualCard::kSectorSize + 2, 0xFF);
      out[0] = 0xFF;
      out[1] = kR1Ready;
      out[2] = 0xFF;
      out[3] = kDataStartToken;
      card_.readBlocks(readLba_, 1, out.data() + 4);
      outgoing_ = std::move(out);
      outgoingCursor_ = 0;
      phase_ = Phase::ReadData;
      return;
    }
    case 24:    // WRITE_BLOCK
    case 25: {  // WRITE_MULTIPLE_BLOCK
      writeLba_ = argument;
      multiWrite_ = index == 25;
      const uint8_t out[2] = {0xFF, kR1Ready};
      outgoing_.assign(out, out + sizeof(out));
      outgoingCursor_ = 0;
      // The response is clocked first, and the host's data token follows.
      phase_ = Phase::Response;
      // After the response drains the card waits for the token; the transfer
      // path moves it there when the response runs out.
      incoming_.clear();
      pendingWrite_ = true;
      return;
    }
    default:
      queueR1(kR1IllegalCommand);
      return;
  }
}

}  // namespace freeink::sim
