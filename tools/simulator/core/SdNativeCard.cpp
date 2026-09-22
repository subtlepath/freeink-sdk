// FreeInk simulator — an SD card answering an SDMMC host.

#include "SdNativeCard.h"

#include <algorithm>
#include <cstring>

namespace freeink::sim {
namespace {

// Packs a 128-bit card register into the four response words the host holds,
// most significant word last, which is the order RESP0..RESP3 are read in.
void packLongResponse(const uint8_t* reg, uint32_t* response) {
  for (int word = 0; word < 4; ++word) {
    const uint8_t* at = reg + (3 - word) * 4;
    response[word] = (static_cast<uint32_t>(at[0]) << 24) | (static_cast<uint32_t>(at[1]) << 16) |
                     (static_cast<uint32_t>(at[2]) << 8) | at[3];
  }
}

// Card status bits, as an R1 carries them.
constexpr uint32_t kStatusReadyForData = 1u << 8;
constexpr uint32_t kStatusAppCmd = 1u << 5;

uint32_t stateBits(int state) { return static_cast<uint32_t>(state) << 9; }

}  // namespace

void SdNativeCard::reset() {
  state_ = State::Idle;
  appCommand_ = false;
  pending_.clear();
}

size_t SdNativeCard::takeData(uint8_t* out, size_t max) {
  const size_t length = std::min(max, pending_.size());
  if (length) std::memcpy(out, pending_.data(), length);
  pending_.clear();
  return length;
}

uint32_t SdNativeCard::statusWord() const {
  int current = 0;
  switch (state_) {
    case State::Idle: current = 0; break;
    case State::Ready: current = 1; break;
    case State::Identification: current = 2; break;
    case State::Standby: current = 3; break;
    case State::Transfer: current = 4; break;
  }
  uint32_t status = kStatusReadyForData | stateBits(current);
  if (appCommand_) status |= kStatusAppCmd;
  return status;
}

bool SdNativeCard::command(int index, uint32_t argument, bool longResponse, uint32_t* response) {
  std::memset(response, 0, 4 * sizeof(uint32_t));
  if (!card_.present()) return false;

  const bool app = appCommand_;
  appCommand_ = false;

  if (app) {
    switch (index) {
      case 6:  // SET_BUS_WIDTH — the host has already decided; the card agrees
        response[0] = statusWord();
        return true;
      case 41: {  // SD_SEND_OP_COND: the initialisation poll
        // The host sends this until the card reports it has finished powering
        // up. Answering "done" on the first one is not a shortcut — there is
        // no power-up to wait for here — and the host's loop terminates on it
        // exactly as it does on a fast card.
        state_ = State::Ready;
        response[0] = card_.ocr();
        return true;
      }
      case 13:  // SD_STATUS — a 64-byte block, all of it optional to the host
        pending_.assign(64, 0);
        response[0] = statusWord();
        return true;
      case 51: {  // SEND_SCR — answered as an eight-byte block of data
        // Physical layer 2.00, one- and four-bit buses, spec version 3
        // supported: the shape of a card an ESP-IDF host is happy with.
        static const uint8_t kScr[8] = {0x02, 0x35, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00};
        pending_.assign(kScr, kScr + sizeof(kScr));
        response[0] = statusWord();
        return true;
      }
      default:
        return false;
    }
  }

  switch (index) {
    case 0:  // GO_IDLE_STATE — no response
      state_ = State::Idle;
      return true;
    case 2:  // ALL_SEND_CID
      state_ = State::Identification;
      if (longResponse) packLongResponse(card_.cid(), response);
      return true;
    case 3:  // SEND_RELATIVE_ADDR — an R6: the address and a trimmed status
      state_ = State::Standby;
      response[0] = (static_cast<uint32_t>(rca_) << 16) | 0x0500;
      return true;
    case 6: {  // SWITCH_FUNC — a 64-byte status describing what the card can do
      pending_.assign(64, 0);
      // Function group 1 (bus speed) supports its default mode and high speed;
      // whichever the host asked for is reported back as selected. A host that
      // finds high speed unsupported simply stays at the default, so getting
      // this wrong is slow rather than broken.
      pending_[13] = 0x03;
      pending_[16] = static_cast<uint8_t>(argument & 0x0F);
      response[0] = statusWord();
      return true;
    }
    case 7:  // SELECT_CARD
      state_ = (argument >> 16) == rca_ ? State::Transfer : State::Standby;
      response[0] = statusWord();
      return true;
    case 8: {  // SEND_IF_COND — an R7, echoing the voltage and check pattern
      response[0] = argument & 0xFFF;
      return true;
    }
    case 9:  // SEND_CSD
      if (longResponse) packLongResponse(card_.csd(), response);
      return true;
    case 10:  // SEND_CID
      if (longResponse) packLongResponse(card_.cid(), response);
      return true;
    case 12:  // STOP_TRANSMISSION
    case 13:  // SEND_STATUS
    case 16:  // SET_BLOCKLEN
    case 17:  // READ_SINGLE_BLOCK
    case 18:  // READ_MULTIPLE_BLOCK
    case 23:  // SET_BLOCK_COUNT
    case 24:  // WRITE_BLOCK
    case 25:  // WRITE_MULTIPLE_BLOCK
      response[0] = statusWord();
      return true;
    case 55:  // APP_CMD
      appCommand_ = true;
      response[0] = statusWord() | kStatusAppCmd;
      return true;
    default:
      return false;
  }
}

}  // namespace freeink::sim
