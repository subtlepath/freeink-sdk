#pragma once

// FreeInk simulator — an SD card answering an SDMMC host.
//
// The X4 Pro's card is not on SPI: it sits on the chip's native SDMMC host,
// which exchanges the same SD commands as fields rather than as a byte stream.
// So this is the same card as SdSpiCard behind a different front door — the
// state machine an SD card runs (idle, ready, identification, standby,
// transfer) and the responses each command produces.
//
// Keeping the state machine real matters for the same reason it does in SPI
// mode: "is there a card?" is answered by whether this handshake completes,
// and a model that skips to "yes" cannot be used to test the path where the
// answer is no.

#include "VirtualCard.h"

#include <cstdint>
#include <vector>

namespace freeink::sim {

class SdNativeCard {
 public:
  explicit SdNativeCard(VirtualCard& card) : card_(card) {}

  // Runs one command. `response` is four words as the host's RESP0..RESP3 hold
  // them; short responses use the first. Returns false when the card does not
  // answer — an unknown command, or no card in the slot.
  bool command(int index, uint32_t argument, bool longResponse, uint32_t* response);
  void reset();

  // The block of card data the last command produced, if it was one of the
  // few that answer with data rather than off the medium. Taking it clears it.
  size_t takeData(uint8_t* out, size_t max);

  uint16_t rca() const { return rca_; }

 private:
  enum class State { Idle, Ready, Identification, Standby, Transfer };

  uint32_t statusWord() const;

  VirtualCard& card_;
  std::vector<uint8_t> pending_;
  State state_ = State::Idle;
  bool appCommand_ = false;
  uint16_t rca_ = 0x0001;
};

}  // namespace freeink::sim
