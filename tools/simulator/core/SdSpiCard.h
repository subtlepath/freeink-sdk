#pragma once

// FreeInk simulator — an SD card answering in SPI mode.
//
// On the X3 and X4 the card shares the panel's SPI bus with its own chip
// select, and the firmware drives it with the SD card protocol in its SPI
// form: 6-byte commands, R1/R3/R7 responses, a data token, 512 bytes, a CRC.
//
// The whole init handshake is here — CMD0 to enter idle, CMD8 to establish
// that this is a version 2 card, ACMD41 until it leaves idle, CMD58 for the
// capacity bit — because that handshake is exactly what a firmware's "is there
// a card?" answer is made of. Skipping it and reporting a mounted card would
// make the one thing worth testing untestable.
//
// This is a byte-stream device: the SPI model hands it every byte clocked
// while its chip select is asserted, and it returns what the card would put on
// MISO for that byte.

#include "VirtualCard.h"

#include <cstdint>
#include <vector>

namespace freeink::sim {

class SdSpiCard {
 public:
  explicit SdSpiCard(VirtualCard& card) : card_(card) {}

  // One byte clocked out by the host; returns the byte the card drives back.
  uint8_t transfer(uint8_t mosi);
  // The host deasserted the chip select: any half-finished command is dropped,
  // which is what happens on the wire.
  void deselect();
  void reset();

 private:
  enum class Phase {
    Command,       // collecting the six bytes of a command
    Response,      // driving a queued response
    ReadData,      // driving a data token and a block
    WriteWaiting,  // waiting for the host's data token
    WriteData,     // taking a block from the host
  };

  void execute();
  void queue(const uint8_t* bytes, size_t length);
  void queueR1(uint8_t status);

  VirtualCard& card_;
  Phase phase_ = Phase::Command;
  std::vector<uint8_t> command_;
  std::vector<uint8_t> outgoing_;
  size_t outgoingCursor_ = 0;
  std::vector<uint8_t> incoming_;
  uint32_t writeLba_ = 0;
  bool multiWrite_ = false;
  bool multiRead_ = false;
  uint32_t readLba_ = 0;
  // Set when a write command has been answered and the card is now waiting for
  // the host's data token rather than for another command.
  bool pendingWrite_ = false;
  bool idle_ = true;
  bool appCommand_ = false;
  bool initialised_ = false;
};

}  // namespace freeink::sim
