// FreeInk simulator — the board an emulated image runs on.

#include "EmuBoard.h"

#include "I2cDevices.h"
#include "Machine.h"
#include "Panel.h"

#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>

namespace freeink::sim {

EmuBoard::EmuBoard(Machine& machine, const std::string& chip)
    : machine_(machine), chip_(chip), sdCard_(machine.card()), sdNative_(machine.card()) {}

bool EmuBoard::sdPresent() { return machine_.card().present(); }

bool EmuBoard::sdCommand(int index, uint32_t argument, bool longResponse, uint32_t* response) {
  const bool answered = sdNative_.command(index, argument, longResponse, response);
  if (trace_) {
    char buf[96];
    std::snprintf(buf, sizeof(buf), "sdmmc CMD%-2d arg=0x%08X -> %s 0x%08X", index, argument,
                  answered ? "ok" : "no answer", response[0]);
    trace_(buf);
  }
  return answered;
}

size_t EmuBoard::sdCardData(uint8_t* out, size_t max) { return sdNative_.takeData(out, max); }

bool EmuBoard::sdReadBlocks(uint32_t lba, uint32_t blocks, uint8_t* out) {
  return machine_.card().readBlocks(lba, blocks, out);
}

bool EmuBoard::sdWriteBlocks(uint32_t lba, uint32_t blocks, const uint8_t* in) {
  return machine_.card().writeBlocks(lba, blocks, in);
}

void EmuBoard::refreshMode(int pin) {
  if (pin < 0 || pin >= 64) return;
  const PinState& state = pins_[pin];
  uint32_t mode = state.output ? FSIM_PIN_OUTPUT : FSIM_PIN_INPUT;
  if (state.pullUp) mode |= FSIM_PIN_PULLUP;
  if (state.pullDown) mode |= FSIM_PIN_PULLDOWN;
  std::lock_guard<std::mutex> lock(machine_.mutex());
  machine_.gpio().setMode(pin, mode);
}

void EmuBoard::pinPull(int pin, bool pullUp, bool pullDown) {
  if (pin < 0 || pin >= 64) return;
  pins_[pin].pullUp = pullUp;
  pins_[pin].pullDown = pullDown;
  refreshMode(pin);
}

void EmuBoard::pinDirection(int pin, bool output) {
  if (pin < 0 || pin >= 64) return;
  pins_[pin].output = output;
  refreshMode(pin);
}

void EmuBoard::pinWrite(int pin, int level) {
  // Raising the card's chip select ends whatever command was in flight.
  const fsim_board_desc& sdBoard = machine_.board();
  if (machine_.boardKnown() && sdBoard.sd_cs >= 0 && pin == sdBoard.sd_cs && level) {
    sdCard_.deselect();
  }
  int before = 0;
  int after = 0;
  {
    std::lock_guard<std::mutex> lock(machine_.mutex());
    Gpio& gpio = machine_.gpio();
    before = gpio.read(pin);
    gpio.write(pin, level);
    after = gpio.read(pin);
  }

  // The panel's reset line is wired for real: a LOW pulse resets the
  // controller, discarding any half-programmed window or standing command.
  const fsim_board_desc& board = machine_.board();
  if (machine_.boardKnown() && board.epd_rst >= 0 && pin == board.epd_rst && before == 1 &&
      after == 0) {
    machine_.panel().hardwareReset();
  }
}

int EmuBoard::pinRead(int pin, bool observed) {
  // Only a read the firmware actually made counts as sampling the pin. The
  // machine's own sweep for interrupt edges looks at every pad constantly, and
  // counting that would let a scripted press be released before the firmware
  // ever looked.
  if (observed) machine_.noteGpioRead(pin);
  std::lock_guard<std::mutex> lock(machine_.mutex());
  return machine_.gpio().read(pin);
}

namespace {

std::string hexBytes(const uint8_t* data, size_t length, size_t limit = 12) {
  std::string out;
  char buf[8];
  for (size_t index = 0; index < length && index < limit; ++index) {
    std::snprintf(buf, sizeof(buf), "%02X ", data[index]);
    out += buf;
  }
  if (length > limit) out += "...";
  if (!out.empty() && out.back() == ' ') out.pop_back();
  return out;
}

}  // namespace

bool EmuBoard::i2cTransfer(int bus, uint8_t addr, const uint8_t* tx, size_t txLen, uint8_t* rx,
                           size_t rxLen) {
  const bool acked = machine_.i2c().transfer(bus, addr, tx, txLen, rx, rxLen);
  if (trace_) {
    std::string line = "i2c" + std::to_string(bus) + " 0x";
    char buf[8];
    std::snprintf(buf, sizeof(buf), "%02X", addr);
    line += buf;
    if (txLen) line += " w[" + hexBytes(tx, txLen) + "]";
    if (rxLen) line += " r[" + (acked ? hexBytes(rx, rxLen) : std::string()) + "]";
    line += acked ? " ack" : " NACK";
    trace_(line);
  }
  return acked;
}

void EmuBoard::i2cConfigured(int bus, int sda, int scl, uint32_t hz) {
  machine_.i2c().begin(bus, sda, scl, hz);
}

bool EmuBoard::selected(int csPin, bool defaultWhenUnset) const {
  if (csPin < 0) return defaultWhenUnset;
  std::lock_guard<std::mutex> lock(machine_.mutex());
  Gpio& gpio = machine_.gpio();
  // A pad the firmware has never configured as an output is not being driven
  // by software at all — either the SPI peripheral's own chip select is on it,
  // or the device is not in use yet. Which of those it is depends on the
  // device, so the caller says what to assume.
  if (!(gpio.mode(csPin) & FSIM_PIN_OUTPUT)) return defaultWhenUnset;
  return gpio.read(csPin) == 0;
}

void EmuBoard::spiTransfer(int bus, const uint8_t* tx, uint8_t* rx, size_t len) {
  (void)bus;
  if (len == 0) return;
  const fsim_board_desc& board = machine_.board();
  if (!machine_.boardKnown()) {
    if (rx) std::memset(rx, 0xFF, len);
    return;
  }

  // The card shares the panel's bus on these boards, behind its own chip
  // select. Its select is never assumed: a card the firmware has not addressed
  // must not eat the panel's traffic.
  if (board.sd_bus_width == 0 && selected(board.sd_cs, false)) {
    std::vector<uint8_t> answer(len);
    for (size_t index = 0; index < len; ++index) {
      answer[index] = sdCard_.transfer(tx ? tx[index] : 0xFF);
      if (rx) rx[index] = answer[index];
    }
    if (trace_) {
      trace_("sd   " + std::to_string(len) + "B w[" + hexBytes(tx, len) + "] r[" +
             hexBytes(answer.data(), len) + "]");
    }
    return;
  }

  if (board.epd_dc < 0 || !selected(board.epd_cs, true)) {
    if (rx) std::memset(rx, 0xFF, len);
    return;
  }

  // Command or data is decided by the D/C pad at the moment the bytes go out,
  // exactly as the glass decides it. A driver that forgets to lower D/C before
  // a command byte writes pixels instead, here as on device.
  bool isData = true;
  {
    std::lock_guard<std::mutex> lock(machine_.mutex());
    isData = machine_.gpio().read(board.epd_dc) != 0;
  }

  if (trace_) {
    trace_("spi" + std::to_string(bus) + (isData ? " data " : " cmd  ") + std::to_string(len) +
           "B [" + hexBytes(tx, len) + "]");
  }

  Panel& panel = machine_.panel();
  for (size_t index = 0; index < len; ++index) {
    if (tx) panel.spiByte(tx[index], isData);
    if (rx) rx[index] = panel.spiReadByte();
  }
}

int EmuBoard::adcPin(int unit, int channel) const {
  // The channel-to-pad mapping is silicon, not board: ADC1 starts at GPIO0 on
  // the C3 and at GPIO1 on the S3, where ADC2 continues from GPIO11.
  if (chip_ == "esp32c3") {
    if (unit == 1 && channel >= 0 && channel <= 4) return channel;
    return -1;
  }
  if (chip_ == "esp32s3") {
    if (unit == 1 && channel >= 0 && channel <= 9) return channel + 1;
    if (unit == 2 && channel >= 0 && channel <= 9) return channel + 11;
    return -1;
  }
  return -1;
}

uint32_t EmuBoard::adcSample(int unit, int channel) {
  const int pin = adcPin(unit, channel);
  if (pin < 0) return 4095;
  machine_.noteAdcRead(pin);
  std::lock_guard<std::mutex> lock(machine_.mutex());
  return static_cast<uint32_t>(machine_.adc().readRaw(pin));
}

}  // namespace freeink::sim
