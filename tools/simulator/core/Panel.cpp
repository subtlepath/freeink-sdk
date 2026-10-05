// FreeInk simulator — virtual e-paper panel and controller decoding.

#include "Panel.h"

#include "Machine.h"

#include <algorithm>
#include <condition_variable>
#include <cstring>
#include <memory>

namespace freeink::sim {
namespace {

// Waveform durations, in milliseconds. These are the timings the FreeInk docs
// and drivers assume for these panels; they are what makes a firmware's
// "refresh is slow, show a spinner" logic testable. They are not measured
// per-unit values.
constexpr uint32_t kFullRefreshMs = 1600;
constexpr uint32_t kPartialRefreshMs = 800;
constexpr uint32_t kFastRefreshMs = 320;

const char* ssd1677CommandName(uint8_t cmd) {
  switch (cmd) {
    case 0x01: return "DRIVER_OUTPUT_CONTROL";
    case 0x03: return "GATE_VOLTAGE";
    case 0x04: return "SOURCE_VOLTAGE";
    case 0x0C: return "BOOSTER_SOFT_START";
    case 0x10: return "DEEP_SLEEP";
    case 0x11: return "DATA_ENTRY_MODE";
    case 0x12: return "SOFT_RESET";
    case 0x18: return "TEMP_SENSOR_CONTROL";
    case 0x1A: return "WRITE_TEMP";
    case 0x20: return "MASTER_ACTIVATION";
    case 0x21: return "DISPLAY_UPDATE_CTRL1";
    case 0x22: return "DISPLAY_UPDATE_CTRL2";
    case 0x24: return "WRITE_RAM_BW";
    case 0x26: return "WRITE_RAM_RED";
    case 0x2C: return "WRITE_VCOM";
    case 0x32: return "WRITE_LUT";
    case 0x3C: return "BORDER_WAVEFORM";
    case 0x44: return "SET_RAM_X_RANGE";
    case 0x45: return "SET_RAM_Y_RANGE";
    case 0x46: return "AUTO_WRITE_BW_RAM";
    case 0x47: return "AUTO_WRITE_RED_RAM";
    case 0x4E: return "SET_RAM_X_COUNTER";
    case 0x4F: return "SET_RAM_Y_COUNTER";
    default: return "?";
  }
}

const char* uc81xxCommandName(uint8_t cmd) {
  switch (cmd) {
    case 0x00: return "PSR";
    case 0x01: return "PWR";
    case 0x02: return "POF";
    case 0x03: return "PFS";
    case 0x04: return "PON";
    case 0x06: return "BTST";
    case 0x07: return "DSLP";
    case 0x10: return "DTM1";
    case 0x11: return "DSP";
    case 0x12: return "DRF";
    case 0x13: return "DTM2";
    case 0x20: return "LUT_VCOM";
    case 0x21: return "LUT_WW";
    case 0x22: return "LUT_BW";
    case 0x23: return "LUT_WB";
    case 0x24: return "LUT_BB";
    case 0x30: return "PLL";
    case 0x50: return "CDI";
    case 0x61: return "TRES";
    case 0x65: return "GSST";
    case 0x70: return "VER";
    case 0x71: return "FLG";
    case 0x82: return "VCOM_DC";
    case 0x90: return "PTL";
    case 0x91: return "PTIN";
    case 0x92: return "PTOUT";
    case 0xE0: return "CCSET";
    case 0xE1: return "GATE_SCAN";
    case 0xE3: return "PWS";
    case 0xE5: return "TSSET";
    default: return "?";
  }
}

// ── SSD1677 ──────────────────────────────────────────────────────────────────
// Byte-addressed X, row-addressed Y, an explicit window and cursor, and a
// two-step refresh (0x22 selects the sequence, 0x20 runs it).
class Ssd1677Controller : public PanelController {
 public:
  explicit Ssd1677Controller(Panel& panel) : panel_(panel) {}

  const char* name() const override { return "SSD1677"; }

  void command(uint8_t cmd) override {
    // A new command ends the previous one's data phase.
    flushPending();
    cmd_ = cmd;
    args_.clear();

    switch (cmd) {
      case 0x12:  // SOFT_RESET
        reset();
        panel_.recordEvent(cmd, nullptr, 0, ssd1677CommandName(cmd));
        break;
      case 0x24:  // WRITE_RAM_BW
        plane_ = 0;
        ramWrite_ = true;
        ramBytes_ = 0;
        break;
      case 0x26:  // WRITE_RAM_RED
        plane_ = 1;
        ramWrite_ = true;
        ramBytes_ = 0;
        break;
      case 0x20:  // MASTER_ACTIVATION — runs the sequence 0x22 selected
        runActivation();
        break;
      default:
        ramWrite_ = false;
        break;
    }
  }

  void data(const uint8_t* bytes, size_t len) override {
    if (ramWrite_) {
      panel_.writeRam(bytes, len, plane_);
      ramBytes_ += len;
      return;
    }
    args_.insert(args_.end(), bytes, bytes + len);
    applyArgs();
  }

  void reset() override {
    cmd_ = 0;
    args_.clear();
    ramWrite_ = false;
    updateSequence_ = 0;
    updateControl1_ = 0;
    customLut_ = false;
    panel_.setGrayEncoding(GrayEncoding::Mono);
  }

 private:
  void flushPending() {
    if (ramWrite_ && ramBytes_ > 0) {
      panel_.recordEvent(plane_ == 0 ? 0x24 : 0x26, nullptr, ramBytes_,
                         plane_ == 0 ? "WRITE_RAM_BW" : "WRITE_RAM_RED");
      ramBytes_ = 0;
    }
    ramWrite_ = false;
  }

  void applyArgs() {
    switch (cmd_) {
      case 0x11:  // DATA_ENTRY_MODE
        if (args_.size() >= 1) panel_.setDataEntryMode(args_[0]);
        break;
      case 0x44:  // SET_RAM_X_RANGE — start, end, each little-endian 16-bit, in bytes
        if (args_.size() >= 4) {
          xStart_ = args_[0] | (args_[1] << 8);
          xEnd_ = args_[2] | (args_[3] << 8);
          pushWindow();
        }
        break;
      case 0x45:  // SET_RAM_Y_RANGE — start, end, in rows
        if (args_.size() >= 4) {
          yStart_ = args_[0] | (args_[1] << 8);
          yEnd_ = args_[2] | (args_[3] << 8);
          pushWindow();
        }
        break;
      case 0x4E:  // SET_RAM_X_COUNTER
        if (args_.size() >= 2) {
          cursorX_ = args_[0] | (args_[1] << 8);
          panel_.setRamCursor(cursorX_ * 8, cursorY_);
        }
        break;
      case 0x4F:  // SET_RAM_Y_COUNTER
        if (args_.size() >= 2) {
          cursorY_ = args_[0] | (args_[1] << 8);
          panel_.setRamCursor(cursorX_ * 8, cursorY_);
        }
        break;
      case 0x22:  // DISPLAY_UPDATE_CTRL2 — selects the waveform for 0x20
        if (args_.size() >= 1) updateSequence_ = args_[0];
        break;
      case 0x21:  // DISPLAY_UPDATE_CTRL1 — RED RAM bypass
        if (args_.size() >= 1) updateControl1_ = args_[0];
        break;
      case 0x32:  // WRITE_LUT — 50 voltage bytes, 50 timing bytes, 5 frame rates
        if (args_.size() >= 105) {
          customLut_ = true;
          // The AA bank leaves channel 00 undriven, preserving the B/W base.
          // The factory bank drives all four targets with inverted polarity.
          overlayLut_ = std::all_of(args_.begin(), args_.begin() + 10,
                                    [](uint8_t byte) { return byte == 0; });
        }
        break;
      default:
        break;
    }
    if (args_.size() <= 8) panel_.recordEvent(cmd_, args_.data(), args_.size(), ssd1677CommandName(cmd_));
  }

  void pushWindow() {
    // The window is expressed with X in bytes; the model works in pixels. The
    // driver may program the range descending (mirrorX), so normalize.
    const int x0 = std::min(xStart_, xEnd_) * 8;
    const int x1 = std::max(xStart_, xEnd_) * 8 + 7;
    const int y0 = std::min(yStart_, yEnd_);
    const int y1 = std::max(yStart_, yEnd_);
    panel_.setRamWindow(x0, y0, x1, y1);
  }

  void runActivation() {
    panel_.recordEvent(0x20, &updateSequence_, 1, "MASTER_ACTIVATION");
    // The low two bits of the 0x22 sequence are the power-off phases; a
    // sequence with no display phase (0xC0 power-on, 0x03 power-off) moves no
    // pixels, so it must not publish a frame.
    const bool hasDisplayPhase = (updateSequence_ & 0x14) != 0 || updateSequence_ == 0xF7 ||
                                 updateSequence_ == 0xC7 || (updateSequence_ & 0xF0) == 0xF0;
    if (!hasDisplayPhase) {
      panel_.setPowered((updateSequence_ & 0xC0) != 0);
      return;
    }
    // Loading OTP replaces an earlier external LUT. Otherwise use the two RAM
    // planes according to the uploaded bank, unless CTRL1 bypasses RED.
    if (updateSequence_ & 0x10) customLut_ = false;
    panel_.setGrayEncoding(customLut_ && !(updateControl1_ & 0x40)
                              ? (overlayLut_ ? GrayEncoding::Ssd1677Overlay : GrayEncoding::Ssd1677Absolute)
                              : GrayEncoding::Mono);
    // Vendor sequences distinguish the waveform: 0xCC/0xFC/0xD4-style partial
    // (DU) sequences are the fast path, 0xF7/0xC7 the full one.
    RefreshKind kind = RefreshKind::Full;
    uint32_t ms = kFullRefreshMs;
    if (updateSequence_ == 0xCC || updateSequence_ == 0xFC || updateSequence_ == 0xD4 ||
        (updateSequence_ & 0x08) != 0) {
      kind = RefreshKind::Fast;
      ms = kFastRefreshMs;
    }
    panel_.startRefresh(kind, ms);
  }

  Panel& panel_;
  uint8_t cmd_ = 0;
  std::vector<uint8_t> args_;
  bool ramWrite_ = false;
  size_t ramBytes_ = 0;
  int plane_ = 0;
  int xStart_ = 0, xEnd_ = 0, yStart_ = 0, yEnd_ = 0;
  int cursorX_ = 0, cursorY_ = 0;
  uint8_t updateSequence_ = 0;
  uint8_t updateControl1_ = 0;
  bool customLut_ = false;
  bool overlayLut_ = false;
};

// ── UC81xx (UC8253 / UC8179 / UC8279) ────────────────────────────────────────
// KW mode: DTM1 (0x10) carries the OLD plane and DTM2 (0x13) the NEW one, both
// streamed full-frame from the top-left with no cursor. A partial window (0x90
// + 0x91) narrows what the next stream covers.
class Uc81xxController : public PanelController {
 public:
  Uc81xxController(Panel& panel, const char* name, bool answersVersionProbe)
      : panel_(panel), name_(name), answersVersionProbe_(answersVersionProbe) {}

  const char* name() const override { return name_; }

  void command(uint8_t cmd) override {
    flushPending();
    cmd_ = cmd;
    args_.clear();
    streaming_ = false;

    switch (cmd) {
      case 0x10:  // DTM1 — old plane
        streaming_ = true;
        plane_ = 1;
        streamed_ = 0;
        resetStreamCursor();
        break;
      case 0x13:  // DTM2 — new plane
        streaming_ = true;
        plane_ = 0;
        streamed_ = 0;
        resetStreamCursor();
        break;
      case 0x12:  // DRF — display refresh
        runRefresh();
        break;
      case 0x04:  // PON
        panel_.setPowered(true);
        panel_.recordEvent(cmd, nullptr, 0, uc81xxCommandName(cmd));
        break;
      case 0x02:  // POF
        panel_.setPowered(false);
        panel_.recordEvent(cmd, nullptr, 0, uc81xxCommandName(cmd));
        break;
      case 0x92:  // PTOUT — leave partial mode, window covers the whole panel
        partial_ = false;
        panel_.setRamWindow(0, 0, panel_.width() - 1, panel_.height() - 1);
        panel_.recordEvent(cmd, nullptr, 0, uc81xxCommandName(cmd));
        break;
      case 0x91:  // PTIN
        partial_ = true;
        panel_.recordEvent(cmd, nullptr, 0, uc81xxCommandName(cmd));
        break;
      case 0x70:  // VER — the probe that tells UC8179 from UC8279 from SSD1677
        readIndex_ = 0;
        break;
      default:
        break;
    }
  }

  void data(const uint8_t* bytes, size_t len) override {
    if (streaming_) {
      panel_.writeRam(bytes, len, plane_);
      streamed_ += len;
      return;
    }
    args_.insert(args_.end(), bytes, bytes + len);
    applyArgs();
  }

  // SSD1677 has no VER/FLG registers and floats the bus; the UC parts answer.
  // XteinkDetect's probe depends on exactly this difference, so the model has
  // to reproduce it or controller autodetection is untestable.
  uint8_t readByte() override {
    if (!answersVersionProbe_) return 0xFF;
    // Byte 2 (LUT_VER) separates UC8179 (0x02) from UC8279 (0x68).
    static const uint8_t kVersion[4] = {0x00, 0x00, 0x00, 0x00};
    uint8_t value = kVersion[readIndex_ & 0x03];
    if (readIndex_ == 2) value = lutVersion_;
    ++readIndex_;
    return value;
  }

  void setLutVersion(uint8_t version) { lutVersion_ = version; }

  void reset() override {
    cmd_ = 0;
    args_.clear();
    streaming_ = false;
    partial_ = false;
    readIndex_ = 0;
    passiveBlackLut_ = false;
  }

 private:
  void flushPending() {
    if (streaming_ && streamed_ > 0) {
      panel_.recordEvent(plane_ == 1 ? 0x10 : 0x13, nullptr, streamed_, plane_ == 1 ? "DTM1" : "DTM2");
      streamed_ = 0;
    }
    streaming_ = false;
  }

  void resetStreamCursor() {
    // A KW stream always restarts at the window's top-left.
    if (!partial_) panel_.setRamWindow(0, 0, panel_.width() - 1, panel_.height() - 1);
    panel_.setDataEntryMode(0x03);  // X increment, Y increment
    panel_.setRamCursor(partialX0_, partialY0_);
    if (!partial_) panel_.setRamCursor(0, 0);
  }

  void applyArgs() {
    switch (cmd_) {
      case 0x24:  // LUT_BB — voltage selectors every six bytes, then timings
        if (!answersVersionProbe_ && args_.size() >= 42) {
          passiveBlackLut_ = true;
          for (size_t i = 0; i < 42; i += 6) {
            if (args_[i] != 0) passiveBlackLut_ = false;
          }
        }
        break;
      case 0x61:  // TRES — resolution
        if (args_.size() >= 4) {
          const int w = (args_[0] << 8) | args_[1];
          const int h = (args_[2] << 8) | args_[3];
          if (w > 0 && h > 0) panel_.recordEvent(cmd_, args_.data(), args_.size(), "TRES");
        }
        break;
      case 0x90:  // PTL — partial window: xStart, xEnd (byte-aligned), yStart, yEnd
        if (args_.size() >= 7) {
          const int x0 = ((args_[0] << 8) | args_[1]);
          const int x1 = ((args_[2] << 8) | args_[3]);
          const int y0 = ((args_[4] << 8) | args_[5]);
          const int y1 = args_.size() >= 8 ? ((args_[6] << 8) | args_[7]) : y0;
          partialX0_ = std::min(x0, x1);
          partialY0_ = std::min(y0, y1);
          panel_.setRamWindow(partialX0_, partialY0_, std::max(x0, x1), std::max(y0, y1));
        }
        break;
      default:
        break;
    }
    if (args_.size() <= 8) panel_.recordEvent(cmd_, args_.data(), args_.size(), uc81xxCommandName(cmd_));
  }

  void runRefresh() {
    panel_.recordEvent(0x12, nullptr, 0, "DRF");
    // The X3 AA nudge bank leaves BB (00) and WB (10 in old/new order)
    // passive. These are masks over the B/W base, not a replacement DTM2 image.
    panel_.setGrayEncoding(passiveBlackLut_ ? GrayEncoding::Uc8253Overlay : GrayEncoding::Mono);
    const RefreshKind kind = partial_ ? RefreshKind::Partial : RefreshKind::Full;
    panel_.startRefresh(kind, partial_ ? kPartialRefreshMs : kFullRefreshMs);
  }

  Panel& panel_;
  const char* name_;
  bool answersVersionProbe_;
  uint8_t cmd_ = 0;
  std::vector<uint8_t> args_;
  bool streaming_ = false;
  size_t streamed_ = 0;
  int plane_ = 0;
  bool partial_ = false;
  int partialX0_ = 0, partialY0_ = 0;
  int readIndex_ = 0;
  uint8_t lutVersion_ = 0x02;
  bool passiveBlackLut_ = false;
};

}  // namespace

// ── Panel ────────────────────────────────────────────────────────────────────
Panel::Panel(Machine& machine) : machine_(machine) {
  configure(800, 480, FSIM_PANEL_SSD1677, false, false, true);
}

Panel::~Panel() = default;

void Panel::configure(int width, int height, uint8_t controller, bool mirrorX, bool mirrorY,
                      bool gatesReversed) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (width <= 0 || height <= 0) return;
  const bool geometryChanged = width != width_ || height != height_;
  width_ = width;
  height_ = height;
  mirrorX_ = mirrorX;
  mirrorY_ = mirrorY;
  gatesReversed_ = gatesReversed;
  controllerId_ = controller;
  grayEncoding_ = GrayEncoding::Mono;

  switch (controller) {
    case FSIM_PANEL_SSD1677:
      controller_ = std::make_unique<Ssd1677Controller>(*this);
      break;
    case FSIM_PANEL_UC8253_X3:
      controller_ = std::make_unique<Uc81xxController>(*this, "UC8253", false);
      break;
    case FSIM_PANEL_UC8179:
      controller_ = std::make_unique<Uc81xxController>(*this, "UC8179", true);
      break;
    case FSIM_PANEL_UC8279:
      controller_ = std::make_unique<Uc81xxController>(*this, "UC8279", true);
      break;
    default:
      controller_.reset();
      break;
  }

  const size_t pixels = static_cast<size_t>(width_) * height_;
  if (geometryChanged || ram_[0].size() != pixels) {
    ram_[0].assign(pixels, 1);  // 1 = white in both families' mono encoding
    ram_[1].assign(pixels, 1);
    ghost_.assign(pixels, 255);
    frame_.width = width_;
    frame_.height = height_;
    frame_.pixels.assign(pixels, 255);
  }
  winX0_ = 0;
  winY0_ = 0;
  winX1_ = width_ - 1;
  winY1_ = height_ - 1;
  curX_ = 0;
  curY_ = 0;
}

const char* Panel::controllerName() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return controller_ ? controller_->name() : "external";
}

void Panel::spiByte(uint8_t value, bool isData) {
  bool startedRefresh = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!controller_) return;
    const bool wasRefreshing = refreshing_;
    if (isData) {
      controller_->data(&value, 1);
    } else {
      controller_->command(value);
    }
    startedRefresh = refreshing_ && !wasRefreshing;
  }
  // The waveform's BUSY assertion is a real pin edge, raised after the panel
  // lock is dropped so the firmware's ISR cannot deadlock against it.
  if (startedRefresh) machine_.setPanelBusy(true);
}

uint8_t Panel::spiReadByte() {
  std::lock_guard<std::mutex> lock(mutex_);
  return controller_ ? controller_->readByte() : 0xFF;
}

void Panel::hardwareReset() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (controller_) controller_->reset();
    refreshing_ = false;
  }
  machine_.setPanelBusy(false);
}

void Panel::setPowered(bool powered) { powered_ = powered; }

void Panel::setRamWindow(int x0, int y0, int x1, int y1) {
  winX0_ = std::max(0, std::min(x0, width_ - 1));
  winY0_ = std::max(0, std::min(y0, height_ - 1));
  winX1_ = std::max(0, std::min(x1, width_ - 1));
  winY1_ = std::max(0, std::min(y1, height_ - 1));
  curX_ = winX0_;
  curY_ = winY0_;
}

void Panel::setRamCursor(int x, int y) {
  curX_ = std::max(0, std::min(x, width_ - 1));
  curY_ = std::max(0, std::min(y, height_ - 1));
}

void Panel::setDataEntryMode(uint8_t mode) { dataEntry_ = mode; }

void Panel::writeRam(const uint8_t* bytes, size_t len, int plane) {
  if (plane < 0 || plane > 1 || ram_[plane].empty()) return;

  // Bit 0 of the data-entry mode is the X direction, bit 1 the Y direction;
  // 0 means decrement. The SSD1677 drivers here run X-increment/Y-decrement,
  // so getting this wrong flips the image — which is exactly the class of
  // driver bug the model is meant to expose rather than hide.
  const bool xIncrement = (dataEntry_ & 0x01) != 0;
  const bool yIncrement = (dataEntry_ & 0x02) != 0;
  const int windowWidth = winX1_ - winX0_ + 1;

  for (size_t i = 0; i < len; ++i) {
    for (int bit = 7; bit >= 0; --bit) {
      const uint8_t value = (bytes[i] >> bit) & 0x01;
      if (curX_ >= 0 && curX_ < width_ && curY_ >= 0 && curY_ < height_) {
        ram_[plane][static_cast<size_t>(curY_) * width_ + curX_] = value;
      }
      // Advance the cursor along X, wrapping to the next row at the window edge.
      if (xIncrement) {
        ++curX_;
        if (curX_ > winX1_) {
          curX_ = winX0_;
          curY_ += yIncrement ? 1 : -1;
        }
      } else {
        --curX_;
        if (curX_ < winX0_) {
          curX_ = winX1_;
          curY_ += yIncrement ? 1 : -1;
        }
      }
      if (windowWidth <= 0) break;
    }
  }
}

void Panel::clearRam(uint8_t value, int plane) {
  if (plane < 0 || plane > 1) return;
  std::fill(ram_[plane].begin(), ram_[plane].end(), value ? 1 : 0);
}

void Panel::startRefresh(RefreshKind kind, uint32_t durationMs) {
  refreshing_ = true;
  refreshKind_ = kind;
  refreshDurationMs_ = durationMs;
  refreshEndUs_ = machine_.clock().nowUs() + static_cast<uint64_t>(durationMs) * 1000ULL;
}

void Panel::recordEvent(uint8_t cmd, const uint8_t* data, size_t len, const char* name) {
  BusEvent event;
  event.timeUs = machine_.clock().nowUs();
  event.command = cmd;
  event.dataLength = len;
  event.name = name;
  if (data && len > 0) {
    const size_t keep = std::min<size_t>(len, 16);
    event.data.assign(data, data + keep);
  }
  trace_.push_back(std::move(event));
  while (trace_.size() > kMaxTrace) trace_.pop_front();
}

bool Panel::busy() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return refreshing_;
}

void Panel::tick() {
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!refreshing_) return;
    if (machine_.clock().nowUs() < refreshEndUs_) return;
    refreshing_ = false;
    publishFrame(refreshKind_, refreshDurationMs_);
  }
  // Completion edge first, then wake anyone waiting on the frame: a caller that
  // was blocked on the refresh should see BUSY already released.
  machine_.setPanelBusy(false);
  frameCv_.notify_all();
}

void Panel::composite(std::vector<uint8_t>* out) const {
  const size_t pixels = static_cast<size_t>(width_) * height_;
  out->assign(pixels, 255);

  // Reported by the bundle (see epd_gates_reversed): on these panels RAM row 0
  // is the bottom of the glass, and the driver compensates. Applying the
  // physical reversal here is what makes the compensated stream come out
  // upright.
  const bool gatesReversed = gatesReversed_;

  for (int y = 0; y < height_; ++y) {
    const int srcY = gatesReversed ? (height_ - 1 - y) : y;
    for (int x = 0; x < width_; ++x) {
      int srcX = x;
      int dstY = y;
      int dstX = x;
      if (mirrorX_) dstX = width_ - 1 - x;
      if (mirrorY_) dstY = height_ - 1 - y;
      const uint8_t bw = ram_[0][static_cast<size_t>(srcY) * width_ + srcX];

      uint8_t value = bw ? 255 : 0;
      if (grayEncoding_ != GrayEncoding::Mono) {
        // Two-plane grey: the second plane selects the mid tones. The levels
        // are the drivers' nominal targets, not measured optical response.
        const uint8_t second = ram_[1][static_cast<size_t>(srcY) * width_ + srcX];
        const int code = (bw ? 2 : 0) | (second ? 1 : 0);
        if (grayEncoding_ == GrayEncoding::Uc8253Overlay) {
          const size_t destination = static_cast<size_t>(dstY) * width_ + dstX;
          value = code < 2 ? frame_.pixels[destination] : (code == 2 ? 170 : 85);
        } else if (grayEncoding_ == GrayEncoding::Ssd1677Overlay) {
          static const uint8_t kLevels[4] = {0, 170, 128, 85};
          const size_t destination = static_cast<size_t>(dstY) * width_ + dstX;
          value = code == 0 ? frame_.pixels[destination] : kLevels[code];
        } else if (grayEncoding_ == GrayEncoding::Ssd1677Absolute) {
          static const uint8_t kLevels[4] = {255, 85, 170, 0};
          value = kLevels[code];
        } else {
          static const uint8_t kLevels[4] = {0, 85, 170, 255};
          value = kLevels[code];
        }
      }
      (*out)[static_cast<size_t>(dstY) * width_ + dstX] = value;
    }
  }
}

void Panel::publishFrame(RefreshKind kind, uint32_t durationMs) {
  std::vector<uint8_t> fresh;
  composite(&fresh);

  // Ghosting, approximated. A full refresh scrubs the panel clean; a partial or
  // fast one leaves a trace of what was there, which is why reader firmware
  // schedules periodic fulls. This is a visual cue, not a waveform simulation.
  if (kind == RefreshKind::Full) {
    ghost_ = fresh;
  } else {
    for (size_t i = 0; i < fresh.size(); ++i) {
      const int previous = ghost_[i];
      const int current = fresh[i];
      if (current == 255 && previous == 0) {
        fresh[i] = 236;  // white that was recently black keeps a faint shadow
      }
      ghost_[i] = static_cast<uint8_t>(current);
    }
  }

  frame_.seq = ++frameSeq_;
  frame_.timeUs = machine_.clock().nowUs();
  frame_.width = width_;
  frame_.height = height_;
  frame_.kind = kind;
  frame_.durationMs = durationMs;
  frame_.pixels = std::move(fresh);
}

void Panel::pushFrame(const uint8_t* plane, size_t len, int width, int height, int mode) {
  std::unique_lock<std::mutex> lock(mutex_);
  if (!plane || width <= 0 || height <= 0) return;
  if (width != width_ || height != height_) {
    lock.unlock();
    configure(width, height, FSIM_PANEL_EXTERNAL, mirrorX_, mirrorY_, gatesReversed_);
    lock.lock();
  }
  // 1bpp MSB-first, row-major, as the external-bus drivers hand it over.
  const size_t rowBytes = (static_cast<size_t>(width) + 7) / 8;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const size_t byteIndex = static_cast<size_t>(y) * rowBytes + static_cast<size_t>(x) / 8;
      if (byteIndex >= len) break;
      const uint8_t bit = (plane[byteIndex] >> (7 - (x % 8))) & 0x01;
      ram_[0][static_cast<size_t>(y) * width_ + x] = bit;
    }
  }
  publishFrame(mode == 0 ? RefreshKind::Full : RefreshKind::Fast, mode == 0 ? kFullRefreshMs : kFastRefreshMs);
  lock.unlock();
  frameCv_.notify_all();
}

Frame Panel::currentFrame() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return frame_;
}

uint64_t Panel::frameSeq() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return frameSeq_;
}

bool Panel::waitForFrame(uint64_t afterSeq, uint64_t timeoutMs, Frame* out) const {
  std::unique_lock<std::mutex> lock(mutex_);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
  // Wall-clock timeout on purpose: a caller waiting for a refresh needs to be
  // released even if the machine is paused and simulated time is frozen.
  while (frameSeq_ <= afterSeq) {
    if (frameCv_.wait_until(lock, deadline) == std::cv_status::timeout && frameSeq_ <= afterSeq) return false;
  }
  if (out) *out = frame_;
  return true;
}

std::vector<BusEvent> Panel::busTrace(size_t limit) const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<BusEvent> out;
  const size_t start = trace_.size() > limit ? trace_.size() - limit : 0;
  for (size_t i = start; i < trace_.size(); ++i) out.push_back(trace_[i]);
  return out;
}

void Panel::clearBusTrace() {
  std::lock_guard<std::mutex> lock(mutex_);
  trace_.clear();
}

}  // namespace freeink::sim
