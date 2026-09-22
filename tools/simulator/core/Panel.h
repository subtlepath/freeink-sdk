#pragma once

// FreeInk simulator — virtual e-paper panel.
//
// This is the seam that decides how much the simulator is worth. Rather than
// reading the facade's framebuffer, the model decodes the actual SPI traffic
// the panel driver emits: commands are separated from data by the D/C pin, RAM
// writes land in a real image buffer at the window the driver programmed, and a
// display-update command runs a waveform that takes time and holds BUSY.
//
// So the driver under test is exercised for real. A driver that programs the
// wrong RAM window, forgets to set the data-entry mode, streams the planes in
// the wrong order, or fires a refresh without waiting for BUSY produces a wrong
// or missing image here, exactly as it would on glass.
//
// What is NOT modelled: waveform physics. LUT contents are recorded, not
// simulated, so grey levels are the driver's intent rather than measured
// optical response, and ghosting is approximated from refresh mode and history
// rather than derived from voltages. Panel tuning still needs hardware.

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace freeink::sim {

class Machine;

enum class RefreshKind { Full, Partial, Fast, Unknown };

// One finished frame, as the glass would show it.
struct Frame {
  uint64_t seq = 0;
  uint64_t timeUs = 0;
  int width = 0;
  int height = 0;
  RefreshKind kind = RefreshKind::Unknown;
  uint32_t durationMs = 0;
  // 8-bit greyscale, row-major, width*height. 0 = black, 255 = white.
  std::vector<uint8_t> pixels;
};

// A decoded controller command, kept for `freeink-sim bus trace` — the view
// that turns "the screen is blank" into "the driver never sent 0x22/0x20".
struct BusEvent {
  uint64_t timeUs;
  uint8_t command;
  std::vector<uint8_t> data;  // truncated for bulk RAM writes
  size_t dataLength;          // the real length, before truncation
  const char* name;
};

// Per-controller decoding. One subclass per silicon family; each owns its
// command set and its RAM addressing rules.
class PanelController {
 public:
  virtual ~PanelController() = default;
  virtual const char* name() const = 0;
  // A command byte (D/C low) arrived.
  virtual void command(uint8_t cmd) = 0;
  // Data bytes (D/C high) arrived for the standing command.
  virtual void data(const uint8_t* bytes, size_t len) = 0;
  // A byte the driver clocked out while reading; controllers that answer reads
  // (the UC8179/UC8279 VER/FLG probe) override this.
  virtual uint8_t readByte() { return 0xFF; }
  virtual void reset() = 0;
};

class Panel {
 public:
  explicit Panel(Machine& machine);
  ~Panel();

  // Geometry and silicon come from the board description the bundle reported.
  void configure(int width, int height, uint8_t controller, bool mirrorX, bool mirrorY, bool gatesReversed);
  int width() const { return width_; }
  int height() const { return height_; }
  const char* controllerName() const;

  // ── Bus input ──────────────────────────────────────────────────────────────
  // Called from the SPI model for every byte, with the D/C level at that
  // moment. This is the only way pixels reach the panel on native-bus boards.
  void spiByte(uint8_t value, bool isData);
  uint8_t spiReadByte();
  void hardwareReset();
  void setPowered(bool powered);
  bool powered() const { return powered_; }

  // External-bus drivers (M5GFX/LovyanGFX/IT8951) never reach the SPI model, so
  // they hand over a finished plane instead.
  void pushFrame(const uint8_t* plane, size_t len, int width, int height, int mode);

  // ── BUSY ───────────────────────────────────────────────────────────────────
  // True while a waveform is running. The SPI model exposes this on the BUSY
  // pin with the controller's polarity, so EpdBus's waits behave as on device.
  bool busy() const;
  // Advanced by the daemon's clock thread; completes a running waveform and
  // publishes the frame when its duration has elapsed.
  void tick();

  // ── Output ─────────────────────────────────────────────────────────────────
  // The last published frame, or an all-white one before the first refresh.
  Frame currentFrame() const;
  uint64_t frameSeq() const;
  // Waits until a frame newer than `afterSeq` is published, or the deadline
  // passes. This is what `freeink-sim capture --wait-refresh` is built on, and
  // it is what makes screenshots race-free.
  bool waitForFrame(uint64_t afterSeq, uint64_t timeoutMs, Frame* out) const;

  std::vector<BusEvent> busTrace(size_t limit) const;
  void clearBusTrace();

  // ── Model surface used by the controllers ──────────────────────────────────
  // The controllers are friends in spirit: they drive these to express what the
  // command stream told them to do.
  void setRamWindow(int x0, int y0, int x1, int y1);
  void setRamCursor(int x, int y);
  void setDataEntryMode(uint8_t mode);
  // Writes 1bpp data into the active window, advancing the cursor. `plane`
  // selects black/white (0) or the red/second plane (1).
  void writeRam(const uint8_t* bytes, size_t len, int plane);
  void clearRam(uint8_t value, int plane);
  // Starts a waveform of the given kind; the frame is published when it ends.
  void startRefresh(RefreshKind kind, uint32_t durationMs);
  void recordEvent(uint8_t cmd, const uint8_t* data, size_t len, const char* name);
  // Grey depth the controller is currently uploading (1 = mono, 2 = two-plane
  // 4-level grey). Set by drivers that stream dual planes.
  void setGrayPlanes(int planes) { grayPlanes_ = planes; }
  int grayPlanes() const { return grayPlanes_; }

 private:
  void publishFrame(RefreshKind kind, uint32_t durationMs);
  void composite(std::vector<uint8_t>* out) const;

  Machine& machine_;
  mutable std::mutex mutex_;
  mutable std::condition_variable_any frameCv_;

  int width_ = 800;
  int height_ = 480;
  bool mirrorX_ = false;
  bool mirrorY_ = false;
  bool gatesReversed_ = false;
  uint8_t controllerId_ = 0;
  std::unique_ptr<PanelController> controller_;

  bool powered_ = false;

  // Controller RAM: one byte per pixel per plane, so window/cursor bugs are
  // visible as displaced pixels rather than silently packed away.
  std::vector<uint8_t> ram_[2];
  int winX0_ = 0, winY0_ = 0, winX1_ = 0, winY1_ = 0;
  int curX_ = 0, curY_ = 0;
  uint8_t dataEntry_ = 0x03;

  bool refreshing_ = false;
  uint64_t refreshEndUs_ = 0;
  RefreshKind refreshKind_ = RefreshKind::Unknown;
  uint32_t refreshDurationMs_ = 0;

  Frame frame_;
  uint64_t frameSeq_ = 0;
  // Approximated ghosting: the previous image bleeds through a partial refresh
  // and is scrubbed by a full one. Enough to make "this screen needs a full
  // refresh" visible; not a waveform simulation.
  std::vector<uint8_t> ghost_;
  int grayPlanes_ = 1;

  std::deque<BusEvent> trace_;
  static constexpr size_t kMaxTrace = 4096;
};

}  // namespace freeink::sim
