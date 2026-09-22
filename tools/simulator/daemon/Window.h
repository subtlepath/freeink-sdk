#pragma once

// FreeInk simulator — SDL2 window.
//
// Optional: the daemon runs headless without it, which is the mode CI and
// agent loops use. When present it shows the panel at 1:1 (or scaled to fit)
// and maps the keyboard onto the board's buttons and the mouse onto touch, so
// the same machine can be driven by hand and by the socket at once.
//
// macOS requires the event loop on the main thread, which is why the control
// socket runs on its own thread and this owns main().

#include <atomic>
#include <string>

namespace freeink::sim {

class Machine;
class Runtime;

class Window {
 public:
  Window(Machine& machine, Runtime& runtime);
  ~Window();

  // Returns false with `error` set when SDL is unavailable or the display
  // cannot be opened; the daemon then continues headless rather than exiting,
  // because a missing display is not a reason to lose the simulation.
  bool open(std::string* error);
  void close();
  bool isOpen() const { return open_; }

  // Pumps events and repaints. Call from the main thread; returns false when
  // the user closed the window.
  bool pump();
  // Called when the firmware selects a board with different geometry.
  void requestResize();

 private:
  void drawFrame();
  void handleKey(int sdlKeycode, bool down);

  Machine& machine_;
  Runtime& runtime_;
  void* window_ = nullptr;
  void* renderer_ = nullptr;
  void* texture_ = nullptr;
  int textureWidth_ = 0;
  int textureHeight_ = 0;
  uint64_t lastFrameSeq_ = 0;
  bool open_ = false;
  std::atomic<bool> resizeRequested_{false};
};

}  // namespace freeink::sim
