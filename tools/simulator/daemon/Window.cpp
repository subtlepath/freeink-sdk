// FreeInk simulator — SDL2 window.

#include "Window.h"

#include "../core/Machine.h"
#include "../core/Panel.h"
#include "Runtime.h"

#if FSIM_HAVE_SDL
// sdl2-config puts the SDL2 directory itself on the include path, so the
// header is <SDL.h>, not <SDL2/SDL.h> — spelling it the other way can pick up a
// different architecture's SDL from a default include directory.
//
// SDL_MAIN_HANDLED: this daemon owns main(); without it SDL would rename it and
// expect its own entry point.
#define SDL_MAIN_HANDLED
#include <SDL.h>
#endif

#include <algorithm>
#include <vector>

namespace freeink::sim {

#if !FSIM_HAVE_SDL

// Headless build: every entry point is a no-op and open() explains why.
Window::Window(Machine& machine, Runtime& runtime) : machine_(machine), runtime_(runtime) {}
Window::~Window() = default;
bool Window::open(std::string* error) {
  if (error) *error = "this daemon was built without SDL2; run with --headless or rebuild with SDL2 installed";
  return false;
}
void Window::close() {}
bool Window::pump() { return false; }
void Window::requestResize() {}

#else

namespace {

// Keyboard mapping. Arrow keys and Enter/Backspace are the obvious nav pair;
// the letter keys match the button names so they are guessable without a
// reference card.
int buttonForKey(int key) {
  switch (key) {
    case SDLK_LEFT: return 2;       // left
    case SDLK_RIGHT: return 3;      // right
    case SDLK_UP: return 4;         // up
    case SDLK_DOWN: return 5;       // down
    case SDLK_RETURN: return 1;     // confirm
    case SDLK_KP_ENTER: return 1;
    case SDLK_SPACE: return 1;
    case SDLK_BACKSPACE: return 0;  // back
    case SDLK_ESCAPE: return 0;
    case SDLK_p: return 6;          // power
    default: return -1;
  }
}

}  // namespace

Window::Window(Machine& machine, Runtime& runtime) : machine_(machine), runtime_(runtime) {}

Window::~Window() { close(); }

bool Window::open(std::string* error) {
  SDL_SetMainReady();
  if (SDL_Init(SDL_INIT_VIDEO) != 0) {
    if (error) *error = std::string("SDL_Init: ") + SDL_GetError();
    return false;
  }

  auto& panel = machine_.panel();
  textureWidth_ = panel.width();
  textureHeight_ = panel.height();

  // Fit the window to the display: these panels are 800x480 or 792x528, which
  // fits most screens 1:1, but a scaled-down window beats one that runs off.
  SDL_DisplayMode mode{};
  int windowWidth = textureWidth_;
  int windowHeight = textureHeight_;
  if (SDL_GetCurrentDisplayMode(0, &mode) == 0) {
    while (windowWidth > mode.w - 80 || windowHeight > mode.h - 120) {
      windowWidth /= 2;
      windowHeight /= 2;
    }
  }

  auto* window = SDL_CreateWindow("FreeInk Simulator", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, windowWidth,
                                  windowHeight, SDL_WINDOW_SHOWN | SDL_WINDOW_ALLOW_HIGHDPI);
  if (!window) {
    if (error) *error = std::string("SDL_CreateWindow: ") + SDL_GetError();
    SDL_Quit();
    return false;
  }
  window_ = window;

  auto* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
  if (!renderer) renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_SOFTWARE);
  if (!renderer) {
    if (error) *error = std::string("SDL_CreateRenderer: ") + SDL_GetError();
    SDL_DestroyWindow(window);
    window_ = nullptr;
    SDL_Quit();
    return false;
  }
  renderer_ = renderer;
  SDL_RenderSetLogicalSize(renderer, textureWidth_, textureHeight_);

  texture_ = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGB24, SDL_TEXTUREACCESS_STREAMING, textureWidth_,
                               textureHeight_);
  if (!texture_) {
    if (error) *error = std::string("SDL_CreateTexture: ") + SDL_GetError();
    close();
    return false;
  }

  open_ = true;
  lastFrameSeq_ = 0;
  return true;
}

void Window::close() {
  if (texture_) SDL_DestroyTexture(static_cast<SDL_Texture*>(texture_));
  if (renderer_) SDL_DestroyRenderer(static_cast<SDL_Renderer*>(renderer_));
  if (window_) SDL_DestroyWindow(static_cast<SDL_Window*>(window_));
  texture_ = nullptr;
  renderer_ = nullptr;
  window_ = nullptr;
  if (open_) SDL_Quit();
  open_ = false;
}

void Window::requestResize() { resizeRequested_ = true; }

void Window::handleKey(int key, bool down) {
  const int button = buttonForKey(key);
  if (button >= 0) {
    machine_.setButton(button, down);
    return;
  }
  if (!down) return;
  // Machine controls, on keys no board button claims.
  switch (key) {
    case SDLK_F5: {
      std::string error;
      runtime_.reset(&error);
      break;
    }
    case SDLK_F6:
      if (machine_.clock().paused()) {
        machine_.clock().resume();
      } else {
        machine_.clock().pause();
      }
      break;
    default:
      break;
  }
}

bool Window::pump() {
  if (!open_) return false;

  SDL_Event event;
  while (SDL_PollEvent(&event)) {
    switch (event.type) {
      case SDL_QUIT:
        return false;
      case SDL_KEYDOWN:
        if (event.key.repeat) break;  // a held key is one press, not a stream
        handleKey(event.key.keysym.sym, true);
        break;
      case SDL_KEYUP:
        handleKey(event.key.keysym.sym, false);
        break;
      case SDL_MOUSEBUTTONDOWN:
      case SDL_MOUSEBUTTONUP:
      case SDL_MOUSEMOTION: {
        // Mouse as a finger, but only on boards that have a digitizer —
        // synthesizing touch on a button-only board would let a test pass
        // against input the hardware cannot produce.
        if (machine_.board().touch_controller == FSIM_TOUCH_NONE) break;
        int windowW = 1;
        int windowH = 1;
        SDL_GetWindowSize(static_cast<SDL_Window*>(window_), &windowW, &windowH);
        const int x = event.type == SDL_MOUSEMOTION ? event.motion.x : event.button.x;
        const int y = event.type == SDL_MOUSEMOTION ? event.motion.y : event.button.y;
        const int panelX = static_cast<int>(static_cast<double>(x) * textureWidth_ / std::max(1, windowW));
        const int panelY = static_cast<int>(static_cast<double>(y) * textureHeight_ / std::max(1, windowH));
        if (event.type == SDL_MOUSEBUTTONDOWN) {
          machine_.setTouch(panelX, panelY, true);
        } else if (event.type == SDL_MOUSEBUTTONUP) {
          machine_.clearTouch();
        } else if (machine_.touch().down) {
          machine_.setTouch(panelX, panelY, true);
        }
        break;
      }
      default:
        break;
    }
  }

  if (resizeRequested_.exchange(false)) {
    auto& panel = machine_.panel();
    if (panel.width() != textureWidth_ || panel.height() != textureHeight_) {
      textureWidth_ = panel.width();
      textureHeight_ = panel.height();
      if (texture_) SDL_DestroyTexture(static_cast<SDL_Texture*>(texture_));
      texture_ = SDL_CreateTexture(static_cast<SDL_Renderer*>(renderer_), SDL_PIXELFORMAT_RGB24,
                                   SDL_TEXTUREACCESS_STREAMING, textureWidth_, textureHeight_);
      SDL_RenderSetLogicalSize(static_cast<SDL_Renderer*>(renderer_), textureWidth_, textureHeight_);
      SDL_SetWindowSize(static_cast<SDL_Window*>(window_), textureWidth_, textureHeight_);
      lastFrameSeq_ = 0;
    }
  }

  drawFrame();
  return true;
}

void Window::drawFrame() {
  auto& panel = machine_.panel();
  const uint64_t seq = panel.frameSeq();
  auto* renderer = static_cast<SDL_Renderer*>(renderer_);
  auto* texture = static_cast<SDL_Texture*>(texture_);
  if (!renderer || !texture) return;

  if (seq != lastFrameSeq_) {
    const Frame frame = panel.currentFrame();
    if (frame.width == textureWidth_ && frame.height == textureHeight_ && !frame.pixels.empty()) {
      void* raw = nullptr;
      int pitch = 0;
      if (SDL_LockTexture(texture, nullptr, &raw, &pitch) == 0) {
        auto* out = static_cast<uint8_t*>(raw);
        for (int y = 0; y < frame.height; ++y) {
          uint8_t* row = out + static_cast<size_t>(y) * pitch;
          const uint8_t* src = frame.pixels.data() + static_cast<size_t>(y) * frame.width;
          for (int x = 0; x < frame.width; ++x) {
            // E-paper white is warm, not pure: a slightly tinted white reads
            // far closer to the real panel than 0xFFFFFF.
            const uint8_t v = src[x];
            row[x * 3 + 0] = v;
            row[x * 3 + 1] = v;
            row[x * 3 + 2] = static_cast<uint8_t>(v > 8 ? v - 6 : v);
          }
        }
        SDL_UnlockTexture(texture);
      }
      lastFrameSeq_ = seq;
    }
  }

  SDL_SetRenderDrawColor(renderer, 24, 24, 26, 255);
  SDL_RenderClear(renderer);
  SDL_RenderCopy(renderer, texture, nullptr, nullptr);
  SDL_RenderPresent(renderer);
}

#endif  // FSIM_HAVE_SDL

}  // namespace freeink::sim
