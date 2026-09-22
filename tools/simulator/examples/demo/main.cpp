// FreeInk simulator — demo firmware.
//
// A deliberately small sketch whose only job is to prove the simulator end to
// end: it paints through the real EInkDisplay facade (so the real panel driver
// and EpdBus run, and the virtual controller decodes their SPI traffic), reads
// buttons through the real InputManager (so the real debounce and ADC-ladder
// decode run), and repaints on every press.
//
// This is not a reference for how to structure firmware — consumer projects own
// their entry point. It is a fixture.

#include <Arduino.h>
#include <BoardConfig.h>
#include <EInkDisplay.h>
#include <InputManager.h>

namespace {

EInkDisplay display(BoardConfig::DEFAULT_DEVICE.display.sclk, BoardConfig::DEFAULT_DEVICE.display.mosi,
                    BoardConfig::DEFAULT_DEVICE.display.cs, BoardConfig::DEFAULT_DEVICE.display.dc,
                    BoardConfig::DEFAULT_DEVICE.display.rst, BoardConfig::DEFAULT_DEVICE.display.busy);
InputManager input;

int pressCount = 0;
const char* lastButton = "none";

// The framebuffer is 1bpp, MSB first, 1 = white. Draw straight into it: the
// point here is to exercise the driver's upload path, not to demonstrate the UI
// layer.
void setPixel(int x, int y, bool black) {
  if (x < 0 || y < 0 || x >= display.getDisplayWidth() || y >= display.getDisplayHeight()) return;
  uint8_t* fb = display.getFrameBuffer();
  if (!fb) return;
  const int stride = display.getDisplayWidthBytes();
  uint8_t& cell = fb[y * stride + (x >> 3)];
  const uint8_t mask = static_cast<uint8_t>(0x80 >> (x & 7));
  if (black) {
    cell &= static_cast<uint8_t>(~mask);
  } else {
    cell |= mask;
  }
}

void fillRect(int x, int y, int w, int h, bool black) {
  for (int row = y; row < y + h; ++row) {
    for (int col = x; col < x + w; ++col) setPixel(col, row, black);
  }
}

// A 5x7 block font, enough to render the status line without pulling the font
// stack into a fixture.
const uint8_t kGlyphs[][5] = {
    {0x7E, 0x11, 0x11, 0x11, 0x7E},  // A
    {0x7F, 0x49, 0x49, 0x49, 0x36},  // B
    {0x3E, 0x41, 0x41, 0x41, 0x22},  // C
    {0x7F, 0x41, 0x41, 0x22, 0x1C},  // D
    {0x7F, 0x49, 0x49, 0x49, 0x41},  // E
    {0x7F, 0x09, 0x09, 0x09, 0x01},  // F
};

void drawGlyph(int x, int y, const uint8_t glyph[5], int scale) {
  for (int col = 0; col < 5; ++col) {
    for (int row = 0; row < 7; ++row) {
      if (glyph[col] & (1 << row)) fillRect(x + col * scale, y + row * scale, scale, scale, true);
    }
  }
}

void paint() {
  // White page, a header bar, and a row of blocks counting presses — enough
  // structure that a capture is obviously right side up and not mirrored.
  memset(display.getFrameBuffer(), 0xFF, display.getBufferSize());

  const int width = display.getDisplayWidth();
  const int height = display.getDisplayHeight();

  fillRect(0, 0, width, 48, true);          // header bar, top of the screen
  fillRect(8, 8, 32, 32, false);            // a white square inside it, left
  fillRect(0, height - 8, width, 8, true);  // footer rule, bottom

  // "A" below the header, where it reads as ink on white. Asymmetric top to
  // bottom, so a vertically flipped frame is obvious at a glance.
  drawGlyph(64, 60, kGlyphs[0], 4);

  // One filled block per press, left to right.
  for (int i = 0; i < pressCount && i < 20; ++i) {
    fillRect(16 + i * 36, 96, 28, 28, true);
  }

  // A diagonal, so a transposed or mirrored upload is unmistakable.
  for (int i = 0; i < height - 64; ++i) {
    fillRect(width - 64 - i / 2, 56 + i, 2, 2, true);
  }

  display.displayBuffer(EInkDisplay::FULL_REFRESH);
}

}  // namespace

// Boards with a frontlight get it lit, at half brightness on each channel the
// profile declares. Nothing about this is board-specific in the fixture: the
// pins, the frequency and the second warm channel all come from the profile,
// which is the same place the simulator builds its model of the board from.
void lightFrontlight() {
  const BoardConfig::FrontlightConfig& frontlight = BoardConfig::ACTIVE.frontlight;
  if (frontlight.gpio == BoardConfig::PIN_UNASSIGNED) return;

  const uint32_t half = (1u << frontlight.pwmResolutionBits) / 2;
  ledcAttach(frontlight.gpio, frontlight.pwmFrequency, frontlight.pwmResolutionBits);
  ledcWrite(frontlight.gpio, frontlight.activeHigh ? half : (1u << frontlight.pwmResolutionBits) - half);
  Serial.printf("[demo] frontlight on GPIO%d\n", frontlight.gpio);

  if (frontlight.gpioWarm == BoardConfig::PIN_UNASSIGNED) return;
  ledcAttach(frontlight.gpioWarm, frontlight.pwmFrequency, frontlight.pwmResolutionBits);
  ledcWrite(frontlight.gpioWarm, frontlight.activeHigh ? half : (1u << frontlight.pwmResolutionBits) - half);
  Serial.printf("[demo] warm frontlight on GPIO%d\n", frontlight.gpioWarm);
}

void setup() {
  Serial.begin(115200);
  Serial.printf("[demo] booting on %s (%dx%d)\n", BoardConfig::ACTIVE.name, BoardConfig::ACTIVE.displayWidth,
                BoardConfig::ACTIVE.displayHeight);

  BoardConfig::holdPowerRails();
  display.begin();
  input.begin();

  Serial.printf("[demo] display ready: %dx%d, %d byte framebuffer\n", display.getDisplayWidth(),
                display.getDisplayHeight(), display.getBufferSize());
  lightFrontlight();
  paint();
  Serial.println("[demo] first paint complete");
}

void loop() {
  input.update();

  for (uint8_t button = 0; button < 7; ++button) {
    if (!input.wasPressed(button)) continue;
    lastButton = InputManager::getButtonName(button);
    ++pressCount;
    Serial.printf("[demo] press #%d: %s\n", pressCount, lastButton);
    paint();
    Serial.printf("[demo] repainted after %s\n", lastButton);
  }

  delay(20);
}
