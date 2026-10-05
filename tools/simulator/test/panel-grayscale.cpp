// Exercise the controller's actual SPI command path with the shipped LUTs.
#include "../core/Machine.h"
#include "../core/Panel.h"
#include "../../../libs/display/FreeInkDisplay/src/lut/Ssd1677Luts.h"
#include "../../../libs/display/FreeInkDisplay/src/lut/Uc8253X3Luts.h"
#include <cassert>
#include <iostream>

using namespace freeink::sim;

int main() {
  auto& machine = Machine::instance();
  auto& panel = machine.panel();
  panel.configure(8, 1, FSIM_PANEL_SSD1677, false, false, false);
  const auto command = [&](uint8_t cmd, uint8_t value) {
    panel.spiByte(cmd, false); panel.spiByte(value, true);
  };
  const auto plane = [&](uint8_t cmd, uint8_t bits) {
    panel.setRamCursor(0, 0); command(cmd, bits);
  };
  const auto lut = [&](const unsigned char* bytes) {
    panel.spiByte(0x32, false);
    for (int i = 0; i < 105; ++i) panel.spiByte(bytes[i], true);
  };
  const auto refresh = [&](uint8_t sequence) {
    command(0x22, sequence); panel.spiByte(0x20, false);
    machine.clock().advanceExternal(2000000); panel.tick();
  };
  plane(0x24, 0x55); refresh(0xF7);
  const auto base = panel.currentFrame().pixels;
  assert((base == std::vector<uint8_t>{0, 255, 0, 255, 0, 255, 0, 255}));

  // AA's 00 code must preserve both white paper and solid black text.
  plane(0x24, 0x18); plane(0x26, 0x28);
  lut(freeink::lut_grayscale); refresh(0xCC);
  assert((panel.currentFrame().pixels == std::vector<uint8_t>{0, 255, 170, 128, 85, 255, 0, 255}));
  const auto gray = panel.currentFrame();
  plane(0x26, 0x55); // Firmware cleanup restores RED's B/W differential baseline.
  refresh(0x03);    // Power-off must neither repaint nor re-composite that RAM.
  assert(panel.frameSeq() == gray.seq);
  assert(panel.currentFrame().pixels == gray.pixels);

  // OTP reload restores mono interpretation after an external grayscale LUT.
  plane(0x24, 0x55); refresh(0xF7);
  assert(panel.currentFrame().pixels == base);

  // Factory LUT targets have the inverse polarity of the public gray planes.
  plane(0x24, 0x33); plane(0x26, 0x55);
  lut(freeink::lut_factory_quality); refresh(0xCC);
  const auto absolute = panel.currentFrame().pixels;
  for (int i = 0; i < 8; ++i) {
    const uint8_t expected[] = {255, 85, 170, 0};
    // A fast waveform may leave the model's faint shadow on white targets.
    assert(expected[i % 4] == 255 ? absolute[i] >= 220 : absolute[i] == expected[i % 4]);
  }

  // HALF's RED bypass must win even when an external LUT remains loaded.
  command(0x21, 0x40); plane(0x24, 0x55); refresh(0xCC);
  const auto bypass = panel.currentFrame().pixels;
  for (int i = 0; i < 8; ++i) assert(base[i] == 255 ? bypass[i] >= 220 : bypass[i] == 0);
  std::cout << "SSD1677 grayscale overlay, factory polarity, cleanup, OTP and RED bypass passed.\n";

  panel.configure(8, 1, FSIM_PANEL_UC8253_X3, false, false, false);
  const auto ucBank = [&](const uint8_t* bb) {
    panel.spiByte(0x24, false);
    for (int i = 0; i < 42; ++i) panel.spiByte(bb[i], true);
  };
  const auto ucRefresh = [&]() {
    panel.spiByte(0x12, false); machine.clock().advanceExternal(2000000); panel.tick();
  };
  ucBank(freeink::lut_x3_bb_normal); plane(0x13, 0x55); ucRefresh();
  assert(panel.currentFrame().pixels == base);
  // DTM1 carries the LSB mask, DTM2 the MSB mask; idle pairs preserve ink.
  plane(0x10, 0x18); plane(0x13, 0x28);
  ucBank(freeink::lut_x3_bb_gc); ucRefresh();
  assert((panel.currentFrame().pixels == std::vector<uint8_t>{0, 255, 170, 255, 85, 255, 0, 255}));
  ucBank(freeink::lut_x3_bb_normal); plane(0x13, 0x55); ucRefresh();
  assert(panel.currentFrame().pixels == base);
  std::cout << "UC8253 grayscale nudge preserves its base and returns to B/W.\n";
}
