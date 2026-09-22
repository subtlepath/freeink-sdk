#pragma once

// FreeInk simulator — BoardProfile -> fsim_board_desc.
//
// The daemon needs the pinout to route traffic: which pad is the panel's D/C,
// which GPIOs are buttons, which I2C address the digitizer answers on. Rather
// than keep a second copy of that in the simulator — guaranteed to drift — it
// is read out of the SDK's own BoardConfig, here, once.
//
// Two callers share this translation, which is why it is a header rather than
// living in either of them:
//
//   * a firmware bundle's entry stub, which reports BoardConfig::ACTIVE as the
//     firmware it was compiled with sees it;
//   * the board table the simulator uses for a *device image*, which carries no
//     profile of its own and has to be told which board it is running on.
//
// Adding a board to the SDK therefore makes it simulatable both ways with no
// change here and none in the daemon.

#include <BoardConfig.h>
#include <freeink_sim_abi.h>

namespace freeink::sim::boards {

inline uint8_t panelControllerFor(BoardConfig::DisplayController controller) {
  switch (controller) {
    case BoardConfig::DisplayController::SSD1677: return FSIM_PANEL_SSD1677;
    case BoardConfig::DisplayController::UC8253: return FSIM_PANEL_UC8253_X3;
    case BoardConfig::DisplayController::UC8179: return FSIM_PANEL_UC8179;
    case BoardConfig::DisplayController::UC8279: return FSIM_PANEL_UC8279;
    default: return FSIM_PANEL_EXTERNAL;
  }
}

inline uint8_t inputStyleFor(BoardConfig::InputStyle style) {
  switch (style) {
    case BoardConfig::InputStyle::XteinkAdcLadder: return FSIM_INPUT_ADC_LADDER;
    case BoardConfig::InputStyle::OnePageAdcLadder: return FSIM_INPUT_ONEPAGE_LADDER;
    default: return FSIM_INPUT_DIGITAL;
  }
}

inline uint8_t touchControllerFor(BoardConfig::TouchController controller) {
  switch (controller) {
    case BoardConfig::TouchController::Chsc6x: return FSIM_TOUCH_CHSC6X;
    case BoardConfig::TouchController::Gt911: return FSIM_TOUCH_GT911;
    case BoardConfig::TouchController::Ft5x06: return FSIM_TOUCH_FT5X06;
    case BoardConfig::TouchController::Ft6336u: return FSIM_TOUCH_FT6336U;
    case BoardConfig::TouchController::Gslx680: return FSIM_TOUCH_GSLX680;
    default: return FSIM_TOUCH_NONE;
  }
}

// Whether the panel's gates scan in reverse of RAM row order. This tracks which
// driver the SDK selects, because the compensation lives in the driver:
//
//   * Ssd1677Driver    — "Gates are physically reversed on this panel": it
//                        programs a descending Y window on every RAM write.
//   * Uc8253X3Driver   — maps logical y to (height-1-y) gate coordinates.
//   * Uc8279Driver     — the X3's UltraChip variant, same gate mapping.
//   * Uc8179Driver,
//     Uc8279X4Driver   — the X4-family panels; no reversal.
//
// If this is ever wrong for a new board the symptom is unmistakable: captures
// come out vertically flipped.
inline bool gatesReversedFor(const BoardConfig::BoardProfile& profile) {
  if (profile.displayController == BoardConfig::DisplayController::SSD1677) return true;
  // The X3 panel (792x528) and its UC8279d variant both reverse; the X4
  // family's UltraChip panels (800x480) do not.
  return profile.board == BoardConfig::Board::XteinkX3 ||
         profile.board == BoardConfig::Board::XteinkX3Uc8279;
}

inline fsim_board_desc describe(const BoardConfig::BoardProfile& profile) {
  fsim_board_desc desc{};

  desc.board_name = profile.name;
  desc.panel_width = profile.displayWidth;
  desc.panel_height = profile.displayHeight;
  desc.panel_controller = panelControllerFor(profile.displayController);
  desc.epd_sclk = profile.display.sclk;
  desc.epd_mosi = profile.display.mosi;
  desc.epd_cs = profile.display.cs;
  desc.epd_dc = profile.display.dc;
  desc.epd_rst = profile.display.rst;
  desc.epd_busy = profile.display.busy;
  desc.epd_power_enable = profile.display.powerEnable;
  desc.epd_mirror_x = profile.orientation.mirrorX ? 1 : 0;
  desc.epd_mirror_y = profile.orientation.mirrorY ? 1 : 0;
  desc.epd_gates_reversed = gatesReversedFor(profile) ? 1 : 0;

  desc.input_style = inputStyleFor(profile.inputStyle);
  desc.buttons[0] = profile.input.back;
  desc.buttons[1] = profile.input.confirm;
  desc.buttons[2] = profile.input.left;
  desc.buttons[3] = profile.input.right;
  desc.buttons[4] = profile.input.up;
  desc.buttons[5] = profile.input.down;
  desc.buttons[6] = profile.input.power;
  desc.power_active_high = profile.input.powerActiveHigh ? 1 : 0;
  desc.adc_ladder_pin = profile.input.adcLadderPin;

  desc.touch_controller = touchControllerFor(profile.touch.controller);
  desc.touch_sda = profile.touch.sda;
  desc.touch_scl = profile.touch.scl;
  desc.touch_irq = profile.touch.irq;
  desc.touch_rst = profile.touch.reset;
  desc.touch_addr = profile.touch.i2cAddress;
  desc.touch_addr_alt = profile.touch.i2cAddressAlt;
  desc.touch_raw_max_x = profile.touch.rawMaxX;
  desc.touch_raw_max_y = profile.touch.rawMaxY;
  desc.touch_swap_xy = profile.touch.swapXY ? 1 : 0;
  desc.touch_flip_x = profile.touch.flipX ? 1 : 0;
  desc.touch_flip_y = profile.touch.flipY ? 1 : 0;
  desc.touch_irq_active_low = profile.touch.irqActiveLow ? 1 : 0;
  desc.touch_power_enable = profile.touch.powerEnable;
  desc.touch_power_active_high = profile.touch.powerEnableActiveHigh ? 1 : 0;

  desc.battery_adc = profile.batteryAdc;
  desc.battery_divider = profile.batteryDividerMultiplier;
  desc.battery_charge_status = profile.batteryChargeStatus;
  desc.battery_charge_active_high = profile.batteryChargeStatusActiveHigh ? 1 : 0;
  desc.sensors_sda = profile.sensors.i2cSda;
  desc.sensors_scl = profile.sensors.i2cScl;
  desc.gauge_addr = profile.batteryGauge.gaugeAddr;
  desc.rtc_addr = profile.sensors.rtcAddr;
  desc.imu_addr = profile.sensors.imuAddr;

  desc.frontlight_pin = profile.frontlight.gpio;
  desc.frontlight_i2c_addr =
      profile.i2cFrontlight.controller == BoardConfig::I2cFrontlightController::None
          ? 0
          : profile.i2cFrontlight.address;
  desc.sd_power_enable = profile.sd.powerEnable;
  desc.sd_power_active_high = profile.sd.powerActiveHigh ? 1 : 0;
  desc.sd_sclk = profile.sd.sclk;
  desc.sd_miso = profile.sd.miso;
  desc.sd_mosi = profile.sd.mosi;
  desc.sd_cs = profile.sd.cs;
  desc.sd_bus_width = profile.sdmmc.busWidth;
  desc.sdmmc_clk = profile.sdmmc.clk;
  desc.sdmmc_cmd = profile.sdmmc.cmd;
  desc.sdmmc_d0 = profile.sdmmc.d0;
  desc.sdmmc_d1 = profile.sdmmc.d1;
  desc.sdmmc_d2 = profile.sdmmc.d2;
  desc.sdmmc_d3 = profile.sdmmc.d3;

  return desc;
}

}  // namespace freeink::sim::boards
