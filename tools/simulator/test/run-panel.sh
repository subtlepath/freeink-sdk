#!/bin/sh
set -eu
SIM_DIR=$(cd "$(dirname "$0")/.." && pwd)
SDK_DIR=$(cd "$SIM_DIR/../.." && pwd)
OUT="$SIM_DIR/build/panel-grayscale-test"
set -- "$SIM_DIR/test/panel-grayscale.cpp"
for name in Machine Panel I2cDevices VirtualCard Fat32Image SdSpiCard SdNativeCard; do
  set -- "$@" "$SIM_DIR/core/$name.cpp"
done
"${CXX:-c++}" -std=c++17 -O1 -pthread \
  -DFREEINK_DEVICE_X3=1 -DFSIM_SOC_SPI_PERIPH_NUM=1 -DFSIM_SOC_GPIO_PIN_COUNT=22 \
  -I"$SIM_DIR/include" -I"$SIM_DIR/platform" -I"$SDK_DIR/libs/hardware/BoardConfig/include" \
  "$@" -o "$OUT"
"$OUT"
