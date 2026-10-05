#!/bin/sh
set -eu
SIM_DIR=$(cd "$(dirname "$0")/.." && pwd)
SDK_DIR=$(cd "$SIM_DIR/../.." && pwd)
OUT="$SIM_DIR/build/rtos-queue-test"
"${CXX:-c++}" -std=c++17 -O1 -pthread \
  -DFREEINK_DEVICE_X3=1 -DFSIM_SOC_SPI_PERIPH_NUM=1 -DFSIM_SOC_GPIO_PIN_COUNT=22 \
  -I"$SIM_DIR/include" -I"$SIM_DIR/platform" -I"$SDK_DIR/libs/hardware/BoardConfig/include" \
  "$SIM_DIR/test/rtos-queue.cpp" "$SIM_DIR/platform/sim_rtos.cpp" -o "$OUT"
"$OUT"
