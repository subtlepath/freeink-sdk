#!/bin/sh
# Builds a FreeInk firmware bundle for the simulator.
#
# A bundle is a shared library holding the firmware sketch, the FreeInk SDK, and
# the host platform shims. It resolves its fsim_* symbols from the daemon at
# dlopen() time, so it is not standalone and cannot be run directly.
#
#   sh tools/simulator/build/build-firmware.sh --device X3 main.cpp
#   sh tools/simulator/build/build-firmware.sh --device X4CLASSIC --out fw.dylib src/*.cpp
#
# Options:
#   --device NAME    FreeInk device (X3, X4, X4CLASSIC, X4PRO, ...). Required:
#                    it selects the board profile, and there is no default.
#   --out PATH       Output bundle (default: build/<device>.dylib|.so)
#   --define FLAG    Extra -D to pass through; repeatable
#   --cap NAME       Enable a FreeInk capability (-DFREEINK_CAP_<NAME>)
#   --lib PATH       Extra include/source root under libs/ to compile in
#   Remaining arguments are the firmware's own source files.

set -e

SIM_DIR=$(cd "$(dirname "$0")/.." && pwd)
SDK_DIR=$(cd "$SIM_DIR/../.." && pwd)
CXX=${CXX:-c++}

DEVICE=""
OUT=""
DEFINES=""
SOURCES=""
EXTRA_LIBS=""

while [ $# -gt 0 ]; do
  case "$1" in
    --device) DEVICE="$2"; shift 2 ;;
    --out) OUT="$2"; shift 2 ;;
    --define) DEFINES="$DEFINES -D$2"; shift 2 ;;
    --cap) DEFINES="$DEFINES -DFREEINK_CAP_$2=1"; shift 2 ;;
    --lib) EXTRA_LIBS="$EXTRA_LIBS $2"; shift 2 ;;
    -h|--help) sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) SOURCES="$SOURCES $1"; shift ;;
  esac
done

if [ -z "$DEVICE" ]; then
  echo "build-firmware.sh: --device is required (X3, X4, X4CLASSIC, X4PRO, ...)" >&2
  echo "It selects the FreeInk board profile; the SDK has no default." >&2
  exit 2
fi
if [ -z "$SOURCES" ]; then
  echo "build-firmware.sh: no firmware sources given" >&2
  exit 2
fi

case "$(uname -s)" in
  Darwin) EXT="dylib"; SHARED="-dynamiclib -undefined dynamic_lookup" ;;
  *)      EXT="so";    SHARED="-shared" ;;
esac
[ -n "$OUT" ] || OUT="$SIM_DIR/build/$(echo "$DEVICE" | tr 'A-Z' 'a-z').$EXT"

# The MCU family follows the device, exactly as BoardConfig derives it. It only
# affects the soc_caps the shim reports, so the firmware takes the same
# compile-time branches it would on the real part.
case "$DEVICE" in
  X3|X4)     SOC="-DFSIM_SOC_SPI_PERIPH_NUM=1 -DFSIM_SOC_GPIO_PIN_COUNT=22" ;;
  ONEPAGE)   SOC="-DFSIM_SOC_SPI_PERIPH_NUM=1 -DFSIM_SOC_GPIO_PIN_COUNT=32" ;;
  M5PAPER)   SOC="-DFSIM_SOC_SPI_PERIPH_NUM=2 -DFSIM_SOC_GPIO_PIN_COUNT=40" ;;
  *)         SOC="-DFSIM_SOC_SPI_PERIPH_NUM=2 -DFSIM_SOC_GPIO_PIN_COUNT=48" ;;
esac

INCLUDES="-I$SIM_DIR/include -I$SIM_DIR/platform"
for dir in \
  "$SDK_DIR/libs/hardware/BoardConfig/include" \
  "$SDK_DIR/libs/hardware/InputManager/include" \
  "$SDK_DIR/libs/hardware/BatteryMonitor/include" \
  "$SDK_DIR/libs/hardware/PowerManager/include" \
  "$SDK_DIR/libs/hardware/SDCardManager/include" \
  "$SDK_DIR/libs/hardware/MemoryManager/include" \
  "$SDK_DIR/libs/hardware/FrontlightManager/include" \
  "$SDK_DIR/libs/hardware/LedManager/include" \
  "$SDK_DIR/libs/hardware/Buzzer/include" \
  "$SDK_DIR/libs/hardware/Rtc/include" \
  "$SDK_DIR/libs/hardware/EnvironmentSensor/include" \
  "$SDK_DIR/libs/hardware/Imu/include" \
  "$SDK_DIR/libs/hardware/XteinkDetect/include" \
  "$SDK_DIR/libs/display/FreeInkDisplay/include" \
  "$SDK_DIR/libs/ui/FreeInkUI/include" \
  "$SDK_DIR/libs/assets/Icons/include" \
  $EXTRA_LIBS
do
  [ -d "$dir" ] && INCLUDES="$INCLUDES -I$dir"
done

# SDK translation units the simulator always needs: the display facade, its
# native panel drivers and bus, and the input abstraction. Capability-gated
# managers are compiled in too; they gate themselves on the board profile.
SDK_SOURCES="$SDK_DIR/libs/display/FreeInkDisplay/src/FreeInkDisplay.cpp
$SDK_DIR/libs/display/FreeInkDisplay/src/bus/EpdBus.cpp
$SDK_DIR/libs/display/FreeInkDisplay/src/driver/Ssd1677Driver.cpp
$SDK_DIR/libs/display/FreeInkDisplay/src/driver/Uc8253X3Driver.cpp
$SDK_DIR/libs/display/FreeInkDisplay/src/driver/Uc8179Driver.cpp
$SDK_DIR/libs/display/FreeInkDisplay/src/driver/Uc8279Driver.cpp
$SDK_DIR/libs/display/FreeInkDisplay/src/driver/Uc8279X4Driver.cpp
$SDK_DIR/libs/hardware/InputManager/src/InputManager.cpp"

SDK_COMPILE=""
for src in $SDK_SOURCES; do
  [ -f "$src" ] && SDK_COMPILE="$SDK_COMPILE $src"
done

SHIM_SOURCES="$SIM_DIR/platform/sim_bridge.cpp
$SIM_DIR/platform/sim_rtos.cpp
$SIM_DIR/platform/sim_fs.cpp
$SIM_DIR/platform/sim_entry.cpp"

set -x
$CXX -std=c++17 -O1 -g -fPIC -pthread \
  $SHARED \
  -DFREEINK_DEVICE_$DEVICE=1 \
  -DFSIM_FIRMWARE_NAME="\"$(basename "$OUT")\"" \
  -DFSIM_BUILD_FLAGS="\"-DFREEINK_DEVICE_$DEVICE$DEFINES\"" \
  $SOC $DEFINES \
  $INCLUDES \
  $SHIM_SOURCES $SDK_COMPILE $SOURCES \
  -o "$OUT"
set +x

echo "built $OUT"
echo "  freeink-sim load $OUT"
