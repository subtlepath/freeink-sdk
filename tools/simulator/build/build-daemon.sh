#!/bin/sh
# Builds the FreeInk simulator daemon (freeink-simd) and the freeink-emu tool.
#
# SDL2 is optional: without it the daemon still builds and runs headless, which
# is the mode CI and agent loops use. With it you also get a window.
#
#   sh tools/simulator/build/build-daemon.sh          # auto-detect SDL2
#   FSIM_NO_SDL=1 sh .../build-daemon.sh              # force headless build
#   FSIM_OUT=/path/freeink-simd sh .../build-daemon.sh

set -e

SIM_DIR=$(cd "$(dirname "$0")/.." && pwd)
SDK_DIR=$(cd "$SIM_DIR/../.." && pwd)
OUT=${FSIM_OUT:-"$SIM_DIR/build/freeink-simd"}
CXX=${CXX:-c++}

SDL_CFLAGS=""
SDL_LIBS=""
HAVE_SDL=0
if [ -z "$FSIM_NO_SDL" ] && command -v sdl2-config >/dev/null 2>&1; then
  SDL_CFLAGS=$(sdl2-config --cflags)
  # -lSDL2main provides an entry point this daemon does not want; Window.cpp
  # sets SDL_MAIN_HANDLED and keeps its own main().
  SDL_LIBS=$(sdl2-config --libs | sed 's/-lSDL2main//')
  HAVE_SDL=1
elif [ -z "$FSIM_NO_SDL" ] && command -v pkg-config >/dev/null 2>&1 && pkg-config --exists sdl2; then
  SDL_CFLAGS=$(pkg-config --cflags sdl2)
  SDL_LIBS=$(pkg-config --libs sdl2 | sed 's/-lSDL2main//')
  HAVE_SDL=1
fi

if [ "$HAVE_SDL" = "0" ]; then
  echo "SDL2 not found — building a headless daemon."
  echo "  macOS:  brew install sdl2        Debian/Ubuntu:  apt install libsdl2-dev"
fi

# -rdynamic / -export_dynamic: the firmware bundle resolves its fsim_* symbols
# from this executable at dlopen() time, so they must be in the dynamic table.
case "$(uname -s)" in
  Darwin) EXPORT_FLAGS="-Wl,-export_dynamic" ;;
  *)      EXPORT_FLAGS="-rdynamic" ;;
esac

set -x
$CXX -std=c++17 -O2 -g -Wall -Wextra -pthread \
  -DFSIM_HAVE_SDL=$HAVE_SDL \
  -DFREEINK_DEVICE_X3=1 -DFSIM_SOC_SPI_PERIPH_NUM=1 -DFSIM_SOC_GPIO_PIN_COUNT=22 \
  -I"$SIM_DIR/include" \
  -I"$SIM_DIR/emu" \
  -I"$SDK_DIR/libs/hardware/BoardConfig/include" \
  -I"$SIM_DIR/platform" \
  $SDL_CFLAGS \
  "$SIM_DIR/core/Machine.cpp" \
  "$SIM_DIR/core/Panel.cpp" \
  "$SIM_DIR/core/I2cDevices.cpp" \
  "$SIM_DIR/core/VirtualCard.cpp" \
  "$SIM_DIR/core/Fat32Image.cpp" \
  "$SIM_DIR/core/SdSpiCard.cpp" \
  "$SIM_DIR/core/SdNativeCard.cpp" \
  "$SIM_DIR/core/Abi.cpp" \
  "$SIM_DIR/core/EmuBoard.cpp" \
  "$SIM_DIR/boards/BoardTable.cpp" \
  "$SIM_DIR/emu/Soc.cpp" \
  "$SIM_DIR/emu/Image.cpp" \
  "$SIM_DIR/emu/Md5.cpp" \
  "$SIM_DIR/emu/Bus.cpp" \
  "$SIM_DIR/emu/Rom.cpp" \
  "$SIM_DIR/emu/RomHandlers.cpp" \
  "$SIM_DIR/emu/RomFlash.cpp" \
  "$SIM_DIR/emu/RiscvCore.cpp" \
  "$SIM_DIR/emu/XtensaCore.cpp" \
  "$SIM_DIR/emu/Peripherals.cpp" \
  "$SIM_DIR/emu/BoardPeripherals.cpp" \
  "$SIM_DIR/emu/FlashController.cpp" \
  "$SIM_DIR/emu/EmuMachine.cpp" \
  "$SIM_DIR/daemon/Runtime.cpp" \
  "$SIM_DIR/daemon/Server.cpp" \
  "$SIM_DIR/daemon/Png.cpp" \
  "$SIM_DIR/daemon/Window.cpp" \
  "$SIM_DIR/daemon/main.cpp" \
  $EXPORT_FLAGS $SDL_LIBS -lz -ldl \
  -o "$OUT"
set +x

echo "built $OUT"

# freeink-emu: the same emulator, without the daemon around it. Useful for
# inspecting an image, and for a boot that should fail a test rather than hang.
EMU_OUT=${FSIM_EMU_OUT:-"$SIM_DIR/build/freeink-emu"}
set -x
# freeink-emu carries the board model too: an image needs a board to be a
# device, and the tool is how a test boots one and checks what it painted.
$CXX -std=c++17 -O2 -g -Wall -Wextra -pthread \
  -I"$SIM_DIR/include" \
  -I"$SIM_DIR/emu" \
  -I"$SDK_DIR/libs/hardware/BoardConfig/include" \
  -I"$SIM_DIR/platform" \
  -DFREEINK_DEVICE_X3=1 -DFSIM_SOC_SPI_PERIPH_NUM=1 -DFSIM_SOC_GPIO_PIN_COUNT=22 \
  "$SIM_DIR/core/Machine.cpp" \
  "$SIM_DIR/core/Panel.cpp" \
  "$SIM_DIR/core/I2cDevices.cpp" \
  "$SIM_DIR/core/VirtualCard.cpp" \
  "$SIM_DIR/core/Fat32Image.cpp" \
  "$SIM_DIR/core/SdSpiCard.cpp" \
  "$SIM_DIR/core/SdNativeCard.cpp" \
  "$SIM_DIR/core/EmuBoard.cpp" \
  "$SIM_DIR/boards/BoardTable.cpp" \
  "$SIM_DIR/daemon/Png.cpp" \
  "$SIM_DIR/emu/Soc.cpp" \
  "$SIM_DIR/emu/Image.cpp" \
  "$SIM_DIR/emu/Md5.cpp" \
  "$SIM_DIR/emu/Bus.cpp" \
  "$SIM_DIR/emu/Rom.cpp" \
  "$SIM_DIR/emu/RomHandlers.cpp" \
  "$SIM_DIR/emu/RomFlash.cpp" \
  "$SIM_DIR/emu/RiscvCore.cpp" \
  "$SIM_DIR/emu/XtensaCore.cpp" \
  "$SIM_DIR/emu/Peripherals.cpp" \
  "$SIM_DIR/emu/BoardPeripherals.cpp" \
  "$SIM_DIR/emu/FlashController.cpp" \
  "$SIM_DIR/emu/EmuMachine.cpp" \
  "$SIM_DIR/emu/EmuTool.cpp" \
  -lz \
  -o "$EMU_OUT"
set +x

echo "built $EMU_OUT"
