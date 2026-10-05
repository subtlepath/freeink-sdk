#!/bin/sh
set -eu
HERE=$(cd "$(dirname "$0")" && pwd)
SIM=$(cd "$HERE/.." && pwd)
SDK=$(cd "$SIM/../.." && pwd)
OUT=${FSIM_WEB_OUT:-"$HERE/public/runtime"}
mkdir -p "$OUT/rom"
cp "$SIM"/emu/rom/*.romsyms "$OUT/rom/"
set -- "$HERE/emulator.cpp" "$SIM/boards/BoardTable.cpp"
for name in Machine Panel I2cDevices VirtualCard Fat32Image SdSpiCard SdNativeCard EmuBoard; do
  set -- "$@" "$SIM/core/$name.cpp"
done
for name in Soc Image Md5 Bus Rom RomHandlers RomFlash RiscvCore XtensaCore Peripherals BoardPeripherals FlashController EmuMachine; do
  set -- "$@" "$SIM/emu/$name.cpp"
done
"${EMXX:-em++}" -std=c++17 -O2 \
  -DFREEINK_DEVICE_X3=1 -DFSIM_SOC_SPI_PERIPH_NUM=1 -DFSIM_SOC_GPIO_PIN_COUNT=22 \
  -I"$SIM/include" -I"$SIM/platform" -I"$SIM/emu" -I"$SDK/libs/hardware/BoardConfig/include" \
  "$@" --no-entry -sMODULARIZE=1 -sEXPORT_ES6=1 -sENVIRONMENT=web,worker,node \
  -sALLOW_MEMORY_GROWTH=1 -sINITIAL_MEMORY=67108864 -sMAXIMUM_MEMORY=536870912 \
  '-sEXPORTED_RUNTIME_METHODS=["FS","ccall","UTF8ToString","HEAPU8"]' -o "$OUT/emulator.mjs"
