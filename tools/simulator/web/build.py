#!/usr/bin/env python3
"""Compile real firmware sources and the FreeInk simulator for the browser."""
import argparse
import pathlib
import subprocess
import shutil

HERE = pathlib.Path(__file__).resolve().parent
SDK = HERE.parents[2]
SIM = HERE.parent

def build_tinta(root, device, compiler, out):
    compat = out / "compat"
    compat.mkdir(parents=True, exist_ok=True)
    # Same compatibility fix as Tinta's native tools/sim/build.sh.
    text = (SDK / "libs/hardware/BoardConfig/include/M5Pm1.h").read_text()
    (compat / "M5Pm1.h").write_text(text.replace("Wire.requestFrom(ADDR, len)", "Wire.requestFrom(ADDR, static_cast<size_t>(len))"))
    includes = [root / "tools/sim/compat", compat, root / "src/platform/sdk", SIM / "include", SIM / "platform", root / "src"]
    includes += sorted((SDK / "libs").glob("*/*/include"))
    sources = [SIM / "core" / f"{s}.cpp" for s in ["Machine", "Panel", "I2cDevices", "VirtualCard", "Fat32Image", "SdSpiCard", "SdNativeCard", "Abi"]]
    sources += [SIM / "platform" / f"{s}.cpp" for s in ["sim_bridge", "sim_rtos", "sim_fs", "sim_entry"]]
    display = SDK / "libs/display/FreeInkDisplay/src"
    sources += [display / "FreeInkDisplay.cpp", display / "bus/EpdBus.cpp"]
    sources += [display / f"driver/{s}Driver.cpp" for s in ["Ssd1677", "Uc8253X3", "Uc8179", "Uc8279", "Uc8279X4"]]
    for library in ["InputManager", "XteinkDetect", "BatteryMonitor", "PowerManager", "FrontlightManager"]:
        sources += sorted((SDK / f"libs/hardware/{library}/src").rglob("*.cpp"))
    sources += [SDK / "libs/ui/FreeInkUI/src/FreeInkUI.cpp"]
    sources += sorted((root / "src").rglob("*.cpp"))
    # TINTA_SIM loads the simulator course pack, so the device's binary asset
    # wrapper (incbin) is unnecessary and not portable to wasm assemblers.
    sources = [s for s in sources if s != root / "src/assets/CoursePack.cpp"]
    sources += [HERE / "firmware.cpp"]
    target = out / f"tinta-{device.lower()}.mjs"
    args = [compiler, "-std=c++17", "-O2", "-pthread", "-DFSIM_WEB=1", "-DTINTA_SIM=1", "-DEINK_DISPLAY_SINGLE_BUFFER_MODE=1", f"-DFREEINK_DEVICE_{device}=1",
            "-DFSIM_SOC_SPI_PERIPH_NUM=" + ("1" if device == "X3" else "2"),
            "-DFSIM_SOC_GPIO_PIN_COUNT=" + ("22" if device == "X3" else "48")]
    args += [f"-I{p}" for p in includes]
    args += [str(s) for s in sources]
    args += ["--no-entry", "-sMODULARIZE=1", "-sEXPORT_ES6=1", "-sENVIRONMENT=web,worker,node", "-sALLOW_MEMORY_GROWTH=1", "-sINITIAL_MEMORY=67108864", "-sMAXIMUM_MEMORY=536870912", "-sPTHREAD_POOL_SIZE=6", "-sDEFAULT_PTHREAD_STACK_SIZE=2097152", "-sEXPORTED_RUNTIME_METHODS=['FS','ccall','UTF8ToString','HEAPU8']", "-o", str(target)]
    print(f"Building Tinta {device}", flush=True)
    subprocess.run(args, check=True)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--spanish", type=pathlib.Path, default=SDK.parent / "spanish")
    parser.add_argument("--device", choices=["X3", "X4CLASSIC", "X4PRO", "all"], default="all")
    parser.add_argument("--compiler", default=shutil.which("em++") or "em++")
    parser.add_argument("--out", type=pathlib.Path, default=HERE / "public/runtime")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    pack = args.spanish / "build/sim/course-sim.pack"
    if not pack.is_file():
        parser.error("Build the Tinta simulator fixture first: sh ../spanish/tools/sim/build.sh X3")
    shutil.copyfile(pack, args.out / "course.pack")
    devices = ["X3", "X4CLASSIC", "X4PRO"] if args.device == "all" else [args.device]
    for device in devices:
        build_tinta(args.spanish.resolve(), device, args.compiler, args.out.resolve())

if __name__ == "__main__":
    main()
