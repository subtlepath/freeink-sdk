# FreeInk device simulator

Run FreeInk firmware on your machine, drive it from a CLI, and screenshot the
panel — no device needed. Full guide: [docs/simulator.md](../../docs/simulator.md).

Two kinds of firmware go in. **A bundle** is your source compiled for the host —
fast, and what you want while writing firmware. **A device image** is the `.bin`
that would be flashed, run on an emulated SoC — slower, and the only way to run
firmware you did not build.

```sh
sh tools/simulator/build/build-daemon.sh

# Your own firmware, compiled for the host:
sh tools/simulator/build/build-firmware.sh --device X4CLASSIC src/main.cpp
tools/simulator/build/freeink-simd tools/simulator/build/x4classic.dylib

# Or a device image, with no source at all. It has no board profile of its own
# and no card, and these devices boot only while their power key is held:
tools/simulator/build/freeink-simd firmware-x3-x4-v1.5.1.bin --device X3 --sd ~/books
tools/simulator/cli/freeink-sim hold power
```

```sh
tools/simulator/cli/freeink-sim status
tools/simulator/cli/freeink-sim press confirm
tools/simulator/cli/freeink-sim capture screen.png --wait-refresh
tools/simulator/cli/freeink-sim log --follow
tools/simulator/cli/freeink-sim emu            # on an emulated image
```

The firmware is not stubbed: the real display facade, panel driver, `EpdBus` and
`InputManager` run, against a virtual controller that decodes their actual SPI
traffic and a GPIO matrix that carries real pin levels. An emulated image is not
stubbed either — its instructions are executed against a modelled chip, with
mask ROM intercepted at the call rather than shipped as a blob, and its GPIO,
I2C, SPI and card controllers wired to the same virtual devices a bundle drives.
Both vendor images boot to their own interface: the home screen, the file
browser reading a FAT32 card built from a host directory, and the keys.

Both architectures run: RISC-V for the X3 and X4, Xtensa LX7 — register windows,
both cores — for the X4 Pro and X4 Classic. Waveform physics, the FAT layer's
fragmentation and PSRAM are not modelled — see the guide's limits tables before
trusting a green run.

For a firmware that boots and then sits there, `freeink-emu boot <image>
--tasks --blocks --profile 20 --trace-bus` is the set that turns "it hangs" into
a cause: which task is blocked, which peripheral block it is hammering, which
interrupt nothing is clearing, and what it last said on a bus.

Requires a C++17 compiler, Python 3, and optionally SDL2 for a window
(`brew install sdl2`, `apt install libsdl2-dev`; without it the daemon builds
headless).

Tests: `sh tools/simulator/test/run.sh` and
`sh tools/simulator/test/run-emulator.sh`
(add `FSIM_TEST_IMAGE=<vendor.bin>` to the second for the vendor-image checks).
