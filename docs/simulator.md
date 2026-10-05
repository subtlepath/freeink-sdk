# FreeInk device simulator

For browser demos of Tinta and lila on X3, X4 Classic and X4 Pro, plus Web Serial
installation and sp2 site integration, see [the web simulator](../tools/simulator/web/README.md).

A host simulator for FreeInk firmware, in the shape of Apple's platform
simulators: a long-running daemon that models a device, and a CLI that drives it
over a Unix socket. It exists so firmware can be built, run, driven and
screenshotted without a device in hand — by a person at a keyboard, by a script,
or by an agent in a loop.

There are two ways to get firmware into it, and they answer different questions:

| | **Bundle** | **Device image** |
|---|---|---|
| What it is | your source, compiled for the host | the `.bin` you would flash |
| Built by | `build/build-firmware.sh` | your device build, or a vendor |
| Needs the source? | yes | no |
| Speed | native | 50–70 M emulated instructions/s |
| Answers | "does my change work?" | "what does this firmware do?" |

Both drive the same virtual machine — the same panel, the same I2C devices, the
same card, the same keys — so every command in this guide works either way. The
daemon tells them apart by reading the file.

```
   firmware.cpp ──┐                              firmware.bin
                  ├─▶ clang (host) ─▶ bundle          │ (ESP32 image)
   FreeInk SDK  ──┘                    │ dlopen       │ emulated SoC
                                       ▼              ▼
                                        freeink-simd (daemon)
              ┌──────────────┬──────────────────┼───────────────┐
        virtual panel    GPIO matrix       virtual I2C      storage, power,
        (SSD1677,        + ADC ladders     (GT911, RTC,     audio, Wi-Fi,
         UC8253, …)                         gauge, IMU)      USB
                                                │
                              SDL2 window   ────┴──── /…/sim.sock
                              (optional)                │
                                                  freeink-sim (CLI)
```

## What it actually simulates

The firmware under test is compiled for the host, but **it is not stubbed out**.
The real `FreeInkDisplay` facade, the real panel driver, the real `EpdBus` and
the real `InputManager` all run. What they talk to is a model:

- **The panel is driven over the bus, not through a back door.** The virtual
  controller decodes the SPI byte stream, separating commands from data by the
  D/C pin exactly as the glass does. RAM writes land at the window and cursor
  the driver programmed, in the data-entry direction it selected. A display
  update runs a waveform that takes time and holds BUSY, and `EpdBus`'s
  interrupt-driven completion wait wakes on a real BUSY pin edge.

  So a driver that programs the wrong RAM window, streams planes in the wrong
  order, forgets to lower D/C before a command, or fires a refresh without
  waiting for BUSY produces a wrong or missing image here, as it would on glass.

- **Input is electrical.** A button press pulls its GPIO to ground, or puts the
  matching voltage on the ADC resistor ladder; the SDK's own debounce and ladder
  decode do the rest. The X3's two-ladder scheme and the X4 Classic's seven
  discrete keys both work because the board profile says so, not because the
  simulator has a button API.

- **I2C devices answer or NACK.** The GT911, BM8563/PCF8563, CW2017, BQ27220 and
  QMI8658 answer register reads the way their silicon does; an address with
  nothing behind it NACKs. That is what makes controller autodetection and
  "is this peripheral fitted?" paths testable instead of assumed.

- **Pads, rails and sleep behave.** A pin held for deep sleep ignores writes
  until released — the latch `EpdBus` and `PowerManager` work around on wake.
  Deep sleep parks the firmware and a wake source re-enters `setup()` with the
  wake cause set.

### What it does not simulate

Be clear about the limits before trusting a green run:

| Not modelled | Consequence |
|---|---|
| Waveform physics | Grey levels are the driver's intent, not measured optical response. LUT contents are recorded, not simulated. **Panel tuning still needs hardware.** |
| Ghosting | Approximated from refresh mode and history — enough to see that a screen wants a full refresh, not a prediction of what the glass will look like. |
| The FAT layer, for a bundle | A bundle's SD card is a host directory: file I/O is real, but cluster allocation, LFN encoding, corrupt-volume recovery and raw sector access are not. An emulated *image* gets a real FAT32 volume and reads it through its own driver, so for images only fragmentation is missing — every file is laid out contiguously. |
| Analogue and RF | Bus timing is charged at the configured clock, but signal integrity, SPI glitches, brownouts and radio behaviour are absent. |
| ESP-IDF itself | Bootloader, partition checks, linker/section placement, PSRAM behaviour and real task scheduling are host approximations. A build that runs here can still fail to flash or fit. |

The simulator replaces the edit/run/look cycle. It does not replace bring-up on
hardware, and `docs/testing.md`'s validation limits still apply.

## Running a device image

A device image is the artefact that gets flashed: `build/<app>.bin` from an
ESP-IDF build, or a vendor's firmware release. Running one means emulating the
SoC, because there is no source to compile — the instructions in the file are
executed one at a time against a modelled chip.

```sh
tools/simulator/build/freeink-emu info firmware-x3-x4-v1.5.1.bin   # what is this?
freeink-sim load firmware-x3-x4-v1.5.1.bin                         # run it
freeink-sim emu                                                    # how is it doing?
freeink-sim log --channel firmware                                 # its console
```

`load` takes an image or a bundle and works out which it has by reading the
file, so nothing else about the workflow changes: `pause`, `step`, `capture`,
`battery`, `nvs` and the rest behave the same.

An image needs two things a bundle brings with it — a board to be running on,
and a card to read — and these devices need a third, which is their power key
held while they boot. All three are below; the short version is:

```sh
freeink-simd firmware-x3-x4-v1.5.1.bin --device X3 --sd ~/books
freeink-sim hold power        # then release it once the interface appears
```

### What is emulated

- **The CPU, instruction by instruction, in both architectures.** RV32IMC for
  the ESP32-C3 (the X3 and the X4), including the compressed encodings, the M
  extension, machine-mode CSRs, traps and the chip's own interrupt controller
  — its priority/threshold gating and its vectored dispatch. Xtensa LX7 for the
  ESP32-S3 (the X4 Pro and the X4 Classic): the core ISA with the density,
  loop, 32-bit multiply and divide, min/max, NSA, SALT, boolean and
  single-precision floating-point options, and — the part that makes it Xtensa
  — **the register window**.

  The window is not a detail that can be papered over. The LX7 has 64 physical
  registers and shows sixteen; a call rotates that view rather than saving
  anything, and when the rotation would run onto a frame that is still live the
  hardware raises an exception and *the firmware's own handler* writes that
  frame out to its stack. So the emulator checks every register operand against
  what the current frame owns, exactly as the silicon does, and dispatches
  overflow and underflow to the vectors the application installed. Getting it
  wrong does not produce a wrong number somewhere — it produces a firmware that
  spills the wrong registers and returns into nothing.

  About 70 million instructions a second for RISC-V, 50 million for Xtensa.

- **Both cores of a dual-core part.** The S3's PRO and APP CPUs run in turn
  against one bus, each with its own half of the interrupt matrix. That is not
  optional detail either: an IDF application starts the second core during boot
  and then *waits* for it to report in, so a single-core model never gets a
  dual-core firmware to its first line of output.
- **The flash and its MMU.** The cached instruction and data windows translate
  through the chip's 64 KB page table into a flash model, exactly as the
  silicon does, rather than being preloaded copies. `esp_mmu_map()`, OTA reads
  and a firmware that remaps its own pages all behave.
- **The SPI flash controller.** Transactions composed in the `USER`/`USER1`/
  `USER2` registers are decoded and run as NOR commands — read, page program,
  sector and block erase, JEDEC ID, status. Erase-before-write is enforced, so
  firmware that forgets an erase misbehaves here the way it does on the device.
- **The peripherals boot depends on.** The system timer FreeRTOS ticks from,
  the timer group's RTC clock calibration, the RTC controller's counter, the
  interrupt matrix, the cross-CPU software interrupts that start the first
  task, the UART and USB-serial consoles, the SAR ADC, the hardware RNG.
- **The peripherals that make it a device.** The GPIO matrix and IO_MUX, so a
  pad reads what is driving it and a pin-change interrupt fires; the I2C
  master, decoded from the command list the driver builds into its registers;
  the SPI masters, decoded from the phases composed in their `USER` registers,
  with the payload pulled through GDMA's descriptor chain where the driver put
  it; and the SDMMC host, with its own descriptor-driven DMA. Those four are
  what carry a firmware's traffic to the panel, the digitizer, the RTC, the
  fuel gauge and the card — which is to say, they are the difference between
  running a firmware's instructions and running the device.
- **Mask ROM, at the call rather than the instruction.** See below.

### Mask ROM

An ESP32 application is not self-contained: it calls into the chip's mask ROM
for libc, for libgcc's 64-bit and soft-float helpers, for early UART and GPIO
setup, and for flash access. That ROM is a binary Espressif publishes
separately, and this emulator ships no blob.

Instead, calls into ROM address space are intercepted. The address is looked up
in a symbol map generated from ESP-IDF's published linker scripts — names and
addresses, no code — and a native implementation runs against the guest's
registers and memory. Where a ROM routine takes a callback, such as `qsort`'s
comparator, the emulator calls back into the firmware's own code to run it.

The maps live in `tools/simulator/emu/rom/` and are regenerated with:

```sh
tools/simulator/emu/rom/import-rom-symbols.py --chip esp32c3 --idf ~/esp/esp-idf
tools/simulator/emu/rom/import-rom-symbols.py --chip esp32c3 --ref v5.5   # or fetch them
```

A routine with no implementation stops the machine and names it:

```
ROM routine `esp_rom_spiflash_erase_block` (0x40000148) is not implemented
```

That is a diagnostic, not a dead end — adding one is a table entry in
`RomHandlers.cpp`. The alternative, returning a plausible zero, produces a
failure thousands of instructions later in a place that tells you nothing.

### The flash around the image

An application image is what you flash at 0x10000; it is not a whole flash. A
device also has a partition table at 0x8000, and an IDF app reads it —
`esp_ota_get_running_partition()`, the NVS lookup, every `esp_partition_find()`.

So the loader builds a flash around the image: the app at its offset, a
synthesized partition table (IDF's conventional single-factory layout, with the
app partition grown to fit), and a bootloader header that is well-formed but
never executed. A real dump — anything that already has a partition table at
0x8000 — is used verbatim instead.

The synthesized table is a *guess*. `freeink-sim emu` says when one is in use,
and a real one can be supplied:

```sh
tools/simulator/build/freeink-emu boot app.bin --partitions build/partition_table/partition-table.bin
```

That matters for firmware that reads its own storage partition, and not at all
for firmware that does not.

### The bootloader is not executed

The second-stage bootloader's whole job is to verify the image, set the MMU up
and jump. The emulator does that directly and enters the application at its
entry point with the machine in the state the bootloader leaves it in — which
is exactly the contract `call_start_cpu0` is written against. Emulating the
bootloader would only add its own ROM dependencies without testing anything the
application cares about.

Pass a bootloader image and the loader says so rather than running it.

### Time

Emulating an instruction set costs far more than one host instruction per guest
instruction, so an emulated image cannot keep up with wall time. Loading one
therefore switches the clock to virtual mode and says so in the log: simulated
time is set by what the firmware executed, not by the wall.

This is not a convenience. In realtime mode the clock would run ahead of the
firmware, and every piece of modelled timing — panel waveforms, BUSY, input
sampling — would be keyed to a clock the firmware is behind.

Virtual time has the opposite failure, and loading an image fixes that too. An
idle machine fast-forwards straight to its next timer, so a device that spends
most of its time waiting runs its clock thousands of times faster than the
wall — and the firmware's own inactivity timeout then expires while the
operator is still reaching for the keyboard. So an image's clock is also
**paced**: simulated time advances at most `rate` times wall time.

```sh
freeink-sim clock --rate 4     # four times as fast as the wall
freeink-sim clock --rate 0     # no ceiling, for a test that wants the end
```

A bundle's virtual clock is never paced — there is nothing to pace, and waiting
on the wall would throw away the reason for having one — and `freeink-emu`, the
batch tool, runs unpaced too.

### Debugging an image

A firmware image has no symbols, so the emulator provides the primitives that
let you work from a disassembly:

```sh
freeink-sim mem 0x3FC80000 256          # read emulated memory, with its region named
freeink-sim registers                   # peripheral registers nothing models, most-touched first
freeink-sim registers 0x60023000        # every register touched in one block
freeink-emu boot app.bin --break 0x42001234   # stop with the registers intact
```

`registers` with no argument is the one to reach for when an image behaves
oddly: it is the list of hardware the firmware used that the emulator does not
model, ordered by how much it used it. When a firmware sits in a loop, the
register it is spinning on is usually at the top.

A fault report names the first exception the firmware took — not the panic
handler that ran afterwards and overwrote the evidence — with the control
transfers that led to it:

```
first exception the firmware took: illegal instruction at 0x3FC92CCA, in IRAM+0x16CCA
  how it got there (most recent last):
    0x421C64A4 -> 0x421C7BE8
    0x421C7BD2 -> 0x3FC92CC8
```

### freeink-emu

The same emulator without the daemon around it, for inspecting an image and for
boots that should fail a test rather than hang one:

```sh
freeink-emu info <image>                     # chip, segments, descriptor, partitions
freeink-emu boards                           # boards --device accepts
freeink-emu boot <image> [options]
    --device NAME        board to run it on (X3, X4, X4PRO, X4CLASSIC, ...)
    --no-board           run the bare chip, every bus empty
    --card DIR           insert a card holding a FAT32 volume built from DIR
    --card-image PATH    insert a raw card image, and write changes back
    --hold BUTTON        hold a key down for the whole run
    --touch X,Y          hold a finger on the glass for the whole run
    --capture PATH       write what the panel is showing to a PNG
    --limit N            stop after N instructions
    --break ADDR         stop at an address, registers intact
    --partitions PATH    use a real partition table
    --registers N        list the N most-touched unmodelled registers
    --registers-at ADDR  dump one peripheral block
    --blocks             every peripheral block touched, and every interrupt taken
    --tasks              the firmware's FreeRTOS tasks, and which is running
    --profile N          the N addresses it spent its time in
    --trace-bus          every I2C, SPI and card transaction as it happens
    --seed N             seed the hardware RNG, for a reproducible run
```

### The board an image runs on

The emulator models the chip. A firmware *bundle* reports its own board — its
entry stub hands `BoardConfig::ACTIVE` straight over — but an image is a `.bin`
with no profile in it, so it has to be told which board it is on:

```sh
freeink-simd firmware-x3-x4-v1.5.1.bin --device X3
freeink-emu boards                       # what --device accepts
```

The table comes out of the SDK's own `BoardConfig`, compiled into the daemon,
so a board added to the SDK is one an image can be run on with no change here.
Without `--device` the usual board for the image's chip is assumed — the X4 on
a C3, the X4 Pro on an S3 — and the log says which, because that guess decides
the panel geometry, which pad is D/C and where the buttons are.

That profile is what wires the chip's pins to the rest of the machine: the SPI
byte stream reaches the virtual panel through the pad the board calls D/C, the
I2C addresses the board declares are the ones the virtual devices answer on,
and a `freeink-sim press` lands on the pad — or the ADC ladder — the board says
that key is wired to.

### Holding the power key

These devices are turned on by holding their power key, and their firmware
checks it: released, it configures its peripherals, puts the panel to sleep and
goes back to deep sleep within a second or two. That is not an emulator
artefact, it is what the device does on the bench, so a session starts the same
way:

```sh
freeink-sim hold power
freeink-sim load firmware-x3-x4-v1.5.1.bin
# ... it boots ...
freeink-sim release power
```

`freeink-emu boot ... --hold power` is the same thing for a one-shot run.

### The card

A reader firmware with no card shows an error screen and stops there, so the
virtual card is not a convenience — it is the difference between a boot and a
session. A bundle reaches its files through the host, but an image talks to a
card controller and expects 512-byte sectors, so for an image the card has to
be a card:

```sh
freeink-simd firmware.bin --device X3 --sd ~/books
freeink-sim sd mount ~/books            # or later, over the socket
freeink-emu boot firmware.bin --device X3 --card ~/books
```

The directory is turned into a FAT32 volume — MBR, boot sector, FAT chains,
long file names — and the firmware's own FAT driver mounts and reads it. Both
ways a board can have its card are modelled: SPI, behind its own chip select on
the panel's bus (the X3 and X4), and the chip's native SDMMC host with its
descriptor-driven DMA (the X4 Pro).

A card built from a directory is a **snapshot**. The firmware can write to it —
and must be able to, or it cannot save a reading position — but the host
directory is not modified, so a test run never edits its own fixtures. Pass
`freeink-sim sd image card.img` instead to use a raw image that changes persist
into.

### Sleep

A firmware that puts the chip to sleep really does stop: the core loses power
on the device and comes back through reset, so the machine stops, says it has
slept, and waits. The power key wakes it, as it does on the device, and so does
`freeink-sim wake`; a sleep with a timer wakes when the timer expires. Waking
re-enters the image from its entry point, which is what a deep-sleep wake is.

Wake sources the chip has and the emulator does not model — a touch, a charger
being plugged in — are the reason `wake` exists.

### Status, and what does not work yet

| | State |
|---|---|
| **ESP32-C3 images** (Xteink X3, X4) | Boot to the firmware's own interface. ROM interception, clock and cache setup, the partition table, heap and system initialisation, FreeRTOS, `setup()`, the I2C peripherals, the card mounted over SPI, and the panel painted from the firmware's own driver. The vendor CrossInk image reaches its home screen, browses the card and responds to its keys. |
| **ESP32-S3 images** (X4 Pro, X4 Classic) | The same, on both cores, plus the register window spilling through the firmware's handlers, the APP CPU released and checking in, and the card on the native SDMMC host. The vendor X4 Pro image reaches its home screen with the card mounted and read. Its *input* does not land yet: its menu is driven from the GT911, and that firmware probes the digitizer once and never initialises it, so neither `tap` nor its two hardware keys move the selection. `--tasks` shows why it is worth chasing rather than assuming — every task including the vendor's own `ActivityManager` is up and blocked waiting for exactly that. |
| **PSRAM** | Not modelled. The X4 Pro carries octal PSRAM; its firmware reports the chip missing and carries on without it, which is what that firmware does on a unit that has none. Whatever it would have put there, it does without. |
| **The X4 Pro's fuel gauge** | The CW2017 answers — version, voltage, the soft-reset handshake and the battery profile — but the vendor firmware reads its mode register once and goes no further, so that screen shows 0%. A bundle's `BatteryMonitor` drives the whole sequence and reads a real percentage. |
| **The watchdogs** | Not modelled. Simulated time only advances when the firmware lets it, so a firmware that stops asking for time is reported as waiting rather than shot — but firmware that *relies* on a watchdog reset to recover waits here instead. |
| **Radio** | Wi-Fi and Bluetooth initialisation is let through so firmware reaches the rest of its code, but nothing is transmitted or received. A firmware that waits for a network waits forever. |
| **Fragmented files** | The generated card lays every file out contiguously, so the firmware's FAT driver is exercised on chains, long names and directories but not on fragmentation. |

Everything in the "What it does not simulate" table above applies to emulated
images too.

### Working out where a firmware stopped

An image has no symbols, so "it boots and then just sits there" is otherwise an
opaque answer. These four are the ones that turn it into a cause:

```sh
freeink-emu boot image.bin --tasks     # the FreeRTOS tasks, and which is running
freeink-emu boot image.bin --blocks    # every peripheral block it touched, busiest first
freeink-emu boot image.bin --profile 20  # the addresses it is spinning in
freeink-emu boot image.bin --trace-bus   # every I2C, SPI and card transaction
```

`--tasks` finds task control blocks in RAM by their shape — a pointer to a
stack followed by a name — and reports where each task will resume and what is
still on its stack. It is a heuristic over the firmware's memory, so it is a
diagnostic and not a fact, but it is the difference between "somewhere in
FreeRTOS" and "`loopTask` is blocked and these are the routines it is blocked
in".

`--blocks` lists the interrupts the firmware took as well, with how many times
each was *raised* and how many times it was *taken*. A source raised once and
taken three million times is an interrupt nothing is clearing, which is the
shape of most "it runs but makes no progress" faults.

## Getting started

```sh
# Build the daemon and freeink-emu once. SDL2 is optional — without it you get
# a headless build.
sh tools/simulator/build/build-daemon.sh

# Either: build your own firmware as a bundle and start the simulator on it.
sh tools/simulator/build/build-firmware.sh --device X4CLASSIC src/main.cpp
tools/simulator/build/freeink-simd tools/simulator/build/x4classic.dylib

# Or: start the simulator on a device image, with no source at all. An image
# has no board profile and these devices boot only with their power key held,
# so say which board it is and hold the key.
tools/simulator/build/freeink-simd firmware-x3-x4-v1.5.1.bin --device X3 --sd ~/books
```

In another shell:

```sh
alias sim=tools/simulator/cli/freeink-sim

sim hold power                # a device image: it boots only while this is held
sim status
sim release power
sim press confirm
sim capture screen.png
sim log --follow
```

There is a demo firmware at `tools/simulator/examples/demo/main.cpp` if you want
to see it work before pointing it at your own project.

## The firmware bundle

A bundle is a shared library holding your firmware, the FreeInk SDK, and the
host platform shims in `tools/simulator/platform/`. It resolves its `fsim_*`
symbols from the daemon at `dlopen()` time, so it is not standalone.

```sh
sh tools/simulator/build/build-firmware.sh \
    --device X4CLASSIC \
    --cap FRONTLIGHT \
    --define MY_FIRMWARE_FLAG=1 \
    --out build/fw.dylib \
    src/main.cpp src/screens.cpp
```

`--device` is required and takes the same names as
`-DFREEINK_DEVICE_<NAME>`: `X3`, `X4`, `X4CLASSIC`, `X4PRO`, and so on. It
selects the board profile; the SDK has no default.

Your firmware keeps its ordinary Arduino entry points — `setup()` and `loop()`.
The bundle's entry stub calls them, after reporting `BoardConfig::ACTIVE` to the
daemon so the virtual devices are wired to the same pins the SDK will drive.
**That is the only place board facts are translated for the model**, which is
why adding a board to the SDK makes it simulatable without touching the daemon.

Reload without restarting the daemon:

```sh
sim load build/fw.dylib        # rebuilds are picked up by loading again
sim run --device X4CLASSIC src/main.cpp   # build + load + follow the log
```

## Driving it

### The screen

```sh
sim capture screen.png                       # whatever is on the panel now
sim capture screen.png --wait-refresh        # wait for the next refresh first
sim capture > screen.png                     # PNG on stdout, for piping
sim wait-refresh --timeout 5000              # block until the panel repaints
```

`--wait-refresh` is what makes screenshots race-free: it returns the frame the
firmware actually finished painting, rather than catching the panel mid-update.

### Input

```sh
sim press confirm                # one press
sim press left right up          # a sequence
sim hold power                   # hold down
sim release power
sim press confirm --hold 2000    # a long press, for hold-to-act handling
sim tap --x 400 --y 240          # touch, on boards that have a digitizer
```

`press` holds the button down until the firmware has actually sampled it, then
confirms the release edge was sampled too. This matters: a 60 ms tap that lands
during a 1.6 s blocking refresh is genuinely missed on hardware, and scripted
input would lose events at random without it. Pass `--hold` to force a fixed
duration instead when that is what you are testing.

Touch on a board with no digitizer is **refused, not synthesized** — a test
should not be able to pass against input the hardware cannot produce.

### Waiting for the firmware

```sh
sim expect "book loaded"            # block until a log line matches
sim expect "page \d+ rendered" --timeout 10000
sim log --follow
sim log --channel sim               # the simulator's own notes
```

`expect` is the primitive an agent loop needs: do something, then block until
the firmware says it happened. It searches the whole log by default, since the
action often completes before `expect` runs.

### Time

```sh
sim pause                 # freeze the machine, including firmware tasks
sim step 250              # advance 250 ms and stop again
sim resume
sim clock --rate 4        # run realtime four times as fast
```

The daemon has two clock modes:

- **realtime** (default) — simulated time tracks wall time. What you want when
  watching the window; a refresh takes as long as it takes.
- **virtual** (`--clock virtual`) — time advances only when every firmware
  thread is waiting, then jumps straight to the earliest deadline. A firmware
  that sleeps 30 seconds between refreshes costs no wall time, and a run is
  reproducible. **Use this in tests and agent loops**; it turns a five-second
  boot into about forty milliseconds.

Pausing freezes firmware tasks too, not just the main loop — a background task
that kept running would change the screen under a capture.

### Peripherals and state

```sh
sim sd mount ./cards/default --capacity-mb 8192   # a host directory as the card
sim sd eject
sim nvs                              # stored settings
sim nvs set app theme dark
sim battery --percent 8 --charging false
sim usb plug
sim light                            # frontlight / LED / buzzer PWM state
sim imu --x 0 --y 1 --z 0
sim wifi add HomeNet --password hunter2
sim wifi enable-internet true        # host TCP is OFF unless you say so
sim audio save captured.wav
sim gpio 7 --drive 0                 # drive any pin directly
```

Networking is disabled by default: a simulated device should not reach the
internet just because the firmware asked.

### Debugging the display

When the screen is blank or wrong, the decoded bus trace usually says why:

```sh
$ sim bus --limit 12
DATA_ENTRY_MODE       0x11 len=1      01
SET_RAM_X_RANGE       0x44 len=4      00 00 63 00
SET_RAM_Y_RANGE       0x45 len=4      DF 01 00 00
WRITE_RAM_BW          0x24 len=48000
DISPLAY_UPDATE_CTRL2  0x22 len=1      F7
MASTER_ACTIVATION     0x20 len=1      F7
```

That turns "nothing appeared" into "the driver never sent 0x22/0x20", or "the Y
range is wrong", without a logic analyser.

## The window

Built with SDL2 present, the daemon opens a window showing the panel, and the
socket keeps working at the same time — you can drive it by hand and by script
together.

| Key | Action |
|---|---|
| Arrows | Left / Right / Up / Down |
| Enter, Space | Confirm |
| Backspace, Esc | Back |
| `P` | Power |
| Mouse | Touch, on boards with a digitizer |
| F5 | Restart the firmware |
| F6 | Pause / resume |

Run with `--headless` for CI and agent loops, or build with `FSIM_NO_SDL=1` to
drop the dependency entirely. If a window cannot be opened the daemon says so
and continues headless rather than exiting.

## Using it from a script or an agent

Every command prints a JSON reply and exits nonzero on failure, so `set -e`
works and output is parseable:

```sh
#!/bin/sh
set -e
sim="tools/simulator/cli/freeink-sim --socket $SOCKET"

$sim load build/fw.dylib
$sim expect "boot complete" --timeout 20000
$sim press down
$sim press confirm
$sim expect "book opened"
$sim capture --wait-refresh artifacts/reader.png
```

Start the daemon with `--clock virtual --headless --seed 1` for a run that is
fast, reproducible and needs no display. `--paused` starts before `setup()` so
you can set battery level, NVS keys or a mounted card first.

`tools/simulator/test/run.sh` is a worked example: it builds both Xteink device
families, drives them, and asserts on captures, bus traces and log output.

## Running the tests

```sh
sh tools/simulator/test/run.sh           # the host-bundle path
sh tools/simulator/test/run-emulator.sh  # the device-image path
```

The first builds the daemon and the demo firmware for three boards — one per
shape the SDK has to get right — then checks board detection, first paint,
BUSY-edge refresh timing, capture geometry, I2C device presence, bus decoding,
panel orientation, virtual-clock speed and firmware restart across all of them,
plus what is particular to each: the X4 Classic's seven discrete keys, the X3's
ADC resistor ladder, and the X4 Pro's digitizer and two-channel frontlight. A
board with no digitizer must refuse a tap and a board with no I2C frontlight
must not answer as one, which is the same assertion from both sides.

The second checks the emulator: image parsing and refusal of non-images,
instruction execution, the peripheral write path, breakpoints, flash MMU
translation, and the daemon integration — once per architecture. Its fixtures
are two tiny images assembled by `test/make-test-image.py`, the same program in
RISC-V and in Xtensa, so the suite owns its own input and needs no vendor
firmware. The Xtensa one writes its last characters from a windowed subroutine,
which is the part with no RISC-V equivalent: if CALL8, ENTRY and RETW do not
agree about where the argument went, the fixture prints the wrong bytes.

Point `FSIM_TEST_IMAGE` at a real image to add a boot check against it:

```sh
FSIM_TEST_IMAGE=firmware-x3-x4-v1.5.1.bin sh tools/simulator/test/run-emulator.sh
FSIM_TEST_IMAGE=firmware-x4-pro-v1.5.1.bin sh tools/simulator/test/run-emulator.sh
```

Neither needs a device.

## Adding a chip

Nothing outside `emu/Soc.cpp` names a chip either. A descriptor gives its
memory map, its SRAM aliasing, its MMU encoding, its peripheral base addresses
and the handful of facts that are none of those — where the system timer and
the "interrupt from CPU" sources sit in its interrupt matrix, where the second
core's half of that matrix starts, which register says the cache is idle, which
one says a PLL has finished calibrating. All of them are published in ESP-IDF's
`soc` headers, and each field names the macro it mirrors so the two can be
compared when a new IDF release moves something.

Those last few are in the descriptor because they are the ones that hang a boot
rather than break it: firmware polls them with interrupts off, so a zero where
the silicon says "ready" is a machine that stops with nothing to say.
`freeink-sim registers` exists for that moment — the register a stuck firmware
is spinning on sorts to the top of it.

A part whose core is already implemented needs nothing else; a new instruction
set needs a `Cpu` subclass beside `RiscvCore` and `XtensaCore`.

## Adding a board

Nothing in the daemon names a board. The bundle reports `BoardConfig::ACTIVE` at
startup — pins, input style, touch controller and addresses, gauge, RTC and IMU
addresses — and the daemon builds its virtual devices from that. A new board
profile in the SDK is simulatable as soon as its device flag builds.

Two things do need a look for a genuinely new panel:

1. **The controller.** If it is not SSD1677 or UC81xx, add a `PanelController`
   subclass in `tools/simulator/core/Panel.cpp` decoding its command set.
2. **`epd_gates_reversed`** in `tools/simulator/boards/BoardDesc.h`, which
   records whether the glass scans its gates in reverse of RAM row order. It is
   a property of the panel that each affected driver compensates for, and it
   varies within a controller family — the X3's UC8279d reverses, the X4's does
   not. If it is wrong the symptom is unmistakable: captures come out
   vertically flipped.

For a *device image* there is one more step, because an image cannot report its
own board: add the profile to the table in `tools/simulator/boards/BoardTable.cpp`
under the name its `-DFREEINK_DEVICE_<NAME>` uses, and `--device` accepts it.
That table and the bundle's entry stub share one translation — `BoardDesc.h` —
so the two can never describe the same board differently.

## Layout

```
tools/simulator/
  include/freeink_sim_abi.h   the daemon↔bundle contract
  platform/                   host shims: Arduino, SPI, Wire, FreeRTOS, ESP-IDF,
                              SdFat, Preferences, WiFi, USB — compiled into bundles
  boards/                     BoardConfig -> the daemon's view of a board, shared
                              by the bundle stub and the device-image table
  core/                       the machine: clock, GPIO, ADC, panel, I2C devices,
                              the virtual card and its two controllers
  emu/                        the SoC emulator, for device images:
    Soc.{h,cpp}                 what a chip is — one descriptor per part
    Image.{h,cpp}               image parsing, and the flash built around it
    Bus.{h,cpp}                 the address space and the flash MMU
    RiscvCore.{h,cpp}           RV32IMC, as fitted to the ESP32-C3
    XtensaCore.{h,cpp}          Xtensa LX7, as fitted to the ESP32-S3 —
                                  register windows and all
    Rom*.{h,cpp}                mask ROM, emulated at the call
    Peripherals.{h,cpp}         timers, interrupts, consoles, ADC, RNG
    Board.h                     the seam between the chip and the board
    BoardPeripherals.{h,cpp}    the pin-facing blocks: GPIO, IO_MUX, I2C,
                                  SPI, GDMA, the SDMMC host
    FlashController.{h,cpp}     the SPI memory controller
    EmuMachine.{h,cpp}          one emulated device
    EmuTool.cpp                 freeink-emu
    rom/                        generated ROM symbol maps, and their importer
  daemon/                     freeink-simd: socket server, window, firmware loading
  cli/freeink-sim             the CLI (Python 3 standard library only)
  examples/demo/              a fixture firmware
  test/run.sh                 the host-bundle regression suite
  test/run-emulator.sh        the device-image regression suite
```
