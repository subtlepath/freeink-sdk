#!/bin/sh
# FreeInk emulator regression suite.
#
# Covers the path that runs a device-ready firmware image rather than a host
# build: the image loader, the flash MMU, both cores, the peripheral models and
# the daemon integration. One fixture image per architecture is assembled by
# test/make-test-image.py — RISC-V for the X3 and X4, Xtensa for the X4 Pro and
# X4 Classic — so the suite owns its own input and needs no vendor firmware.
# Point FSIM_TEST_IMAGE at a real one and it runs an extra set of checks
# against that too.
#
#   sh tools/simulator/test/run-emulator.sh
#   FSIM_TEST_IMAGE=firmware-x3-x4-v1.5.1.bin sh tools/simulator/test/run-emulator.sh
#   FSIM_TEST_IMAGE=firmware-x4-pro-v1.5.1.bin sh tools/simulator/test/run-emulator.sh

set -e

SIM_DIR=$(cd "$(dirname "$0")/.." && pwd)
WORK="${TMPDIR:-/tmp}/freeink-emu-tests.$$"
SOCKET="$WORK/sim.sock"
SIM="$SIM_DIR/cli/freeink-sim --socket $SOCKET"
DAEMON="$SIM_DIR/build/freeink-simd"
EMU="$SIM_DIR/build/freeink-emu"
IMAGE="$WORK/fixture.bin"
XTENSA_IMAGE="$WORK/fixture-xtensa.bin"
DAEMON_PID=""

mkdir -p "$WORK/state"
echo "work dir: $WORK"

pass=0
fail=0

cleanup() {
  [ -n "$DAEMON_PID" ] && kill "$DAEMON_PID" 2>/dev/null || true
  sleep 1
  [ -n "$DAEMON_PID" ] && kill -9 "$DAEMON_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

check() {
  # check <description> <actual> <expected-substring>
  if printf '%s' "$2" | grep -q -- "$3"; then
    echo "  ok   $1"
    pass=$((pass + 1))
  else
    echo "  FAIL $1"
    echo "         expected to contain: $3"
    echo "         got: $2"
    fail=$((fail + 1))
  fi
}

start_daemon() {
  cleanup
  DAEMON_PID=""
  rm -f "$SOCKET"
  "$DAEMON" --headless --socket "$SOCKET" --state-dir "$WORK/state" "$@" \
    > "$WORK/daemon.log" 2>&1 &
  DAEMON_PID=$!
  waited=0
  while [ ! -S "$SOCKET" ] && [ $waited -lt 100 ]; do
    sleep 0.1
    waited=$((waited + 1))
  done
  [ -S "$SOCKET" ] || { echo "daemon did not start; see $WORK/daemon.log"; exit 1; }
}

echo
echo "── Building ────────────────────────────────────────────────────────────"
sh "$SIM_DIR/build/build-daemon.sh" > "$WORK/build.log" 2>&1 ||
  { echo "build failed; see $WORK/build.log"; exit 1; }
echo "  daemon and freeink-emu built"
python3 "$SIM_DIR/test/make-test-image.py" "$IMAGE" > /dev/null
python3 "$SIM_DIR/test/make-test-image.py" --chip esp32s3 "$XTENSA_IMAGE" > /dev/null
echo "  fixture images assembled (RISC-V and Xtensa)"

# ── The image loader ────────────────────────────────────────────────────────
# An image describes its own silicon; nothing here is inferred from the file
# name or guessed from the contents.
echo
echo "── Image loading ───────────────────────────────────────────────────────"
info=$("$EMU" info "$IMAGE")
check "identifies the chip from the image header" "$info" "esp32c3"
check "identifies the architecture"               "$info" "RISC-V"
check "reads the application descriptor"          "$info" "freeink-test"
check "reports the entry point"                   "$info" "0x40380000"
check "separates RAM from flash-mapped segments"  "$info" "flash-mapped DROM"
check "synthesizes a partition table"             "$info" "app/factory"

bad=$("$EMU" info "$SIM_DIR/test/run-emulator.sh" 2>&1 || true)
check "refuses a file that is not an image"       "$bad" "not an ESP32 image"

# ── Executing ───────────────────────────────────────────────────────────────
# The fixture writes a string to the UART transmit FIFO one byte at a time and
# stops on a breakpoint. Getting the string out proves the whole chain: segment
# loading, instruction decode, the peripheral model and the console.
echo
echo "── Execution ───────────────────────────────────────────────────────────"
run=$("$EMU" boot "$IMAGE" --limit 100000 2>&1)
check "runs the firmware's instructions"     "$run" "FreeInk emulator fixture"
check "stops where the firmware asked it to" "$run" "ebreak with no handler"
check "accounts for what it executed"        "$run" "instructions"

"$EMU" boot "$IMAGE" --limit 100000 --quiet > /dev/null
check "a deliberate halt is not a failure" "exit $?" "exit 0"

# A breakpoint stops with the registers intact — the debugging primitive an
# image without symbols otherwise lacks.
stopped=$("$EMU" boot "$IMAGE" --limit 100000 --quiet --break 0x40380004 2>&1)
check "stops at a breakpoint"          "$stopped" "breakpoint at 0x40380004"
check "reports registers at the stop"  "$stopped" "a0=0x60000000"

# ── The flash MMU ───────────────────────────────────────────────────────────
# The cached data window is not a copy of the image: it translates through the
# chip's page table into the flash model, which is what `mem` proves by reading
# the application descriptor back through it.
echo
echo "── Flash MMU ───────────────────────────────────────────────────────────"
start_daemon
$SIM load "$IMAGE" -q > /dev/null
sleep 1
mem=$($SIM mem 0x3C000020 16)
check "maps the cached data window to flash" "$mem" "MMU page 0 -> flash 0x010020"
check "reads the descriptor through the MMU" "$mem" "32 54 CD AB"

unmapped=$($SIM mem 0x10000000 16 2>&1 || true)
check "an unmapped address is reported, not faked" "$unmapped" "nothing is mapped"

# ── Daemon integration ──────────────────────────────────────────────────────
# Loading an image goes through the same command as loading a host bundle, and
# the machine reports which kind it is running.
echo
echo "── Daemon integration ──────────────────────────────────────────────────"
status=$($SIM --json status)
check "status reports the emulated chip" "$status" '"chip": "esp32c3"'
check "status counts executed instructions" "$status" '"instructions"'

emu=$($SIM --json emu)
check "emu reports the image's project"  "$emu" '"project": "freeink-test"'
check "emu reports the instruction set"  "$emu" '"isa": "rv32imc"'
check "emu flags a synthesized table"    "$emu" '"partitions_synthesized": true'

check "firmware output reaches the log" "$($SIM log)" "FreeInk emulator fixture"

# Loading an image forces virtual time: emulation cannot keep pace with wall
# time, and a clock that ran ahead of the firmware would make every piece of
# modelled timing wrong.
check "switches the clock to virtual time" "$($SIM --json status)" '"mode": "virtual"'
check "and says why"                       "$($SIM log --channel sim)" "clock switched to virtual"

# ── The Xtensa core ─────────────────────────────────────────────────────────
# The X4 Pro and the X4 Classic are ESP32-S3, so their images are Xtensa from
# the first instruction. The same fixture program, in the other instruction
# set, exercises the loader, the decoder and the console over again — and its
# last three characters are written by a windowed subroutine, which is the part
# with no RISC-V equivalent: CALL8 leaves the return address and the argument
# in the caller's high registers, ENTRY rotates the window so the callee reads
# them as its own, and RETW rotates back.
echo
echo "── Xtensa core ─────────────────────────────────────────────────────────"
info=$("$EMU" info "$XTENSA_IMAGE")
check "identifies an Xtensa part"      "$info" "esp32s3"
check "reports the architecture"       "$info" "Xtensa, 2 cores"
check "reads its application descriptor" "$info" "freeink-test"

run=$("$EMU" boot "$XTENSA_IMAGE" --limit 100000 2>&1)
check "executes Xtensa instructions"   "$run" "FreeInk emulator fixture"
check "a windowed call returns"        "$run" "break 1, 15"
check "reports the whole window"       "$run" "a8-a15"

"$EMU" boot "$XTENSA_IMAGE" --limit 100000 --quiet > /dev/null
check "a deliberate halt is not a failure" "exit $?" "exit 0"

start_daemon
$SIM load "$XTENSA_IMAGE" -q > /dev/null
sleep 1
check "the daemon runs an Xtensa image" "$($SIM --json emu)" '"isa": "xtensa lx7"'
check "and its output reaches the log"  "$($SIM log)" "FreeInk emulator fixture"

# ── The board table ─────────────────────────────────────────────────────────
# An image carries no board profile, so the emulator is told which board it is
# on out of the SDK's own BoardConfig. If that table ever stops being generated
# the symptom is a device image with no panel and no buttons, which is a much
# harder thing to recognise than a missing name here.
echo
echo "── Boards ──────────────────────────────────────────────────────────────"
boards=$("$EMU" boards 2>&1)
check "lists the X3"                "$boards" "X3 "
check "lists the X4 Pro"            "$boards" "X4PRO"
check "lists the X4 Classic"        "$boards" "X4CLASSIC"
check "reports the X3 geometry"     "$boards" "792x528"
check "refuses a board it has none of" \
      "$("$EMU" boot "$IMAGE" --device NOSUCHBOARD 2>&1 || true)" "no board profile named"

# ── The virtual SD card ─────────────────────────────────────────────────────
# A reader firmware with no card shows an error screen and goes no further, so
# the card is not a convenience: it is the difference between a boot and a
# session. The volume is built from a host directory and read back through the
# emulated card controller, so what is asserted here is that the firmware's own
# FAT driver can mount what the builder produced.
echo
echo "── Virtual SD card ─────────────────────────────────────────────────────"
mkdir -p "$WORK/card/books"
printf 'Chapter One\n' > "$WORK/card/books/A Long Book Name.txt"
printf 'x' > "$WORK/card/README.TXT"
carded=$("$EMU" boot "$IMAGE" --card "$WORK/card" --limit 1000 2>&1 || true)
check "builds a volume from a directory" "$carded" "card:"
check "and reports its size"             "$carded" "MB from"
check "rejects a card that is not there" \
      "$("$EMU" boot "$IMAGE" --card "$WORK/no-such-directory" 2>&1 || true)" "cannot read"

# ── A real device image, when one is available ──────────────────────────────
if [ -n "$FSIM_TEST_IMAGE" ] && [ -f "$FSIM_TEST_IMAGE" ]; then
  echo
  echo "── Vendor image: $FSIM_TEST_IMAGE ──────────────────────────────"
  real=$("$EMU" boot "$FSIM_TEST_IMAGE" --limit 400000000 2>&1 || true)
  check "boots past the ROM and into ESP-IDF" "$real" "instructions"
  check "the scheduler takes interrupts"      "$real" "interrupts"
  # An Xtensa image spills its register window thousands of times on the way
  # up, through the firmware's own handlers. A boot with no exceptions at all
  # is one that never got as far as a deep call chain.
  if "$EMU" info "$FSIM_TEST_IMAGE" | grep -q Xtensa; then
    check "the firmware's window handlers run" "$real" "exceptions"
  fi
  echo "$real" | head -20 > "$WORK/vendor-boot.log"
  echo "  (boot log: $WORK/vendor-boot.log)"

  # The whole point of the exercise: on its own board, with a card in it and
  # its power key held as a person would hold it, the firmware should get past
  # setup() and paint its interface. `--hold power` is not a workaround — these
  # devices are turned on by holding that key, and a firmware that finds it
  # released goes straight back to sleep, here as on the bench.
  if "$EMU" info "$FSIM_TEST_IMAGE" | grep -q Xtensa; then
    device=X4PRO
  else
    device=X3
  fi
  echo
  echo "── $device session: $FSIM_TEST_IMAGE ────────────────────────────"
  # The bus trace of a whole session is megabytes; it goes to a file and the
  # checks read summaries out of it, rather than through a pipe that grep
  # closes halfway.
  "$EMU" boot "$FSIM_TEST_IMAGE" --device "$device" --card "$WORK/card" \
      --hold power --limit 1200000000 --quiet --trace-bus \
      --capture "$WORK/session.png" > "$WORK/session.log" 2>&1 || true
  check "reads the card"                "$(grep -c -E '^\[bus\] (sd|sdmmc)' "$WORK/session.log")x" "^[1-9]"
  check "drives the panel over the bus" "$(grep -c '^\[bus\] spi2' "$WORK/session.log")x" "^[1-9]"
  check "paints a frame"                "$(grep captured "$WORK/session.log" || true)" "captured"
  check "the frame is the panel's size" "$(grep captured "$WORK/session.log" || true)" \
        "$("$EMU" boards | awk -v d="$device" '$1 == d {print $3}')"
  echo "  (screen: $WORK/session.png, bus trace: $WORK/session.log)"
fi

echo
echo "────────────────────────────────────────────────────────────────────────"
echo "$pass passed, $fail failed"
[ "$fail" -eq 0 ] || exit 1
echo "All emulator checks passed."
