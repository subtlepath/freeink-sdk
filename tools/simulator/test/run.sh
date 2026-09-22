#!/bin/sh
# FreeInk simulator regression suite.
#
# Builds the daemon and the demo firmware for three Xteink boards — one per
# shape the SDK has to get right: digital buttons, an ADC resistor ladder, and
# a digitizer with a two-channel frontlight — then drives the simulator over
# its control socket and asserts on what comes back.
# Needs no device and no PlatformIO; SDL2 is optional (the suite runs headless).
#
#   sh tools/simulator/test/run.sh

set -e

SIM_DIR=$(cd "$(dirname "$0")/.." && pwd)
SDK_DIR=$(cd "$SIM_DIR/../.." && pwd)
WORK="${TMPDIR:-/tmp}/freeink-sim-tests.$$"
SOCKET="$WORK/sim.sock"
SIM="$SIM_DIR/cli/freeink-sim --socket $SOCKET"
DAEMON="$SIM_DIR/build/freeink-simd"
DAEMON_PID=""

mkdir -p "$WORK/state" "$WORK/card/books"
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
  # start_daemon <bundle> [extra daemon args...]
  bundle=$1
  shift
  cleanup
  DAEMON_PID=""
  rm -f "$SOCKET"
  "$DAEMON" --headless --socket "$SOCKET" --state-dir "$WORK/state" \
    --sd "$WORK/card" "$@" "$bundle" > "$WORK/daemon.log" 2>&1 &
  DAEMON_PID=$!
  # Wait for the socket rather than sleeping a guessed interval.
  waited=0
  while [ ! -S "$SOCKET" ] && [ $waited -lt 100 ]; do
    sleep 0.1
    waited=$((waited + 1))
  done
  [ -S "$SOCKET" ] || { echo "daemon did not start; see $WORK/daemon.log"; exit 1; }
}

echo
echo "── Building ────────────────────────────────────────────────────────────"
sh "$SIM_DIR/build/build-daemon.sh" > "$WORK/build-daemon.log" 2>&1 ||
  { echo "daemon build failed; see $WORK/build-daemon.log"; exit 1; }
echo "  daemon built"

for device in X4CLASSIC X3 X4PRO; do
  sh "$SIM_DIR/build/build-firmware.sh" --device "$device" \
    --out "$WORK/$device.bundle" \
    "$SIM_DIR/examples/demo/main.cpp" > "$WORK/build-$device.log" 2>&1 ||
    { echo "firmware build failed for $device; see $WORK/build-$device.log"; exit 1; }
  echo "  $device firmware built"
done

# ── Xteink X4 Classic: ESP32-S3, SSD1677, 800x480, digital buttons ───────────
echo
echo "── Xteink X4 Classic ───────────────────────────────────────────────────"
start_daemon "$WORK/X4CLASSIC.bundle"

status=$($SIM --json status)
check "reports the compiled board profile" "$status" '"board": "xteink_x4_classic"'
check "selects the SSD1677 controller"     "$status" '"controller": "SSD1677"'
check "reports the panel geometry"         "$status" '"width": 800'
check "firmware is running"                "$status" '"running": true'

# The first paint must land. This also proves the whole chain: the real facade
# and driver ran, EpdBus framed the traffic, the virtual controller decoded it,
# and the waveform completed.
$SIM expect "first paint complete" --timeout 20000 -q
check "firmware completes its first paint" "$($SIM --json status)" '"seq": 1'

# The refresh must take the modelled waveform time, not a timeout. A driver that
# stopped waiting on BUSY, or a model that never released it, shows up here.
check "refresh completes on the BUSY edge" "$($SIM log)" 'Wait complete: refresh (16'

capture=$($SIM --json capture "$WORK/x4c.png")
check "captures a PNG at panel size" "$capture" '"height": 480'
check "capture file exists"          "$(file "$WORK/x4c.png")" 'PNG image data, 800 x 480'

# Input: the real InputManager decodes real pin levels, and a press is held
# until the firmware samples it rather than racing a blocking refresh.
$SIM press confirm -q
$SIM expect "press #1: Confirm" --timeout 20000 -q
check "a button press reaches the firmware" "$($SIM log)" 'press #1: Confirm'

$SIM press left -q
$SIM press right -q
$SIM expect "press #3: Right" --timeout 30000 -q
check "successive presses are not merged" "$($SIM log)" 'press #3: Right'

# The virtual I2C bus is built from the board profile, and an address nothing
# answers must NACK — that is what makes peripheral-absent paths testable.
i2c=$($SIM --json i2c)
check "the board's RTC answers"        "$i2c" '0x51'
check "the board's fuel gauge answers" "$i2c" 'CW2017'
check "the board's IMU answers"        "$i2c" 'QMI8658'

# This board has no digitizer; touch must be refused, not synthesized.
touch_error=$($SIM raw tap --x 100 --y 100 2>&1 || true)
check "touch is refused on a board without one" "$touch_error" 'no touch controller'

# Decoded bus traffic, the view that explains a blank screen.
bus=$($SIM --json bus --limit 200)
check "bus trace decodes the RAM write" "$bus" 'WRITE_RAM_BW'
check "bus trace decodes the refresh"   "$bus" 'MASTER_ACTIVATION'

# Peripherals the CLI drives.
check "battery state is settable" "$($SIM --json battery --percent 42 --charging true)" '"percent": 42'
check "USB presence is settable"  "$($SIM --json usb plug)" '"connected": true'
check "settings survive as NVS"   "$($SIM --json nvs set app theme dark; $SIM --json nvs get app theme)" 'dark'
check "the SD card is mounted"    "$($SIM --json sd)" '"present": true'

# ── Xteink X4 Pro: ESP32-S3, SSD1677, 800x480, GT911 touch + warm frontlight ─
# The Pro is the X4 Classic's sibling with two things the Classic does not have,
# and they are the two the simulator builds from the board profile rather than
# from a chip: a digitizer, and a frontlight with a second warm channel.
echo
echo "── Xteink X4 Pro ───────────────────────────────────────────────────────"
start_daemon "$WORK/X4PRO.bundle"

status=$($SIM --json status)
check "reports the X4 Pro board profile" "$status" '"board": "xteink_x4_pro"'
check "selects the SSD1677 controller"   "$status" '"controller": "SSD1677"'
check "reports the panel geometry"       "$status" '"width": 800'

$SIM expect "first paint complete" --timeout 20000 -q
check "firmware completes its first paint" "$($SIM --json status)" '"seq": 1'

capture=$($SIM --json capture "$WORK/x4pro.png")
check "captures a PNG at panel size" "$capture" '"height": 480'

# This board has a digitizer, so a tap is delivered rather than refused — the
# mirror image of the X4 Classic check above.
tap=$($SIM --json tap --x 400 --y 240)
check "touch is accepted on a board with a digitizer" "$tap" '"x": 400'
check "the touch controller answers on I2C"           "$($SIM --json i2c)" 'GT911'

# The frontlight is two LEDC channels, cool and warm, and both come from the
# profile. A board with no I2C frontlight controller must have nothing
# answering at its address — that is what makes "is one fitted?" testable.
light=$($SIM --json light)
check "the frontlight PWM channel is driven"  "$light" '"pin": 8'
check "the warm channel is driven too"        "$light" '"pin": 9'
i2c=$($SIM --json i2c)
if printf '%s' "$i2c" | grep -q "LM3630A"; then
  echo "  FAIL a board without an I2C frontlight answers as one anyway"
  fail=$((fail + 1))
else
  echo "  ok   a board without an I2C frontlight does not answer as one"
  pass=$((pass + 1))
fi

# ── Xteink X3: ESP32-C3, UC8253, 792x528, ADC resistor ladder ────────────────
echo
echo "── Xteink X3 ───────────────────────────────────────────────────────────"
start_daemon "$WORK/X3.bundle"

status=$($SIM --json status)
check "reports the X3 board profile"  "$status" '"board": "xteink_x3"'
check "selects the UC8253 controller" "$status" '"controller": "UC8253"'
check "reports the X3 geometry"       "$status" '"width": 792'

$SIM expect "first paint complete" --timeout 40000 -q
capture=$($SIM --json capture "$WORK/x3.png")
check "captures at the X3 panel size" "$capture" '"height": 528'

# The X3 has no digital button GPIOs: every nav key is a voltage on one of two
# ADC ladder pins, decoded by the real InputManager tables.
$SIM press confirm --timeout 40000 -q
$SIM expect "press #1: Confirm" --timeout 40000 -q
check "the ADC resistor ladder decodes a press" "$($SIM log)" 'press #1: Confirm'

bus=$($SIM --json bus --limit 200)
check "UC81xx plane stream is decoded" "$bus" 'DTM2'
check "UC81xx refresh is decoded"      "$bus" 'DRF'

# ── Orientation ─────────────────────────────────────────────────────────────
# Both panels scan their gates in reverse and their drivers compensate. If the
# model's reversal is wrong the image is upside down, which no other assertion
# would catch — so check a pixel that only sits where it does when the frame is
# the right way up. The demo paints a 48px black header across the top.
echo
echo "── Orientation ─────────────────────────────────────────────────────────"
for shot in "$WORK/x4c.png" "$WORK/x3.png"; do
  verdict=$(python3 - "$shot" <<'PY'
import struct, sys, zlib

path = sys.argv[1]
data = open(path, "rb").read()
pos, width, height, raw = 8, 0, 0, b""
while pos < len(data):
    length, kind = struct.unpack(">I4s", data[pos:pos + 8])
    body = data[pos + 8:pos + 8 + length]
    if kind == b"IHDR":
        width, height = struct.unpack(">II", body[:8])
    elif kind == b"IDAT":
        raw += body
    pos += 12 + length

pixels = zlib.decompress(raw)
stride = width + 1

def row_is_dark(y):
    start = y * stride + 1
    row = pixels[start:start + width]
    return sum(row) / len(row) < 64

# Top rows inside the header must be black; the band just below it must be white.
print("upright" if row_is_dark(8) and not row_is_dark(70) else "flipped")
PY
)
  check "$(basename "$shot") is the right way up" "$verdict" "upright"
done

# ── Virtual clock ───────────────────────────────────────────────────────────
# Virtual time advances only when every firmware thread is waiting, and then
# jumps to the earliest deadline. A boot that takes seconds of simulated time
# must therefore cost almost no wall time — that is what makes the simulator
# usable inside a test loop.
echo
echo "── Virtual clock ───────────────────────────────────────────────────────"
start_daemon "$WORK/X4CLASSIC.bundle" --clock virtual

started=$(date +%s)
$SIM expect "first paint complete" --timeout 30000 -q
elapsed=$(( $(date +%s) - started ))
if [ "$elapsed" -le 3 ]; then
  check "virtual time makes a boot near-instant" "fast" "fast"
else
  check "virtual time makes a boot near-instant" "slow (${elapsed}s of wall time)" "fast"
fi

simulated=$($SIM --json status | python3 -c "import json,sys; print(json.load(sys.stdin)['clock']['us'])")
if [ "$simulated" -gt 3000000 ]; then
  check "the simulated clock ran past wall time" "ahead" "ahead"
else
  check "the simulated clock ran past wall time" "only ${simulated}us elapsed" "ahead"
fi

# ── Reload and reboot ───────────────────────────────────────────────────────
echo
echo "── Lifecycle ───────────────────────────────────────────────────────────"
start_daemon "$WORK/X4CLASSIC.bundle"
$SIM expect "first paint complete" --timeout 20000 -q
$SIM log-clear -q > /dev/null
$SIM reset -q > /dev/null
$SIM expect "booting on" --timeout 20000 -q
check "reset re-runs setup() on a fresh bundle" "$($SIM log)" 'booting on'

echo
echo "────────────────────────────────────────────────────────────────────────"
echo "$pass passed, $fail failed"
[ "$fail" -eq 0 ] || exit 1
echo "All simulator checks passed."
