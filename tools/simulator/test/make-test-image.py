#!/usr/bin/env python3
"""Build a tiny ESP32 application image, for testing the emulator itself.

The regression suite needs an image it owns: small, deterministic, and legal to
keep in the repository. This assembles one by hand — a few instructions that
write a string to the UART and stop — which is enough to exercise the whole
path the emulator cares about: the image header and segment table, the
application descriptor, loading a segment into IRAM, mapping a segment into the
cached data window through the flash MMU, instruction decode, a memory-mapped
peripheral write, and a clean halt.

One per architecture, because that last part is the point: the same fixture in
RISC-V and in Xtensa is what says whether a change broke one core or both.

    ./make-test-image.py hello-esp32c3.bin
    ./make-test-image.py --chip esp32s3 hello-esp32s3.bin
"""

import struct
import sys

CHIP_ESP32C3 = 5
CHIP_ESP32S3 = 9

# Where the pieces live. The DROM segment's address and its offset in flash
# must agree in their low 16 bits, because the MMU maps 64 KB pages and can
# only place one at the same offset within a page — the same constraint
# ESP-IDF's linker scripts are written around.
DROM_ADDR = 0x3C000020
UART0_FIFO = 0x60000000
MESSAGE = b"FreeInk emulator fixture\n"

# Each part puts its IRAM somewhere different, and a segment has to land in
# real memory or the loader is right to refuse it.
IRAM_ADDR = {"esp32c3": 0x40380000, "esp32s3": 0x40374000}
CHIP_ID = {"esp32c3": CHIP_ESP32C3, "esp32s3": CHIP_ESP32S3}


def lui(rd, imm20):
    return (imm20 << 12) | (rd << 7) | 0x37


def addi(rd, rs1, imm):
    return ((imm & 0xFFF) << 20) | (rs1 << 15) | (rd << 7) | 0x13


def sw(rs2, rs1, imm):
    return (((imm >> 5) & 0x7F) << 25) | (rs2 << 20) | (rs1 << 15) | (0b010 << 12) | \
           ((imm & 0x1F) << 7) | 0x23


EBREAK = 0x00100073

A0, A1 = 10, 11


def build_code_riscv():
    """Write MESSAGE a byte at a time into the UART transmit FIFO, then stop.

    Deliberately not a loop: a straight-line program makes a failure point at
    one instruction rather than at a branch, and the emulator's own halt on
    `ebreak` with no handler installed is what ends the run."""
    words = [lui(A0, UART0_FIFO >> 12)]
    for byte in MESSAGE:
        words.append(addi(A1, 0, byte))
        words.append(sw(A1, A0, 0))
    words.append(EBREAK)
    return b"".join(struct.pack("<I", word) for word in words)


# ── Xtensa ───────────────────────────────────────────────────────────────────
# Three-byte instructions, little-endian, with the fields at fixed bit
# positions: op2[23:20] op1[19:16] r[15:12] s[11:8] t[7:4] op0[3:0].


def xtensa(op0, t=0, s=0, r=0, op1=0, op2=0):
    word = op0 | (t << 4) | (s << 8) | (r << 12) | (op1 << 16) | (op2 << 20)
    return struct.pack("<I", word)[:3]


def xt_movi(at, imm12):
    """movi at, imm12 — the immediate is split across two fields."""
    return xtensa(0x2, t=at, s=(imm12 >> 8) & 0xF, r=0xA) [:2] + bytes([imm12 & 0xFF])


def xt_s32i(at, as_, offset):
    return xtensa(0x2, t=at, s=as_, r=0x6)[:2] + bytes([offset // 4])


def xt_l32r(at, pc, literal):
    """l32r at, literal — a PC-relative load that only reaches backwards."""
    offset = (literal - ((pc + 3) & ~3)) // 4
    assert offset < 0, "an l32r literal must sit before the instruction"
    word = ((offset & 0xFFFF) << 8) | (at << 4) | 0x1
    return struct.pack("<I", word)[:3]


def xt_break(level=1, code=15):
    return xtensa(0x0, t=code, s=level, r=0x4)


def xt_call8(pc, target):
    """call8 target — the offset is in words from the instruction after it."""
    assert target % 4 == 0, "a call target is four-byte aligned"
    offset = (target - ((pc + 4) & ~3)) // 4
    word = ((offset & 0x3FFFF) << 6) | (2 << 4) | 0x5
    return struct.pack("<I", word)[:3]


def xt_entry(as_, frame):
    word = ((frame // 8) << 12) | (as_ << 8) | (3 << 4) | 0x6
    return struct.pack("<I", word)[:3]


RETW_N = b"\x1d\xf0"


def build_code_xtensa(base):
    """The same program in Xtensa, with the literal pool it needs in front of
    it — there is no instruction that materialises a 32-bit address, so the
    UART address is a word the code loads with `l32r`.

    The last two characters are written by a windowed subroutine rather than
    inline, which is the part a RISC-V fixture cannot cover: CALL8 leaves its
    return address and its argument in the caller's high registers, ENTRY
    rotates the window so the callee finds them as its own a0 and a2, and RETW
    rotates back. Get any of that wrong and the fixture writes the wrong bytes
    or returns into nothing."""
    code = bytearray(struct.pack("<I", UART0_FIFO))
    a2, a3, a10 = 2, 3, 10
    code += xt_l32r(a2, base + len(code), base)
    for byte in MESSAGE[:-3]:
        code += xt_movi(a3, byte)
        code += xt_s32i(a3, a2, 0)

    # The subroutine goes after the caller, four-byte aligned.
    tail = bytearray()
    tail += xt_entry(1, 32)
    for byte in MESSAGE[-3:]:
        tail += xt_movi(a3, byte)
        tail += xt_s32i(a3, 2, 0)   # a2 in the callee's window is the caller's a10
    tail += RETW_N

    body = bytearray()
    body += xtensa(0xD, t=a10, s=a2)[:2]      # mov.n a10, a2 — the argument
    call_at = len(code) + len(body)
    body += b"\x00\x00\x00"                  # room for the call, filled in below
    body += xt_break()

    # The subroutine follows, four-byte aligned because a call target is.
    subroutine = base + len(code) + len(body)
    subroutine += (-subroutine) % 4
    body[call_at - len(code):call_at - len(code) + 3] = xt_call8(base + call_at, subroutine)

    code += body
    code += b"\x00" * ((subroutine - base) - len(code))
    code += tail
    return bytes(code)


def build_app_descriptor():
    """esp_app_desc_t. The emulator uses its magic to tell an application image
    from a bootloader, and reports the rest."""
    desc = bytearray(256)
    struct.pack_into("<I", desc, 0, 0xABCD5432)   # magic
    struct.pack_into("<I", desc, 4, 0)            # secure_version
    desc[16:16 + 7] = b"fixture"                  # version
    desc[48:48 + 12] = b"freeink-test"            # project_name
    desc[80:80 + 8] = b"00:00:00"                 # time
    desc[96:96 + 11] = b"Jan  1 2026"             # date
    desc[112:112 + 5] = b"5.5.2"                  # idf_ver
    return bytes(desc)


def build_image(chip):
    iram = IRAM_ADDR[chip]
    if chip == "esp32s3":
        # The literal pool sits at the start of the segment, so the entry point
        # is the first instruction after it.
        code = build_code_xtensa(iram)
        entry = iram + 4
    else:
        code = build_code_riscv()
        entry = iram
    segments = [(DROM_ADDR, build_app_descriptor()), (iram, code)]

    header = bytearray(24)
    header[0] = 0xE9
    header[1] = len(segments)
    header[2] = 2                      # SPI mode: DIO
    header[3] = (2 << 4) | 0xF         # 4 MB at 80 MHz
    struct.pack_into("<I", header, 4, entry)
    header[8] = 0                      # wp_pin
    struct.pack_into("<H", header, 12, CHIP_ID[chip])
    header[23] = 0                     # no appended hash

    body = bytearray()
    checksum = 0xEF
    for address, data in segments:
        body += struct.pack("<II", address, len(data)) + data
        for byte in data:
            checksum ^= byte

    image = bytes(header) + bytes(body)
    # One checksum byte, at the end of a 16-byte-aligned block.
    padding = 15 - (len(image) % 16)
    image += b"\x00" * padding + bytes([checksum])
    return image


def main():
    arguments = sys.argv[1:]
    chip = "esp32c3"
    if len(arguments) >= 2 and arguments[0] == "--chip":
        chip = arguments[1]
        arguments = arguments[2:]
    if len(arguments) != 1 or chip not in CHIP_ID:
        sys.exit("usage: make-test-image.py [--chip esp32c3|esp32s3] <out.bin>")
    image = build_image(chip)
    with open(arguments[0], "wb") as handle:
        handle.write(image)
    print(f"{arguments[0]}: {len(image)} bytes, {chip}")


if __name__ == "__main__":
    main()
