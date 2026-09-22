// FreeInk emulator — MD5.

#include "Md5.h"

#include <cstring>

namespace freeink::sim::emu {

void md5Transform(uint32_t buf[4], const uint32_t in[16]) {
  auto f1 = [](uint32_t x, uint32_t y, uint32_t z) { return z ^ (x & (y ^ z)); };
  auto f2 = [](uint32_t x, uint32_t y, uint32_t z) { return (x & z) | (y & ~z); };
  auto f3 = [](uint32_t x, uint32_t y, uint32_t z) { return x ^ y ^ z; };
  auto f4 = [](uint32_t x, uint32_t y, uint32_t z) { return y ^ (x | ~z); };
  auto rot = [](uint32_t x, int c) { return (x << c) | (x >> (32 - c)); };

  uint32_t a = buf[0], b = buf[1], c = buf[2], d = buf[3];
#define MD5STEP(f, w, x, y, z, data, s) (w += f(x, y, z) + (data), w = rot(w, s), w += x)
  MD5STEP(f1, a, b, c, d, in[0] + 0xd76aa478, 7);
  MD5STEP(f1, d, a, b, c, in[1] + 0xe8c7b756, 12);
  MD5STEP(f1, c, d, a, b, in[2] + 0x242070db, 17);
  MD5STEP(f1, b, c, d, a, in[3] + 0xc1bdceee, 22);
  MD5STEP(f1, a, b, c, d, in[4] + 0xf57c0faf, 7);
  MD5STEP(f1, d, a, b, c, in[5] + 0x4787c62a, 12);
  MD5STEP(f1, c, d, a, b, in[6] + 0xa8304613, 17);
  MD5STEP(f1, b, c, d, a, in[7] + 0xfd469501, 22);
  MD5STEP(f1, a, b, c, d, in[8] + 0x698098d8, 7);
  MD5STEP(f1, d, a, b, c, in[9] + 0x8b44f7af, 12);
  MD5STEP(f1, c, d, a, b, in[10] + 0xffff5bb1, 17);
  MD5STEP(f1, b, c, d, a, in[11] + 0x895cd7be, 22);
  MD5STEP(f1, a, b, c, d, in[12] + 0x6b901122, 7);
  MD5STEP(f1, d, a, b, c, in[13] + 0xfd987193, 12);
  MD5STEP(f1, c, d, a, b, in[14] + 0xa679438e, 17);
  MD5STEP(f1, b, c, d, a, in[15] + 0x49b40821, 22);

  MD5STEP(f2, a, b, c, d, in[1] + 0xf61e2562, 5);
  MD5STEP(f2, d, a, b, c, in[6] + 0xc040b340, 9);
  MD5STEP(f2, c, d, a, b, in[11] + 0x265e5a51, 14);
  MD5STEP(f2, b, c, d, a, in[0] + 0xe9b6c7aa, 20);
  MD5STEP(f2, a, b, c, d, in[5] + 0xd62f105d, 5);
  MD5STEP(f2, d, a, b, c, in[10] + 0x02441453, 9);
  MD5STEP(f2, c, d, a, b, in[15] + 0xd8a1e681, 14);
  MD5STEP(f2, b, c, d, a, in[4] + 0xe7d3fbc8, 20);
  MD5STEP(f2, a, b, c, d, in[9] + 0x21e1cde6, 5);
  MD5STEP(f2, d, a, b, c, in[14] + 0xc33707d6, 9);
  MD5STEP(f2, c, d, a, b, in[3] + 0xf4d50d87, 14);
  MD5STEP(f2, b, c, d, a, in[8] + 0x455a14ed, 20);
  MD5STEP(f2, a, b, c, d, in[13] + 0xa9e3e905, 5);
  MD5STEP(f2, d, a, b, c, in[2] + 0xfcefa3f8, 9);
  MD5STEP(f2, c, d, a, b, in[7] + 0x676f02d9, 14);
  MD5STEP(f2, b, c, d, a, in[12] + 0x8d2a4c8a, 20);

  MD5STEP(f3, a, b, c, d, in[5] + 0xfffa3942, 4);
  MD5STEP(f3, d, a, b, c, in[8] + 0x8771f681, 11);
  MD5STEP(f3, c, d, a, b, in[11] + 0x6d9d6122, 16);
  MD5STEP(f3, b, c, d, a, in[14] + 0xfde5380c, 23);
  MD5STEP(f3, a, b, c, d, in[1] + 0xa4beea44, 4);
  MD5STEP(f3, d, a, b, c, in[4] + 0x4bdecfa9, 11);
  MD5STEP(f3, c, d, a, b, in[7] + 0xf6bb4b60, 16);
  MD5STEP(f3, b, c, d, a, in[10] + 0xbebfbc70, 23);
  MD5STEP(f3, a, b, c, d, in[13] + 0x289b7ec6, 4);
  MD5STEP(f3, d, a, b, c, in[0] + 0xeaa127fa, 11);
  MD5STEP(f3, c, d, a, b, in[3] + 0xd4ef3085, 16);
  MD5STEP(f3, b, c, d, a, in[6] + 0x04881d05, 23);
  MD5STEP(f3, a, b, c, d, in[9] + 0xd9d4d039, 4);
  MD5STEP(f3, d, a, b, c, in[12] + 0xe6db99e5, 11);
  MD5STEP(f3, c, d, a, b, in[15] + 0x1fa27cf8, 16);
  MD5STEP(f3, b, c, d, a, in[2] + 0xc4ac5665, 23);

  MD5STEP(f4, a, b, c, d, in[0] + 0xf4292244, 6);
  MD5STEP(f4, d, a, b, c, in[7] + 0x432aff97, 10);
  MD5STEP(f4, c, d, a, b, in[14] + 0xab9423a7, 15);
  MD5STEP(f4, b, c, d, a, in[5] + 0xfc93a039, 21);
  MD5STEP(f4, a, b, c, d, in[12] + 0x655b59c3, 6);
  MD5STEP(f4, d, a, b, c, in[3] + 0x8f0ccc92, 10);
  MD5STEP(f4, c, d, a, b, in[10] + 0xffeff47d, 15);
  MD5STEP(f4, b, c, d, a, in[1] + 0x85845dd1, 21);
  MD5STEP(f4, a, b, c, d, in[8] + 0x6fa87e4f, 6);
  MD5STEP(f4, d, a, b, c, in[15] + 0xfe2ce6e0, 10);
  MD5STEP(f4, c, d, a, b, in[6] + 0xa3014314, 15);
  MD5STEP(f4, b, c, d, a, in[13] + 0x4e0811a1, 21);
  MD5STEP(f4, a, b, c, d, in[4] + 0xf7537e82, 6);
  MD5STEP(f4, d, a, b, c, in[11] + 0xbd3af235, 10);
  MD5STEP(f4, c, d, a, b, in[2] + 0x2ad7d2bb, 15);
  MD5STEP(f4, b, c, d, a, in[9] + 0xeb86d391, 21);
#undef MD5STEP

  buf[0] += a;
  buf[1] += b;
  buf[2] += c;
  buf[3] += d;
}


void md5Digest(const uint8_t* data, size_t length, uint8_t out[16]) {
  uint32_t state[4] = {0x67452301, 0xefcdab89, 0x98badcfe, 0x10325476};
  uint8_t block[64];
  size_t offset = 0;
  while (length - offset >= 64) {
    uint32_t words[16];
    std::memcpy(words, data + offset, 64);
    md5Transform(state, words);
    offset += 64;
  }

  const size_t tail = length - offset;
  std::memcpy(block, data + offset, tail);
  block[tail] = 0x80;
  if (tail + 1 > 56) {
    std::memset(block + tail + 1, 0, 64 - tail - 1);
    uint32_t words[16];
    std::memcpy(words, block, 64);
    md5Transform(state, words);
    std::memset(block, 0, 56);
  } else {
    std::memset(block + tail + 1, 0, 56 - tail - 1);
  }
  const uint64_t bits = static_cast<uint64_t>(length) * 8;
  for (int i = 0; i < 8; ++i) block[56 + i] = static_cast<uint8_t>(bits >> (8 * i));
  uint32_t words[16];
  std::memcpy(words, block, 64);
  md5Transform(state, words);
  std::memcpy(out, state, 16);
}

}  // namespace freeink::sim::emu
