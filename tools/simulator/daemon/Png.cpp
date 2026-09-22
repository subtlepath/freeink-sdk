// FreeInk simulator — image and audio output helpers.

#include "Png.h"

#include <zlib.h>

#include <cstring>
#include <fstream>

namespace freeink::sim {
namespace {

void appendBigEndian32(std::string* out, uint32_t value) {
  out->push_back(static_cast<char>((value >> 24) & 0xFF));
  out->push_back(static_cast<char>((value >> 16) & 0xFF));
  out->push_back(static_cast<char>((value >> 8) & 0xFF));
  out->push_back(static_cast<char>(value & 0xFF));
}

void appendChunk(std::string* out, const char type[4], const std::string& data) {
  appendBigEndian32(out, static_cast<uint32_t>(data.size()));
  const size_t crcStart = out->size();
  out->append(type, 4);
  out->append(data);
  const uLong crc = ::crc32(0, reinterpret_cast<const Bytef*>(out->data() + crcStart),
                            static_cast<uInt>(4 + data.size()));
  appendBigEndian32(out, static_cast<uint32_t>(crc));
}

}  // namespace

bool encodeGrayPng(const uint8_t* pixels, int width, int height, std::string* out) {
  if (!pixels || !out || width <= 0 || height <= 0) return false;
  out->clear();
  out->append("\x89PNG\r\n\x1a\n", 8);

  // IHDR: 8-bit greyscale, no interlace.
  std::string ihdr;
  appendBigEndian32(&ihdr, static_cast<uint32_t>(width));
  appendBigEndian32(&ihdr, static_cast<uint32_t>(height));
  ihdr.push_back(8);  // bit depth
  ihdr.push_back(0);  // colour type: greyscale
  ihdr.push_back(0);  // compression
  ihdr.push_back(0);  // filter
  ihdr.push_back(0);  // interlace
  appendChunk(out, "IHDR", ihdr);

  // Raw scanlines, each prefixed with filter type 0. An e-paper frame is mostly
  // flat white, so plain deflate already compresses it well.
  std::string raw;
  raw.reserve(static_cast<size_t>(height) * (width + 1));
  for (int y = 0; y < height; ++y) {
    raw.push_back(0);
    raw.append(reinterpret_cast<const char*>(pixels + static_cast<size_t>(y) * width), width);
  }

  uLongf compressedSize = ::compressBound(static_cast<uLong>(raw.size()));
  std::string compressed(compressedSize, '\0');
  if (::compress2(reinterpret_cast<Bytef*>(&compressed[0]), &compressedSize,
                  reinterpret_cast<const Bytef*>(raw.data()), static_cast<uLong>(raw.size()), 6) != Z_OK) {
    return false;
  }
  compressed.resize(compressedSize);
  appendChunk(out, "IDAT", compressed);
  appendChunk(out, "IEND", "");
  return true;
}

bool writeGrayPng(const std::string& path, const uint8_t* pixels, int width, int height, std::string* error) {
  std::string png;
  if (!encodeGrayPng(pixels, width, height, &png)) {
    if (error) *error = "failed to encode the frame as PNG";
    return false;
  }
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  if (!out) {
    if (error) *error = "cannot write " + path;
    return false;
  }
  out.write(png.data(), static_cast<std::streamsize>(png.size()));
  if (!out) {
    if (error) *error = "short write to " + path;
    return false;
  }
  return true;
}

std::string base64Encode(const std::string& data) {
  static const char* kAlphabet = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  out.reserve(((data.size() + 2) / 3) * 4);
  for (size_t i = 0; i < data.size(); i += 3) {
    const uint32_t a = static_cast<uint8_t>(data[i]);
    const uint32_t b = i + 1 < data.size() ? static_cast<uint8_t>(data[i + 1]) : 0;
    const uint32_t c = i + 2 < data.size() ? static_cast<uint8_t>(data[i + 2]) : 0;
    const uint32_t triple = (a << 16) | (b << 8) | c;
    out += kAlphabet[(triple >> 18) & 0x3F];
    out += kAlphabet[(triple >> 12) & 0x3F];
    out += i + 1 < data.size() ? kAlphabet[(triple >> 6) & 0x3F] : '=';
    out += i + 2 < data.size() ? kAlphabet[triple & 0x3F] : '=';
  }
  return out;
}

bool writeWav(const std::string& path, const std::vector<uint8_t>& samples, uint32_t sampleRate, uint8_t bits,
              uint8_t channels, std::string* error) {
  if (samples.empty()) {
    if (error) *error = "no audio has been captured yet";
    return false;
  }
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  if (!out) {
    if (error) *error = "cannot write " + path;
    return false;
  }

  const uint32_t dataSize = static_cast<uint32_t>(samples.size());
  const uint16_t blockAlign = static_cast<uint16_t>(channels * bits / 8);
  const uint32_t byteRate = sampleRate * blockAlign;

  auto put32 = [&out](uint32_t v) {
    const uint8_t bytes[4] = {static_cast<uint8_t>(v), static_cast<uint8_t>(v >> 8), static_cast<uint8_t>(v >> 16),
                              static_cast<uint8_t>(v >> 24)};
    out.write(reinterpret_cast<const char*>(bytes), 4);
  };
  auto put16 = [&out](uint16_t v) {
    const uint8_t bytes[2] = {static_cast<uint8_t>(v), static_cast<uint8_t>(v >> 8)};
    out.write(reinterpret_cast<const char*>(bytes), 2);
  };

  out.write("RIFF", 4);
  put32(36 + dataSize);
  out.write("WAVEfmt ", 8);
  put32(16);
  put16(1);  // PCM
  put16(channels);
  put32(sampleRate);
  put32(byteRate);
  put16(blockAlign);
  put16(bits);
  out.write("data", 4);
  put32(dataSize);
  out.write(reinterpret_cast<const char*>(samples.data()), static_cast<std::streamsize>(dataSize));
  return static_cast<bool>(out);
}

}  // namespace freeink::sim
