#pragma once

// FreeInk simulator — image and audio output helpers.
//
// Greyscale PNG because that is what an e-paper frame is, and because a PNG is
// the one format every viewer, diff tool and agent can read without help.

#include <cstdint>
#include <string>
#include <vector>

namespace freeink::sim {

bool encodeGrayPng(const uint8_t* pixels, int width, int height, std::string* out);
bool writeGrayPng(const std::string& path, const uint8_t* pixels, int width, int height, std::string* error);
std::string base64Encode(const std::string& data);

// 16-bit PCM WAV, for the captured I2S stream.
bool writeWav(const std::string& path, const std::vector<uint8_t>& samples, uint32_t sampleRate, uint8_t bits,
              uint8_t channels, std::string* error);

}  // namespace freeink::sim
