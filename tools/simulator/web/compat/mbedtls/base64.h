#pragma once
#include <cstddef>
#include <cstdint>
#define MBEDTLS_ERR_BASE64_BUFFER_TOO_SMALL -0x002A
#define MBEDTLS_ERR_BASE64_INVALID_CHARACTER -0x002C
inline int mbedtls_base64_decode(unsigned char* out, size_t capacity, size_t* length, const unsigned char* text, size_t size) {
  static const char alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  size_t count = 0; uint32_t acc = 0; int bits = 0; bool padding = false;
  for (size_t i=0; i<size; i++) {
    if (text[i]=='\r'||text[i]=='\n'||text[i]==' '||text[i]=='\t') continue;
    if (text[i]=='=') { padding = true; continue; }
    if (padding) return MBEDTLS_ERR_BASE64_INVALID_CHARACTER;
    const char* pos = strchr(alphabet, text[i]);
    if (!pos || !text[i]) return MBEDTLS_ERR_BASE64_INVALID_CHARACTER;
    acc = (acc<<6) | static_cast<uint32_t>(pos-alphabet); bits+=6;
    if (bits>=8) { bits-=8; if (out && count<capacity) out[count] = static_cast<uint8_t>(acc>>bits); count++; }
  }
  *length = count;
  return capacity<count ? MBEDTLS_ERR_BASE64_BUFFER_TOO_SMALL : 0;
}
