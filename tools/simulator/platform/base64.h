#pragma once

// FreeInk simulator — Arduino base64 helper.

#include <Arduino.h>

class base64 {
 public:
  static String encode(const uint8_t* data, size_t length);
  static String encode(const String& text) {
    return encode(reinterpret_cast<const uint8_t*>(text.c_str()), text.length());
  }
};
