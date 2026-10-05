#pragma once
#include <Arduino.h>
#include <functional>
#include <string>
namespace freeink {
class SecureHttpClient {
 public:
  void setInsecure() {}
  void setTimeout(uint32_t) {}
  bool begin(const std::string&) { return false; }
  void end() {}
  void addHeader(const std::string&, const std::string&) {}
  int GET() { return -1; }
  int POST(const std::string&) { return -1; }
  int PUT(const std::string&) { return -1; }
  int sendRequest(const std::string&, const std::string&) { return -1; }
  std::string getString() { return {}; }
};
}
