#pragma once
#include <IPAddress.h>
enum class DNSReplyCode { NoError };
class DNSServer {
 public:
  void stop() {}
  void setErrorReplyCode(DNSReplyCode) {}
  bool start(uint16_t, const char*, IPAddress) { return false; }
  void processNextRequest() {}
};
