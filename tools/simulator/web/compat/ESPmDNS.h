#pragma once
struct WebMDNS { void end() {} bool begin(const char*) { return false; } };
inline WebMDNS MDNS;
