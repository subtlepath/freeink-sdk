#pragma once

// FreeInk simulator — Arduino String, backed by std::string.
//
// Only the surface the SDK and typical reader firmware use: construction from
// the scalar types, concatenation, comparison, indexOf/substring, and the
// c_str()/length() accessors. Kept implicitly convertible nowhere: Arduino's
// String has an operator bool via StringIfHelper, which real sketches lean on
// in `if (s)`, so that is provided too.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

class String {
 public:
  String() = default;
  String(const char* s) : _s(s ? s : "") {}
  explicit String(const std::string& s) : _s(s) {}
  String(char c) : _s(1, c) {}
  String(int v) : _s(std::to_string(v)) {}
  String(long v) : _s(std::to_string(v)) {}
  String(unsigned v) : _s(std::to_string(v)) {}
  String(unsigned long v) : _s(std::to_string(v)) {}
  String(unsigned long long v) : _s(std::to_string(v)) {}
  String(long long v) : _s(std::to_string(v)) {}
  String(double v, int decimals = 2) {
    char buf[64];
    snprintf(buf, sizeof(buf), "%.*f", decimals, v);
    _s = buf;
  }
  String(float v, int decimals = 2) : String(static_cast<double>(v), decimals) {}

  const char* c_str() const { return _s.c_str(); }
  size_t length() const { return _s.size(); }
  bool isEmpty() const { return _s.empty(); }
  void clear() { _s.clear(); }
  void reserve(size_t n) { _s.reserve(n); }
  bool concat(const char* text, size_t size) { if (!text) return false; _s.append(text, size); return true; }
  bool concat(const char* text) { return text && concat(text, strlen(text)); }
  bool concat(char text) { _s.push_back(text); return true; }

  char charAt(size_t i) const { return i < _s.size() ? _s[i] : '\0'; }
  char operator[](size_t i) const { return charAt(i); }
  char& operator[](size_t i) { return _s[i]; }

  String& operator+=(const String& o) { _s += o._s; return *this; }
  String& operator+=(const char* o) { if (o) _s += o; return *this; }
  String& operator+=(char c) { _s += c; return *this; }
  String& operator+=(int v) { _s += std::to_string(v); return *this; }
  String& operator+=(unsigned long v) { _s += std::to_string(v); return *this; }

  bool operator==(const String& o) const { return _s == o._s; }
  bool operator==(const char* o) const { return o && _s == o; }
  bool operator!=(const String& o) const { return !(*this == o); }
  bool operator!=(const char* o) const { return !(*this == o); }
  bool operator<(const String& o) const { return _s < o._s; }
  bool operator>(const String& o) const { return _s > o._s; }

  bool equals(const String& o) const { return _s == o._s; }
  bool equalsIgnoreCase(const String& o) const {
    if (_s.size() != o._s.size()) return false;
    for (size_t i = 0; i < _s.size(); ++i) {
      if (tolower(_s[i]) != tolower(o._s[i])) return false;
    }
    return true;
  }

  bool startsWith(const String& p) const { return _s.rfind(p._s, 0) == 0; }
  bool endsWith(const String& p) const {
    return p._s.size() <= _s.size() && _s.compare(_s.size() - p._s.size(), p._s.size(), p._s) == 0;
  }
  int indexOf(char c, size_t from = 0) const {
    const size_t p = _s.find(c, from);
    return p == std::string::npos ? -1 : static_cast<int>(p);
  }
  int indexOf(const String& n, size_t from = 0) const {
    const size_t p = _s.find(n._s, from);
    return p == std::string::npos ? -1 : static_cast<int>(p);
  }
  int lastIndexOf(char c) const {
    const size_t p = _s.rfind(c);
    return p == std::string::npos ? -1 : static_cast<int>(p);
  }
  String substring(size_t from) const { return from >= _s.size() ? String() : String(_s.substr(from)); }
  String substring(size_t from, size_t to) const {
    if (from >= _s.size() || to <= from) return String();
    return String(_s.substr(from, to - from));
  }
  void replace(const String& from, const String& to) {
    if (from._s.empty()) return;
    size_t p = 0;
    while ((p = _s.find(from._s, p)) != std::string::npos) {
      _s.replace(p, from._s.size(), to._s);
      p += to._s.size();
    }
  }
  void trim() {
    const size_t b = _s.find_first_not_of(" \t\r\n");
    const size_t e = _s.find_last_not_of(" \t\r\n");
    _s = (b == std::string::npos) ? "" : _s.substr(b, e - b + 1);
  }
  void toLowerCase() {
    for (auto& c : _s) c = static_cast<char>(tolower(c));
  }
  void toUpperCase() {
    for (auto& c : _s) c = static_cast<char>(toupper(c));
  }
  int toInt() const { return atoi(_s.c_str()); }
  float toFloat() const { return static_cast<float>(atof(_s.c_str())); }
  double toDouble() const { return atof(_s.c_str()); }

  const std::string& std_str() const { return _s; }

  explicit operator bool() const { return !_s.empty(); }

 private:
  std::string _s;
};

inline String operator+(const String& a, const String& b) {
  String r(a);
  r += b;
  return r;
}
inline String operator+(const String& a, const char* b) {
  String r(a);
  r += b;
  return r;
}
inline String operator+(const char* a, const String& b) {
  String r(a);
  r += b;
  return r;
}
inline String operator+(const String& a, char b) {
  String r(a);
  r += b;
  return r;
}
inline bool operator==(const char* a, const String& b) { return b == a; }
