// FreeInk simulator — control socket and command surface.

#include "Server.h"

#include "../core/I2cDevices.h"
#include "../core/Machine.h"
#include "../core/VirtualCard.h"
#include "../core/Panel.h"
#include "../emu/EmuMachine.h"
#include "Png.h"
#include "Runtime.h"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <thread>
#include <cstring>
#include <fstream>
#include <sstream>

#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

namespace freeink::sim {
namespace {

std::string quote(const std::string& text) { return "\"" + jsonEscape(text) + "\""; }

std::string okReply(const std::string& fields = "") {
  return fields.empty() ? "{\"ok\":true}" : "{\"ok\":true," + fields + "}";
}

const char* refreshKindName(RefreshKind kind) {
  switch (kind) {
    case RefreshKind::Full: return "full";
    case RefreshKind::Partial: return "partial";
    case RefreshKind::Fast: return "fast";
    default: return "unknown";
  }
}

const char* powerStateName(PowerState state) {
  switch (state) {
    case PowerState::Running: return "running";
    case PowerState::DeepSleep: return "deep-sleep";
    case PowerState::LightSleep: return "light-sleep";
    default: return "off";
  }
}

std::string hexByte(uint8_t value) {
  static const char* kHex = "0123456789ABCDEF";
  return std::string(1, kHex[value >> 4]) + std::string(1, kHex[value & 0x0F]);
}

}  // namespace

long Command::number(const std::string& name, long fallback) const {
  auto it = flags.find(name);
  if (it == flags.end() || it->second.empty()) return fallback;
  try {
    return std::stol(it->second);
  } catch (...) {
    return fallback;
  }
}

std::string jsonEscape(const std::string& text) {
  std::string out;
  out.reserve(text.size() + 8);
  for (char c : text) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          char buf[8];
          snprintf(buf, sizeof(buf), "\\u%04x", c);
          out += buf;
        } else {
          out += c;
        }
    }
  }
  return out;
}

std::string jsonError(const std::string& message) {
  return "{\"ok\":false,\"error\":\"" + jsonEscape(message) + "\"}";
}

Command parseCommand(const std::string& line) {
  Command command;
  std::vector<std::string> tokens;
  std::string current;
  bool inQuotes = false;
  bool escaped = false;
  bool haveToken = false;

  for (char c : line) {
    if (escaped) {
      current += c;
      escaped = false;
      haveToken = true;
      continue;
    }
    if (c == '\\') {
      escaped = true;
      continue;
    }
    if (c == '"') {
      inQuotes = !inQuotes;
      haveToken = true;
      continue;
    }
    if (!inQuotes && (c == ' ' || c == '\t')) {
      if (haveToken) tokens.push_back(current);
      current.clear();
      haveToken = false;
      continue;
    }
    current += c;
    haveToken = true;
  }
  if (haveToken) tokens.push_back(current);
  if (tokens.empty()) return command;

  command.verb = tokens[0];
  for (size_t i = 1; i < tokens.size(); ++i) {
    const std::string& token = tokens[i];
    if (token.rfind("--", 0) == 0) {
      const std::string name = token.substr(2);
      const size_t equals = name.find('=');
      if (equals != std::string::npos) {
        command.flags[name.substr(0, equals)] = name.substr(equals + 1);
      } else if (i + 1 < tokens.size() && tokens[i + 1].rfind("--", 0) != 0) {
        command.flags[name] = tokens[++i];
      } else {
        command.flags[name] = "true";  // bare flag
      }
    } else {
      command.args.push_back(token);
    }
  }
  return command;
}

// ── Server ───────────────────────────────────────────────────────────────────
Server::Server(Machine& machine, Runtime& runtime) : machine_(machine), runtime_(runtime) {}

Server::~Server() { stop(); }

bool Server::listen(const std::string& socketPath, std::string* error) {
  socketPath_ = socketPath;

  // A stale socket from a crashed daemon must not block startup, but a live one
  // must: probe it by connecting before unlinking.
  struct stat st {};
  if (::stat(socketPath.c_str(), &st) == 0) {
    const int probe = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (probe >= 0) {
      struct sockaddr_un addr {};
      addr.sun_family = AF_UNIX;
      std::strncpy(addr.sun_path, socketPath.c_str(), sizeof(addr.sun_path) - 1);
      const bool alive = ::connect(probe, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) == 0;
      ::close(probe);
      if (alive) {
        if (error) *error = "another freeink-simd is already listening on " + socketPath;
        return false;
      }
    }
    ::unlink(socketPath.c_str());
  }

  listenFd_ = ::socket(AF_UNIX, SOCK_STREAM, 0);
  if (listenFd_ < 0) {
    if (error) *error = std::string("socket: ") + std::strerror(errno);
    return false;
  }

  struct sockaddr_un addr {};
  addr.sun_family = AF_UNIX;
  if (socketPath.size() >= sizeof(addr.sun_path)) {
    if (error) *error = "socket path is too long: " + socketPath;
    return false;
  }
  std::strncpy(addr.sun_path, socketPath.c_str(), sizeof(addr.sun_path) - 1);

  if (::bind(listenFd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) != 0) {
    if (error) *error = std::string("bind ") + socketPath + ": " + std::strerror(errno);
    ::close(listenFd_);
    listenFd_ = -1;
    return false;
  }
  // Owner-only: the socket is a full control channel over the firmware.
  ::chmod(socketPath.c_str(), 0600);

  if (::listen(listenFd_, 8) != 0) {
    if (error) *error = std::string("listen: ") + std::strerror(errno);
    ::close(listenFd_);
    listenFd_ = -1;
    return false;
  }
  return true;
}

void Server::start() {
  running_ = true;
  thread_ = std::thread([this]() { acceptLoop(); });
}

void Server::stop() {
  if (!running_.exchange(false)) return;
  if (listenFd_ >= 0) {
    ::shutdown(listenFd_, SHUT_RDWR);
    ::close(listenFd_);
    listenFd_ = -1;
  }
  if (thread_.joinable()) thread_.join();
  for (auto& client : clients_) {
    if (client.joinable()) client.detach();
  }
  if (!socketPath_.empty()) ::unlink(socketPath_.c_str());
}

void Server::acceptLoop() {
  while (running_.load()) {
    const int fd = ::accept(listenFd_, nullptr, nullptr);
    if (fd < 0) {
      if (!running_.load()) break;
      continue;
    }
    clients_.emplace_back([this, fd]() { serveClient(fd); });
    // Reap finished client threads so a long-lived daemon does not accumulate
    // them; anything still running is joined at stop().
    clients_.erase(std::remove_if(clients_.begin(), clients_.end(),
                                  [](std::thread& t) {
                                    if (!t.joinable()) return true;
                                    return false;
                                  }),
                   clients_.end());
  }
}

void Server::serveClient(int fd) {
  std::string buffer;
  char chunk[4096];
  for (;;) {
    const ssize_t n = ::read(fd, chunk, sizeof(chunk));
    if (n <= 0) break;
    buffer.append(chunk, static_cast<size_t>(n));

    size_t newline;
    while ((newline = buffer.find('\n')) != std::string::npos) {
      const std::string line = buffer.substr(0, newline);
      buffer.erase(0, newline + 1);
      std::string reply;
      try {
        reply = dispatch(line);
      } catch (const std::exception& e) {
        reply = jsonError(std::string("command failed: ") + e.what());
      }
      reply += "\n";
      if (::write(fd, reply.data(), reply.size()) < 0) {
        ::close(fd);
        return;
      }
    }
  }
  ::close(fd);
}

std::string Server::dispatch(const std::string& line) {
  const Command command = parseCommand(line);
  if (command.verb.empty()) return jsonError("empty command");

  Machine& machine = machine_;
  auto& panel = machine.panel();

  // ── Lifecycle ──────────────────────────────────────────────────────────────
  if (command.verb == "ping") return okReply("\"daemon\":\"freeink-simd\"");

  if (command.verb == "load") {
    const std::string path = command.arg(0);
    if (path.empty()) return jsonError("usage: load <bundle.dylib|bundle.so>");
    std::string error;
    if (!runtime_.load(path, &error)) return jsonError(error);
    return okReply("\"bundle\":" + quote(path) + ",\"firmware\":" + quote(runtime_.firmwareName()));
  }

  if (command.verb == "reset") {
    std::string error;
    if (!runtime_.reset(&error)) return jsonError(error);
    return okReply();
  }

  if (command.verb == "unload") {
    runtime_.stop();
    return okReply();
  }

  if (command.verb == "status") {
    const auto& board = machine.board();
    const Frame frame = panel.currentFrame();
    std::ostringstream out;
    out << "{\"ok\":true"
        << ",\"firmware\":" << quote(runtime_.firmwareName())
        << ",\"bundle\":" << quote(runtime_.bundlePath())
        << ",\"running\":" << (runtime_.running() ? "true" : "false")
        << ",\"board\":" << quote(machine.boardKnown() && board.board_name ? board.board_name : "unknown")
        << ",\"controller\":" << quote(panel.controllerName())
        << ",\"panel\":{\"width\":" << panel.width() << ",\"height\":" << panel.height() << "}"
        << ",\"clock\":{\"us\":" << machine.clock().nowUs()
        << ",\"mode\":" << quote(machine.clock().mode() == ClockMode::Virtual ? "virtual" : "realtime")
        << ",\"rate\":" << machine.clock().rate()
        << ",\"paused\":" << (machine.clock().paused() ? "true" : "false") << "}"
        << ",\"power\":" << quote(powerStateName(machine.powerState()))
        << ",\"busy\":" << (panel.busy() ? "true" : "false")
        << ",\"frame\":{\"seq\":" << frame.seq << ",\"kind\":" << quote(refreshKindName(frame.kind))
        << ",\"duration_ms\":" << frame.durationMs << "}"
        << ",\"sd\":" << (machine.storage().cardPresent() ? quote(machine.storage().cardRoot()) : "null");
    if (const emu::EmuMachine* emulator = runtime_.emulator()) {
      // An emulated image is a different kind of firmware, and `status` says so
      // rather than looking identical to a host build that happens to be slow.
      out << ",\"emulated\":{\"chip\":" << quote(emulator->soc().name)
          << ",\"instructions\":" << emulator->cpu().retired()
          << ",\"interrupts\":" << emulator->cpu().interruptsTaken()
          << ",\"stopped\":" << (emulator->stopped() ? "true" : "false") << "}";
    }
    out << "}";
    return out.str();
  }

  // ── Emulated device images ─────────────────────────────────────────────────
  if (command.verb == "emu") {
    const emu::EmuMachine* emulator = runtime_.emulator();
    if (!emulator) {
      return jsonError("no device image is loaded — `emu` reports on firmware loaded from a .bin");
    }
    const emu::AppImage& app = emulator->flash().app();
    std::ostringstream out;
    out << "{\"ok\":true"
        << ",\"chip\":" << quote(emulator->soc().name)
        << ",\"isa\":" << quote(emulator->cpu().isaName())
        << ",\"image\":" << quote(runtime_.bundlePath())
        << ",\"project\":" << quote(app.desc.projectName)
        << ",\"version\":" << quote(app.desc.version)
        << ",\"idf\":" << quote(app.desc.idfVersion)
        << ",\"entry\":" << app.entry
        << ",\"pc\":" << emulator->cpu().pc()
        << ",\"instructions\":" << emulator->cpu().retired()
        << ",\"interrupts\":" << emulator->cpu().interruptsTaken()
        << ",\"exceptions\":" << emulator->cpu().exceptionsTaken()
        << ",\"time_us\":" << emulator->timeUs()
        << ",\"stopped\":" << (emulator->stopped() ? "true" : "false")
        << ",\"reason\":" << quote(emulator->stopped() ? emulator->stopReason() : "running")
        << ",\"partitions_synthesized\":" << (emulator->flash().synthesizedTable() ? "true" : "false")
        << ",\"gaps\":[";
    const std::vector<std::string> gaps = emulator->gaps();
    for (size_t i = 0; i < gaps.size(); ++i) {
      out << (i ? "," : "") << quote(gaps[i]);
    }
    out << "]}";
    return out.str();
  }

  if (command.verb == "mem") {
    emu::EmuMachine* emulator = runtime_.emulator();
    if (!emulator) return jsonError("no device image is loaded — `mem` reads emulated memory");
    if (command.args.empty()) return jsonError("usage: mem <address> [length]");
    const uint32_t address = static_cast<uint32_t>(std::stoul(command.args[0], nullptr, 0));
    const uint32_t length = static_cast<uint32_t>(
        command.args.size() > 1 ? std::stoul(command.args[1], nullptr, 0) : command.number("length", 64));
    if (length == 0 || length > 4096) return jsonError("length must be between 1 and 4096 bytes");

    std::vector<uint8_t> bytes;
    if (!emulator->bus().peek(address, length, &bytes)) {
      return jsonError("nothing is mapped at that address: " + emulator->bus().describe(address));
    }
    std::ostringstream out;
    out << "{\"ok\":true,\"address\":" << address << ",\"region\":"
        << quote(emulator->bus().describe(address)) << ",\"bytes\":\"";
    for (uint8_t byte : bytes) out << hexByte(byte);
    out << "\"}";
    return out.str();
  }

  if (command.verb == "registers") {
    emu::EmuMachine* emulator = runtime_.emulator();
    if (!emulator) return jsonError("no device image is loaded — `registers` reports emulated peripherals");
    std::ostringstream out;
    out << "{\"ok\":true,\"registers\":[";
    bool first = true;
    if (command.args.empty()) {
      // No block named: report what the firmware touched that nothing models,
      // which is the list that explains an image behaving oddly.
      for (const emu::RegisterAccess& access : emulator->unmodelledRegisters(command.number("limit", 20))) {
        out << (first ? "" : ",") << "{\"address\":" << access.address
            << ",\"write\":" << (access.write ? "true" : "false") << ",\"count\":" << access.count
            << ",\"value\":" << access.value << "}";
        first = false;
      }
      out << "],\"scope\":\"unmodelled\"}";
      return out.str();
    }
    const uint32_t base = static_cast<uint32_t>(std::stoul(command.args[0], nullptr, 0));
    for (const emu::RegisterAccess& access : emulator->registersAt(base)) {
      out << (first ? "" : ",") << "{\"address\":" << access.address
          << ",\"write\":" << (access.write ? "true" : "false") << ",\"count\":" << access.count
          << ",\"value\":" << access.value << "}";
      first = false;
    }
    out << "],\"scope\":\"block\"}";
    return out.str();
  }

  // ── Clock ──────────────────────────────────────────────────────────────────
  if (command.verb == "pause") {
    machine.clock().pause();
    return okReply();
  }
  if (command.verb == "resume") {
    machine.clock().resume();
    return okReply();
  }
  if (command.verb == "step") {
    const long ms = command.args.empty() ? command.number("ms", 100) : std::stol(command.args[0]);
    machine.clock().step(static_cast<uint64_t>(ms) * 1000ULL);
    return okReply("\"stepped_ms\":" + std::to_string(ms) + ",\"clock_us\":" +
                   std::to_string(machine.clock().nowUs()));
  }
  if (command.verb == "clock") {
    if (command.has("mode")) {
      const std::string mode = command.flag("mode");
      if (mode != "virtual" && mode != "realtime") return jsonError("clock mode must be virtual or realtime");
      machine.clock().setMode(mode == "virtual" ? ClockMode::Virtual : ClockMode::Realtime);
    }
    if (command.has("rate")) machine.clock().setRate(std::stod(command.flag("rate")));
    return okReply("\"us\":" + std::to_string(machine.clock().nowUs()));
  }

  // ── Input ──────────────────────────────────────────────────────────────────
  if (command.verb == "press" || command.verb == "release" || command.verb == "hold") {
    const std::string name = command.arg(0);
    const int index = Machine::buttonIndexFromName(name);
    if (index < 0) {
      return jsonError("unknown button '" + name + "' (back, confirm, left, right, up, down, power)");
    }
    if (command.verb == "release") {
      machine.setButton(index, false);
      return okReply("\"button\":" + quote(name));
    }
    if (command.verb == "hold") {
      machine.setButton(index, true);
      return okReply("\"button\":" + quote(name) + ",\"held\":true");
    }
    // press: hold the button down until the firmware has actually sampled it,
    // then release. A fixed short hold would be swallowed whenever the press
    // landed during a blocking refresh — which is real hardware behaviour, but
    // makes scripted input a race. Holding until the device notices is what a
    // person does, and it keeps the input path itself completely unmodelled.
    //
    // --hold forces a fixed duration instead, for testing long-press handling.
    const uint64_t before = machine.inputSampleCount(index);
    // Generous by default: a press can arrive while the firmware is inside a
    // blocking refresh, and a multi-pass full refresh (the X3's is three
    // waveform passes) takes several seconds before input is sampled again.
    const long timeoutMs = command.number("timeout", 15000);
    machine.setButton(index, true);

    const auto startedAt = std::chrono::steady_clock::now();
    const auto deadline = startedAt + std::chrono::milliseconds(timeoutMs);
    uint64_t samples = 0;
    if (command.has("hold")) {
      const long holdMs = command.number("hold", 60);
      const uint64_t until = machine.clock().nowUs() + static_cast<uint64_t>(holdMs) * 1000ULL;
      while (machine.clock().nowUs() < until && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      samples = machine.inputSampleCount(index) - before;
    } else {
      // InputManager commits a state change only after two consecutive matching
      // samples, so wait for three to clear its debounce with margin.
      constexpr uint64_t kSamplesNeeded = 3;
      while (std::chrono::steady_clock::now() < deadline) {
        samples = machine.inputSampleCount(index) - before;
        if (samples >= kSamplesNeeded) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      if (samples < kSamplesNeeded) {
        machine.setButton(index, false);
        return jsonError("the firmware did not sample '" + name + "' within " + std::to_string(timeoutMs) +
                         " ms — it may still be inside a blocking refresh. Raise --timeout, or check "
                         "`log` to see where it is.");
      }
    }

    // Release, then wait for the release edge to be sampled too. Without this
    // the command could return while the button is still logically down for the
    // firmware, and the next press would merge into this one — which is exactly
    // how a scripted sequence loses events.
    const uint64_t afterRelease = machine.inputSampleCount(index);
    machine.setButton(index, false);
    while (std::chrono::steady_clock::now() < deadline) {
      if (machine.inputSampleCount(index) - afterRelease >= 2) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    const auto heldMs =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - startedAt).count();
    return okReply("\"button\":" + quote(name) + ",\"held_ms\":" + std::to_string(heldMs) + ",\"samples\":" +
                   std::to_string(samples));
  }

  if (command.verb == "tap" || command.verb == "touch") {
    if (machine.board().touch_controller == FSIM_TOUCH_NONE) {
      return jsonError(std::string("board '") + (machine.board().board_name ? machine.board().board_name : "?") +
                       "' has no touch controller");
    }
    const int x = static_cast<int>(command.number("x", command.args.size() > 0 ? std::stol(command.args[0]) : 0));
    const int y = static_cast<int>(command.number("y", command.args.size() > 1 ? std::stol(command.args[1]) : 0));
    if (command.verb == "touch" && command.has("up")) {
      machine.clearTouch();
      return okReply();
    }
    machine.setTouch(x, y, true);
    if (command.verb == "tap") {
      const long holdMs = command.number("hold", 80);
      const uint64_t deadline = machine.clock().nowUs() + static_cast<uint64_t>(holdMs) * 1000ULL;
      while (machine.clock().nowUs() < deadline) std::this_thread::sleep_for(std::chrono::milliseconds(1));
      machine.clearTouch();
    }
    return okReply("\"x\":" + std::to_string(x) + ",\"y\":" + std::to_string(y));
  }

  // ── Screen ─────────────────────────────────────────────────────────────────
  if (command.verb == "capture") {
    Frame frame;
    if (command.has("wait-refresh")) {
      const long timeoutMs = command.number("timeout", 5000);
      const uint64_t after = command.has("after") ? static_cast<uint64_t>(command.number("after", 0))
                                                  : panel.frameSeq();
      if (!panel.waitForFrame(after, static_cast<uint64_t>(timeoutMs), &frame)) {
        return jsonError("timed out after " + std::to_string(timeoutMs) +
                         " ms waiting for a refresh newer than frame #" + std::to_string(after) +
                         ". The firmware may not have repainted — or it already had, in which case "
                         "capture without --wait-refresh, or pass --after with the frame number you "
                         "saw before the action.");
      }
    } else {
      frame = panel.currentFrame();
    }
    if (frame.pixels.empty()) return jsonError("no frame yet: the firmware has not refreshed the panel");

    const std::string out = command.flag("out");
    std::string fields = "\"seq\":" + std::to_string(frame.seq) + ",\"width\":" + std::to_string(frame.width) +
                         ",\"height\":" + std::to_string(frame.height) + ",\"kind\":" +
                         quote(refreshKindName(frame.kind));
    if (!out.empty()) {
      std::string error;
      if (!writeGrayPng(out, frame.pixels.data(), frame.width, frame.height, &error)) return jsonError(error);
      return okReply(fields + ",\"path\":" + quote(out));
    }
    std::string png;
    if (!encodeGrayPng(frame.pixels.data(), frame.width, frame.height, &png)) {
      return jsonError("failed to encode the frame as PNG");
    }
    return okReply(fields + ",\"png_base64\":" + quote(base64Encode(png)));
  }

  if (command.verb == "wait-refresh") {
    const long timeoutMs = command.number("timeout", 5000);
    Frame frame;
    if (!panel.waitForFrame(panel.frameSeq(), static_cast<uint64_t>(timeoutMs), &frame)) {
      return jsonError("timed out after " + std::to_string(timeoutMs) + " ms waiting for a refresh");
    }
    return okReply("\"seq\":" + std::to_string(frame.seq) + ",\"kind\":" + quote(refreshKindName(frame.kind)) +
                   ",\"duration_ms\":" + std::to_string(frame.durationMs));
  }

  // ── Logs ───────────────────────────────────────────────────────────────────
  if (command.verb == "log") {
    const uint64_t since = static_cast<uint64_t>(command.number("since", 0));
    const auto lines = machine.log().since(since, command.flag("channel"));
    std::ostringstream out;
    out << "{\"ok\":true,\"last\":" << machine.log().lastSeq() << ",\"lines\":[";
    for (size_t i = 0; i < lines.size(); ++i) {
      if (i) out << ",";
      out << "{\"seq\":" << lines[i].first << ",\"text\":" << quote(lines[i].second) << "}";
    }
    out << "]}";
    return out.str();
  }

  if (command.verb == "log-clear") {
    machine.log().clear();
    return okReply();
  }

  // ── Bus trace ──────────────────────────────────────────────────────────────
  if (command.verb == "bus") {
    if (command.arg(0) == "clear") {
      panel.clearBusTrace();
      return okReply();
    }
    const size_t limit = static_cast<size_t>(command.number("limit", 64));
    const auto events = panel.busTrace(limit);
    std::ostringstream out;
    out << "{\"ok\":true,\"events\":[";
    for (size_t i = 0; i < events.size(); ++i) {
      if (i) out << ",";
      out << "{\"us\":" << events[i].timeUs << ",\"cmd\":" << quote(hexByte(events[i].command))
          << ",\"name\":" << quote(events[i].name ? events[i].name : "?")
          << ",\"len\":" << events[i].dataLength << ",\"data\":" << quote([&] {
               std::string hex;
               for (uint8_t b : events[i].data) hex += hexByte(b) + " ";
               if (!hex.empty()) hex.pop_back();
               return hex;
             }())
          << "}";
    }
    out << "]}";
    return out.str();
  }

  // ── I2C ────────────────────────────────────────────────────────────────────
  if (command.verb == "i2c") {
    const auto devices = machine.i2c().devices();
    std::ostringstream out;
    out << "{\"ok\":true,\"devices\":[";
    for (size_t i = 0; i < devices.size(); ++i) {
      if (i) out << ",";
      out << "{\"addr\":" << quote("0x" + hexByte(devices[i].first)) << ",\"name\":" << quote(devices[i].second)
          << "}";
    }
    out << "]}";
    return out.str();
  }

  // ── Storage ────────────────────────────────────────────────────────────────
  if (command.verb == "sd") {
    const std::string action = command.arg(0);
    if (action == "mount") {
      const std::string dir = command.arg(1);
      if (dir.empty()) return jsonError("usage: sd mount <host-directory> [--capacity-mb N]");
      struct stat st {};
      if (::stat(dir.c_str(), &st) != 0 || !S_ISDIR(st.st_mode)) {
        return jsonError("not a directory: " + dir);
      }
      const long capacityMb = command.number("capacity-mb", 8192);
      std::string cardError;
      if (!machine.mountCard(dir, static_cast<uint64_t>(capacityMb) * 1024 * 1024, &cardError)) {
        return jsonError(cardError);
      }
      return okReply("\"mounted\":" + quote(dir) + ",\"capacity_mb\":" +
                     std::to_string(machine.card().capacityBytes() / (1024 * 1024)));
    }
    if (action == "eject") {
      machine.ejectCard();
      return okReply();
    }
    if (action == "image") {
      // A raw card image, for reproducing a card exactly — or for keeping what
      // the firmware wrote, which a directory-backed card deliberately does not.
      std::string cardError;
      if (!machine.mountCardImage(command.arg(1), &cardError)) return jsonError(cardError);
      return okReply("\"mounted\":" + quote(command.arg(1)));
    }
    return okReply("\"present\":" +
                   std::string(machine.card().present() ? "true" : "false") + ",\"root\":" +
                   quote(machine.card().present() ? machine.card().source()
                                                  : machine.storage().cardRoot()) +
                   ",\"capacity_mb\":" +
                   std::to_string(machine.card().capacityBytes() / (1024 * 1024)));
  }

  if (command.verb == "nvs") {
    const std::string action = command.arg(0);
    if (action == "get") {
      std::string value;
      if (!machine.storage().nvsGet(command.arg(1), command.arg(2), &value)) return jsonError("key not found");
      std::string hex;
      for (unsigned char c : value) hex += hexByte(c);
      return okReply("\"hex\":" + quote(hex) + ",\"text\":" + quote(value));
    }
    if (action == "set") {
      machine.storage().nvsSet(command.arg(1), command.arg(2), command.arg(3));
      return okReply();
    }
    if (action == "clear") {
      machine.storage().nvsClear(command.arg(1));
      return okReply();
    }
    const auto all = machine.storage().nvsAll();
    std::ostringstream out;
    out << "{\"ok\":true,\"entries\":[";
    bool first = true;
    for (const auto& [key, value] : all) {
      if (!first) out << ",";
      first = false;
      std::string ns = key;
      std::string name;
      const size_t sep = key.find('\x1f');
      if (sep != std::string::npos) {
        ns = key.substr(0, sep);
        name = key.substr(sep + 1);
      }
      out << "{\"ns\":" << quote(ns) << ",\"key\":" << quote(name) << ",\"bytes\":" << value.size() << "}";
    }
    out << "]}";
    return out.str();
  }

  // ── Power and peripherals ──────────────────────────────────────────────────
  if (command.verb == "battery") {
    std::lock_guard<std::mutex> lock(machine.mutex());
    auto& p = machine.peripherals();
    if (command.has("percent")) p.batteryPercent = static_cast<int>(command.number("percent", p.batteryPercent));
    if (command.has("mv")) p.batteryMillivolts = static_cast<int>(command.number("mv", p.batteryMillivolts));
    if (command.has("charging")) p.charging = command.flag("charging") != "false";
    return okReply("\"percent\":" + std::to_string(p.batteryPercent) + ",\"mv\":" +
                   std::to_string(p.batteryMillivolts) + ",\"charging\":" + (p.charging ? "true" : "false"));
  }

  if (command.verb == "usb") {
    std::lock_guard<std::mutex> lock(machine.mutex());
    auto& p = machine.peripherals();
    if (command.arg(0) == "plug") p.usbConnected = true;
    if (command.arg(0) == "unplug") p.usbConnected = false;
    return okReply("\"connected\":" + std::string(p.usbConnected ? "true" : "false") + ",\"msc\":" +
                   (p.mscActive ? "true" : "false"));
  }

  if (command.verb == "light" || command.verb == "pwm") {
    std::lock_guard<std::mutex> lock(machine.mutex());
    const auto& p = machine.peripherals();
    std::ostringstream out;
    out << "{\"ok\":true,\"channels\":[";
    bool first = true;
    for (const auto& [pin, duty] : p.pwmDuty) {
      if (!first) out << ",";
      first = false;
      auto freq = p.pwmFreq.find(pin);
      out << "{\"pin\":" << pin << ",\"duty\":" << duty
          << ",\"freq\":" << (freq == p.pwmFreq.end() ? 0 : freq->second) << "}";
    }
    out << "]}";
    return out.str();
  }

  if (command.verb == "imu") {
    std::lock_guard<std::mutex> lock(machine.mutex());
    auto& p = machine.peripherals();
    if (command.has("x")) p.imuAccel[0] = std::stod(command.flag("x"));
    if (command.has("y")) p.imuAccel[1] = std::stod(command.flag("y"));
    if (command.has("z")) p.imuAccel[2] = std::stod(command.flag("z"));
    std::ostringstream out;
    out << "{\"ok\":true,\"accel\":[" << p.imuAccel[0] << "," << p.imuAccel[1] << "," << p.imuAccel[2] << "]}";
    return out.str();
  }

  if (command.verb == "wifi") {
    std::lock_guard<std::mutex> lock(machine.mutex());
    auto& p = machine.peripherals();
    const std::string action = command.arg(0);
    if (action == "add") {
      Peripherals::Network network;
      network.ssid = command.arg(1);
      network.rssi = static_cast<int32_t>(command.number("rssi", -55));
      network.enc = static_cast<uint8_t>(command.number("enc", 3));
      network.password = command.flag("password");
      if (network.ssid.empty()) return jsonError("usage: wifi add <ssid> [--password P] [--rssi N]");
      p.networks.push_back(network);
      return okReply("\"ssid\":" + quote(network.ssid));
    }
    if (action == "clear") {
      p.networks.clear();
      return okReply();
    }
    if (action == "enable-internet") {
      p.networkEnabled = command.arg(1) != "false";
      return okReply("\"network_enabled\":" + std::string(p.networkEnabled ? "true" : "false"));
    }
    std::ostringstream out;
    out << "{\"ok\":true,\"state\":" << p.wifiState << ",\"ssid\":" << quote(p.wifiSsid)
        << ",\"internet\":" << (p.networkEnabled ? "true" : "false") << ",\"networks\":[";
    for (size_t i = 0; i < p.networks.size(); ++i) {
      if (i) out << ",";
      out << "{\"ssid\":" << quote(p.networks[i].ssid) << ",\"rssi\":" << p.networks[i].rssi << "}";
    }
    out << "]}";
    return out.str();
  }

  if (command.verb == "audio") {
    std::lock_guard<std::mutex> lock(machine.mutex());
    auto& p = machine.peripherals();
    if (command.arg(0) == "save") {
      const std::string path = command.arg(1);
      if (path.empty()) return jsonError("usage: audio save <path.wav>");
      std::string error;
      if (!writeWav(path, p.audioCapture, p.audioSampleRate ? p.audioSampleRate : 44100, p.audioBits,
                    p.audioChannels, &error)) {
        return jsonError(error);
      }
      return okReply("\"path\":" + quote(path) + ",\"bytes\":" + std::to_string(p.audioCapture.size()));
    }
    if (command.arg(0) == "clear") {
      p.audioCapture.clear();
      return okReply();
    }
    return okReply("\"open\":" + std::string(p.audioOpen ? "true" : "false") + ",\"captured_bytes\":" +
                   std::to_string(p.audioCapture.size()) + ",\"sample_rate\":" +
                   std::to_string(p.audioSampleRate));
  }

  if (command.verb == "gpio") {
    std::lock_guard<std::mutex> lock(machine.mutex());
    const int pin = static_cast<int>(command.number("pin", command.args.empty() ? -1 : std::stol(command.args[0])));
    if (pin < 0) return jsonError("usage: gpio <pin> [--drive 0|1|float]");
    if (command.has("drive")) {
      const std::string drive = command.flag("drive");
      machine.gpio().driveExternal(pin, drive == "float" ? Gpio::kFloat : (drive == "1" ? 1 : 0));
    }
    return okReply("\"pin\":" + std::to_string(pin) + ",\"level\":" + std::to_string(machine.gpio().read(pin)) +
                   ",\"held\":" + (machine.gpio().held(pin) ? "true" : "false"));
  }

  if (command.verb == "wake") {
    machine.wakeNow(static_cast<int>(command.number("cause", 7)));
    return okReply();
  }

  if (command.verb == "seed") {
    machine.seedRandom(static_cast<uint32_t>(command.number("value", 1)));
    return okReply();
  }

  if (command.verb == "quit" || command.verb == "shutdown") {
    runtime_.stop();
    machine.clock().shutdown();
    return okReply("\"shutdown\":true");
  }

  return jsonError("unknown command '" + command.verb + "'");
}

}  // namespace freeink::sim
