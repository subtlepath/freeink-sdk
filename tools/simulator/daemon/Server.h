#pragma once

// FreeInk simulator — control socket.
//
// The protocol is deliberately dull: one request per line, whitespace
// separated with shell-style quoting, and one JSON object per line back. That
// makes it trivially drivable from a shell, from Python, or by an agent, and
// keeps a JSON parser out of the daemon.
//
//   > press confirm --hold 800
//   < {"ok":true,"button":"confirm","held_ms":800}
//
// Every reply carries "ok"; failures carry "error" with a human-readable
// message and nothing else the caller has to parse.

#include <atomic>
#include <functional>
#include <map>
#include <string>
#include <thread>
#include <vector>

namespace freeink::sim {

class Machine;
class Runtime;

class Server {
 public:
  Server(Machine& machine, Runtime& runtime);
  ~Server();

  // Binds the socket, removing a stale one left by a crashed daemon. Returns
  // false with `error` set if another live daemon already holds it.
  bool listen(const std::string& socketPath, std::string* error);
  void start();
  void stop();

  const std::string& socketPath() const { return socketPath_; }

 private:
  void acceptLoop();
  void serveClient(int fd);
  std::string dispatch(const std::string& line);

  Machine& machine_;
  Runtime& runtime_;
  std::string socketPath_;
  int listenFd_ = -1;
  std::atomic<bool> running_{false};
  std::thread thread_;
  std::vector<std::thread> clients_;
};

// Parsed command line: the verb plus positional arguments and --flags.
struct Command {
  std::string verb;
  std::vector<std::string> args;
  std::map<std::string, std::string> flags;

  bool has(const std::string& flag) const { return flags.count(flag) > 0; }
  std::string flag(const std::string& name, const std::string& fallback = "") const {
    auto it = flags.find(name);
    return it == flags.end() ? fallback : it->second;
  }
  long number(const std::string& name, long fallback) const;
  std::string arg(size_t index, const std::string& fallback = "") const {
    return index < args.size() ? args[index] : fallback;
  }
};

Command parseCommand(const std::string& line);

// Minimal JSON emission — enough for flat replies with nested arrays.
std::string jsonEscape(const std::string& text);
std::string jsonError(const std::string& message);

}  // namespace freeink::sim
