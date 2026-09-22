// FreeInk simulator — the boards a device image can be run on.

#include "BoardTable.h"

#include "BoardDesc.h"

#include <algorithm>
#include <cctype>
#include <map>

namespace freeink::sim::boards {
namespace {

struct Entry {
  const char* name;
  const BoardConfig::BoardProfile* profile;
};

// Every profile the SDK ships, under the name its -DFREEINK_DEVICE_<NAME>
// uses. The two X3 entries are the two panel batches of one device, which the
// SDK fingerprints at runtime; an image cannot be asked, so both are offered.
const Entry* table(size_t* count) {
  static const Entry kEntries[] = {
      {"X3", &BoardConfig::XTEINK_X3},
      {"X3-UC8279", &BoardConfig::XTEINK_X3_UC8279},
      {"X4", &BoardConfig::XTEINK_X4},
      {"X4PRO", &BoardConfig::XTEINK_X4_PRO},
      {"X4CLASSIC", &BoardConfig::XTEINK_X4_CLASSIC},
      {"M5", &BoardConfig::M5STACK_PAPER_COLOR},
      {"MURPHY", &BoardConfig::MURPHY_M3},
      {"MURPHY_M4", &BoardConfig::MURPHY_M4},
      {"DELINK", &BoardConfig::DE_LINK},
      {"LILYGO", &BoardConfig::LILYGO_T5S3},
      {"M5PAPER", &BoardConfig::M5PAPER_V11},
      {"PAPERS3", &BoardConfig::M5PAPER_S3},
      {"PAPERMONO", &BoardConfig::PAPER_MONO},
      {"EEGO_A4", &BoardConfig::EEGO_A4},
      {"STICKY", &BoardConfig::STICKY},
      {"WS397", &BoardConfig::WS_EPAPER_397},
      {"ONEPAGE", &BoardConfig::ONEPAGE},
  };
  *count = sizeof(kEntries) / sizeof(kEntries[0]);
  return kEntries;
}

std::string upper(const std::string& text) {
  std::string out = text;
  for (char& c : out) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  return out;
}

// The descriptors are built once and kept, because `fsim_board_desc::board_name`
// and everything reading them expect a stable address.
const std::map<std::string, fsim_board_desc>& descriptors() {
  static const std::map<std::string, fsim_board_desc> kMap = [] {
    std::map<std::string, fsim_board_desc> map;
    size_t count = 0;
    const Entry* entries = table(&count);
    for (size_t index = 0; index < count; ++index) {
      map.emplace(entries[index].name, describe(*entries[index].profile));
    }
    return map;
  }();
  return kMap;
}

}  // namespace

const fsim_board_desc* byName(const std::string& name) {
  const auto& map = descriptors();
  const auto it = map.find(upper(name));
  return it == map.end() ? nullptr : &it->second;
}

std::vector<std::pair<std::string, const fsim_board_desc*>> all() {
  std::vector<std::pair<std::string, const fsim_board_desc*>> out;
  size_t count = 0;
  const Entry* entries = table(&count);
  for (size_t index = 0; index < count; ++index) {
    const fsim_board_desc* desc = byName(entries[index].name);
    if (desc) out.emplace_back(entries[index].name, desc);
  }
  return out;
}

const fsim_board_desc* defaultForChip(const std::string& chip, std::string* name) {
  // One binary drives the X3 and the X4 on the C3, and the SDK picks between
  // them at runtime; the X4 is the more common unit and the one whose panel
  // geometry the X3 image will correct itself to if it is an X3.
  const char* pick = chip == "esp32s3" ? "X4PRO" : "X4";
  if (name) *name = pick;
  return byName(pick);
}

}  // namespace freeink::sim::boards
