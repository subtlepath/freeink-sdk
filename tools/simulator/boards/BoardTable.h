#pragma once

// FreeInk simulator — the boards a device image can be run on.
//
// A firmware *bundle* reports its own board: it was compiled with one, and its
// entry stub hands BoardConfig::ACTIVE straight to the daemon. A device image
// cannot — it is a `.bin` with no profile in it, and the emulator models the
// chip rather than the board. Without being told, it can run the firmware's
// instructions and nothing else: no panel, no buttons, no I2C.
//
// So the simulator carries the SDK's own board table, generated from the same
// BoardConfig the SDK compiles against, and an image is loaded *onto a board*.
// The default is inferred from the chip the image names, and overridden with
// `--device`.

#include <freeink_sim_abi.h>

#include <string>
#include <vector>

namespace freeink::sim::boards {

// The profile registered under `name` (case-insensitive, matching the SDK's
// -DFREEINK_DEVICE_<NAME> spelling), or nullptr.
const fsim_board_desc* byName(const std::string& name);

// Every board the table knows, in the order they should be listed.
std::vector<std::pair<std::string, const fsim_board_desc*>> all();

// The board to assume for an image built for `chip` ("esp32c3", "esp32s3", …)
// when the operator did not say. Both are a guess, and both are the right one
// for the devices this SDK exists for — but a guess is still a guess, so the
// caller says which board it picked rather than letting it pass silently.
const fsim_board_desc* defaultForChip(const std::string& chip, std::string* name);

}  // namespace freeink::sim::boards
