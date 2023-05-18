// Copyright (c) 2022 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <cstdint>

namespace carla {
namespace multigpu {

enum MultiGPUCommand : uint32_t {
  SEND_FRAME = 0,
  LOAD_MAP,
  GET_TOKEN,
  YOU_ALIVE
};

struct CommandHeader {
  MultiGPUCommand id;
  uint32_t size;
};

} // namespace multigpu
} // namespace carla
