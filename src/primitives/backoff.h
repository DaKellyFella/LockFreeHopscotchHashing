#pragma once

/*
An implementation of backoff algorithms.
Copyright (C) 2018  Robert Kelly
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "random/xorshift.h"
#include <cstdint>

namespace concurrent_data_structures {

class ExponentialBackoff {
private:
  static const std::size_t S_MAX_COUNT = 10;
  static const std::size_t S_SLEEP_EPOCH = 5;
  XORShift64 m_sleep_generator;
  std::size_t m_collision_count;

public:
  ExponentialBackoff();
  void reset();
  void backoff();
};
}
