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

#include <chrono>
#include <cstdint>
#include <random>
#include <thread>

#include "backoff.h"

namespace concurrent_data_structures {

ExponentialBackoff::ExponentialBackoff()
    : m_sleep_generator(1), m_collision_count(0) {}

void ExponentialBackoff::reset() { m_collision_count = 0; }
void ExponentialBackoff::backoff() {
  const std::size_t sleep_amount =
      m_sleep_generator.next_rand() &
      ((std::size_t(1) << m_collision_count++) - 1);

  if (sleep_amount != 0) {
    std::this_thread::sleep_for(
        std::chrono::microseconds(S_SLEEP_EPOCH * sleep_amount));
  }
}
}
