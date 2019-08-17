#pragma once

/*
A number of XORShift RNGs.
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

#include <cstdint>

namespace concurrent_data_structures {

class XORShift32 {
private:
  std::uint32_t m_current;

public:
  static const std::size_t NUM_BITS = 32;

  XORShift32(const std::uint32_t seed) : m_current(seed == 0 ? 1 : seed) {}
  const std::size_t next_rand() {
    m_current ^= m_current << 13;
    m_current ^= m_current >> 17;
    m_current ^= m_current << 5;
    return m_current;
  }
};

class XORShift64 {
private:
  std::uint64_t m_current;

public:
  static const std::size_t NUM_BITS = 64;

  XORShift64(const std::uint64_t seed) : m_current(seed == 0 ? 1 : seed) {}
  const std::size_t next_rand() {
    m_current ^= m_current << 13;
    m_current ^= m_current >> 7;
    m_current ^= m_current << 17;
    return m_current;
  }
};

class XORShift128 {
private:
  std::uint32_t m_current[4];

public:
  static const std::size_t NUM_BITS = 32;

  XORShift128(const std::size_t seed) {
    m_current[0] = (seed == 0 ? 1 : seed);
    m_current[1] = m_current[0];
    m_current[2] = m_current[0];
    m_current[3] = m_current[0];
  }
  const std::uint32_t next_rand() {
    uint32_t s = m_current[3], t = m_current[3];
    t ^= t << 11;
    t ^= t >> 8;
    m_current[3] = m_current[2];
    m_current[2] = m_current[1];
    m_current[1] = m_current[0];
    t ^= s;
    t ^= s >> 19;
    m_current[0] = t;
    return t;
  }
};
}
