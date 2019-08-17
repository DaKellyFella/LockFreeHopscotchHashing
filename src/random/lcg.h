#pragma once

/*
A number of LCG RNGs
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

#include "pcg_random.h"
#include <cstdint>
#include <random>

namespace concurrent_data_structures {

// class LCG {
// private:
//  const static std::size_t S_MODULUS_MASK = (std::size_t(1) << 31) - 1;
//  //  const static std::size_t S_MODULUS_MASK = (std::size_t(1) << 48) - 1;
//  //  const static std::size_t S_MULTIPLIER = std::size_t(1103515245);
//  //  const static std::size_t S_MULTIPLIER = std::size_t(25214903917);
//  const static std::size_t S_MULTIPLIER = std::size_t(1103515245);
//  //  const static std::size_t S_INCREMENT = std::size_t(12345);
//  //  const static std::size_t S_INCREMENT = std::size_t(11);
//  const static std::size_t S_INCREMENT = std::size_t(12345);
//  std::size_t m_current;

// public:
//  static const std::size_t NUM_BITS = 32;

//  LCG() : m_current(0) {}
//  LCG(const std::size_t seed) : m_current(seed & S_MODULUS_MASK) {}
//  const std::size_t next_rand() {
//    m_current = ((S_MULTIPLIER * m_current) + S_INCREMENT) & S_MODULUS_MASK;
//    return m_current >> 16;
//  }
//};

class PCG {
private:
  pcg32 m_random_generator;
  std::uniform_int_distribution<std::size_t> m_data_distribution;

public:
  static const std::size_t NUM_BITS = 32;

  PCG()
      : m_random_generator(pcg_extras::seed_seq_from<std::random_device>()),
        m_data_distribution(0, std::size_t(1) << NUM_BITS) {}
  PCG(const std::size_t seed)
      : m_random_generator(seed),
        m_data_distribution(0, std::size_t(1) << NUM_BITS) {}
  const std::size_t next_rand() {
    return m_data_distribution(m_random_generator);
  }
};

class LCGBSD {
private:
  const static std::size_t S_MODULUS_MASK = (std::size_t(1) << 31) - 1;
  const static std::size_t S_MULTIPLIER = std::size_t(1103515245);
  const static std::size_t S_INCREMENT = std::size_t(12345);
  std::size_t m_current;

public:
  static const std::size_t NUM_BITS = 31;

  LCGBSD() : m_current(0) {}
  LCGBSD(const std::size_t seed) : m_current(seed & S_MODULUS_MASK) {}
  const std::size_t next_rand() {
    m_current = ((S_MULTIPLIER * m_current) + S_INCREMENT) & S_MODULUS_MASK;
    return m_current;
  }
};
}
