#pragma once

/*
Generates random actions for benchmarking tools.
Copyright (C) 2018 Robert Kelly

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "benchmark_config.h"
#include "random/lcg.h"
#include "random/pcg_random.h"
#include "random/xorshift.h"
#include <iostream>
#include <random>

namespace concurrent_data_structures {

enum class SetAction {
  Contains,
  Add,
  Remove,
};

template <class Random, class Key> class SetActionGenerator {
private:
  Random m_action_generator;
  Random m_key_generator;
  const std::uintptr_t m_key_mask;
  const std::uintptr_t m_read_limit, m_add_limit;

public:
  SetActionGenerator(const SetBenchmarkConfig &config)
      : m_action_generator(std::random_device{}()),
        m_key_generator(std::random_device{}()),
        m_key_mask((config.table_size) - 1),
        m_read_limit((m_key_mask) -
                     (static_cast<double>(m_key_mask) *
                      static_cast<double>(config.updates) / 100.0)),
        m_add_limit(m_read_limit +
                    (static_cast<double>(m_key_mask) *
                     static_cast<double>(config.updates) / 200.0)) {}
  SetAction generate_action() {
    const std::uintptr_t action = m_action_generator.next_rand() & m_key_mask;
    if (action <= m_read_limit) {
      return SetAction::Contains;
    } else if (action <= m_add_limit) {
      return SetAction::Add;
    } else {
      return SetAction::Remove;
    }
  }
  Key generate_key() {
    const Key key = m_key_generator.next_rand() & m_key_mask;
    return key;
  }
};
}
