#pragma once

/*
Simple class to initialise a hash table.
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

#include "bench/benchmark_config.h"
#include "primitives/cache_utils.h"
#include "random/pcg_random.h"
#include <algorithm>
#include <random>

namespace concurrent_data_structures {

template <class Table, class Key, class Allocator>
static Table *TableInit(const SetBenchmarkConfig &config) {
  Table *table = new (Allocator::aligned_alloc(S_CACHE_PADDING, sizeof(Table)))
      Table(config.table_size, config.base.num_threads);
  std::size_t amount =
      static_cast<std::size_t>(config.table_size * config.load_factor);
  std::vector<Key> keys(config.table_size);
  for (std::size_t i = 0; i < keys.size(); i++) {
    keys[i] = i;
  }
  pcg32 random = pcg_extras::seed_seq_from<std::random_device>();
  std::shuffle(keys.begin(), keys.end(), random);
  std::size_t total = 0;
  for (std::size_t i = 0; total < amount; i++) {
    if (table->add(keys[i], i % config.base.num_threads)) {
      total++;
    }
  }
  if (total < amount) {
    assert(false);
  }
  return table;
}
}
