#pragma once

/*
Configuration class for benchmarking.
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

#include "allocators.h"
#include "mem-reclaimer/reclaimer.h"
#include "random.h"
#include "table.h"
#include <boost/filesystem.hpp>
#include <chrono>
#include <cstdint>
#include <ostream>

namespace concurrent_data_structures {

struct BenchmarkConfig {
  std::size_t num_threads;
  std::chrono::seconds duration;
  boost::filesystem::path results_directory;
  RandomGenerator generator;
  Reclaimer reclaimer;
  Allocator allocator;
  bool papi_active, hyperthreading;
  void print(std::ostream &os) const;
};

struct SetBenchmarkConfig {
  BenchmarkConfig base;
  std::size_t table_size, updates;
  double load_factor;
  HashTable table;
  void print(std::ostream &os) const;
};
}
