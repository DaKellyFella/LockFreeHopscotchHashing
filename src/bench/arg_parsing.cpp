#include "arg_parsing.h"

/*
Body for parsing args to main.
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
#include "random.h"
#include "table.h"
#include <functional>
#include <iostream>

namespace concurrent_data_structures {

namespace {
// Table_Sync_(Opti/Method)_(Set/Map)
static const std::map<std::string, HashTable> table_map{
    // Hopscotch Hashing
    std::make_pair("hsbm_serial_set", HashTable::HSBM_SERIAL_SET),
    std::make_pair("hs_trans_set", HashTable::HS_TRANS_SET),
    std::make_pair("hs_locked_set", HashTable::HS_LOCKED_SET),
    std::make_pair("hsc_locked_set", HashTable::HSC_LOCKED_SET),
    std::make_pair("hsbm_locked_set", HashTable::HSBM_LOCKED_SET),
    std::make_pair("hsbm_lf_set", HashTable::HSBM_LF_SET),
    // Purcell Harris
    std::make_pair("ph_qp_set", HashTable::PH_QP_SET),
};

static const std::map<std::string, RandomGenerator> generator_map{
    std::make_pair("lcg_bsd", RandomGenerator::LCGBSD),
    std::make_pair("xor_shift_32", RandomGenerator::XOR_SHIFT_32),
    std::make_pair("xor_shift_64", RandomGenerator::XOR_SHIFT_64),
    std::make_pair("pcg", RandomGenerator::PCG),
};

static const std::map<std::string, Reclaimer> reclaimer_map{
    std::make_pair("leaky", Reclaimer::Leaky),
};

static const std::map<std::string, Allocator> allocator_map{
    std::make_pair("je", Allocator::JeMalloc),
    std::make_pair("glibc", Allocator::Glibc),
    std::make_pair("intel", Allocator::Intel),
};

static std::function<void()> S_PRINT_AND_EXIT_FUNC;
}

bool parse_base_arg(BenchmarkConfig &base, const int current_option,
                    char *arg) {
  switch (current_option) {
  case 'R': {
    auto random_res = generator_map.find(std::string(arg));
    if (generator_map.end() == random_res) {
      std::cout << "Invalid benchmark choice." << std::endl;
      S_PRINT_AND_EXIT_FUNC();
    } else {
      base.generator = random_res->second;
    }
    return true;
  }
  case 'D':
    base.duration = std::chrono::seconds(std::atoi(arg));
    return true;
  case 'T':
    base.num_threads = std::size_t(std::atoi(arg));
    return true;
  case 'M': {
    auto reclaimer_res = reclaimer_map.find(std::string(arg));
    if (reclaimer_map.end() == reclaimer_res) {
      std::cout << "Invalid reclaimer choice." << std::endl;
      S_PRINT_AND_EXIT_FUNC();
    } else {
      base.reclaimer = reclaimer_res->second;
    }
    return true;
  }
  case 'A': {
    auto allocator_res = allocator_map.find(std::string(arg));
    if (allocator_map.end() == allocator_res) {
      std::cout << "Invalid allocator choice." << std::endl;
      S_PRINT_AND_EXIT_FUNC();
    } else {
      base.allocator = allocator_res->second;
    }
    return true;
  }
  case 'P':
    base.papi_active = std::string(optarg) == "true";
    return true;
  case 'H':
    base.hyperthreading = std::string(optarg) == "true";
    return true;
  case 'N':
    base.results_directory = std::string(optarg);
    return true;
  }
  return false;
}

SetBenchmarkConfig parse_set_args(std::int32_t argc, char *argv[]) {
  S_PRINT_AND_EXIT_FUNC = []() { set_print_help_and_exit(); };
  SetBenchmarkConfig config = {
      BenchmarkConfig{1, std::chrono::seconds(1),
                      boost::filesystem::path{"results"},
                      RandomGenerator::XOR_SHIFT_64, Reclaimer::Leaky,
                      Allocator::JeMalloc, true, false},
      1 << 23, 10, 0.5, HashTable::HSBM_LF_SET};
  int current_option;
  while ((current_option = getopt(argc, argv, ":R:L:S:D:T:U:B:M:P:V:A:H:N:")) !=
         -1) {
    if (parse_base_arg(config.base, current_option, optarg)) {
      continue;
    }
    switch (current_option) {
    case 'L':
      config.load_factor = std::stod(optarg);
      break;
    case 'S':
      config.table_size = std::size_t(1 << std::atoi(optarg));
      break;
    case 'U':
      config.updates = std::size_t(std::atoi(optarg));
      break;
    case 'B': { // C++
      auto table_res = table_map.find(std::string(optarg));
      if (table_map.end() == table_res) {
        std::cout << "Invalid benchmark choice." << std::endl;
        S_PRINT_AND_EXIT_FUNC();
      } else {
        config.table = table_res->second;
      }
    } break;
    case ':':
    default:
      S_PRINT_AND_EXIT_FUNC();
    }
  }
  return config;
}

void base_print_help() {
  std::cout
      << "D: Duration of benchmark in seconds. Default = 1 second.\n"
      << "T: Number of concurrent threads. Default = 1.\n"
      << "M: Memory reclaimer using within table (if needed). Default = None.\n"
      << "A: Allocator used within the table. Default = JeMalloc.\n"
      << "R: Random number generator used during the benchmark. Default = XOR "
         "Shift 64.\n"
      << "P: Whether PAPI is turned on or not. Default = True.\n"
      << "H: Whether to schedule threads with a preference for "
         "Hyperthreading. Default = False.\n"
      << "N: Name of results directory. Default = results" << std::endl;
}

void set_print_help_and_exit() {
  base_print_help();
  std::cout << "L: Load Factor. Default = 50%.\n"
            << "S: Power of two size. Default = 1 << 23.\n"
            << "U: Updates as a percentage of workload. Default = 10%.\n"
            << "B: Table being benchmarked. Default = kcas_rh_set.\n"
            << std::endl;
  exit(0);
}
}
