#undef NDEBUG
#include "allocators/glib_allocator.h"
#include "allocators/intel.h"
#include "allocators/jemalloc_allocator.h"
#include "allocators/super_malloc_allocator.h"
#include "bench/arg_parsing.h"
#include "bench/benchmark_config.h"
#include "bench/benchmark_summary.h"
#include "bench/benchmark_table.h"
#include "hash-tables/hs_locked.h"
#include "hash-tables/hs_serial.h"
#include "hash-tables/hs_trans.h"
#include "hash-tables/hsbm_lf.h"
#include "hash-tables/hsbm_lf_compact.h"
#include "hash-tables/hsbm_locked.h"
#include "hash-tables/ph_qp.h"
#include "mem-reclaimer/leaky.h"
#include "random/lcg.h"
#include "random/xorshift.h"
#include <boost/filesystem.hpp>
#include <cassert>
#include <cstdint>
#include <papi.h>
#include <sstream>
#include <string>

using namespace concurrent_data_structures;

namespace {

static const boost::filesystem::path BENCH_PATH =
    boost::filesystem::path("hash_tables");

// https://stackoverflow.com/a/3418285
void replaceAll(std::string &str, const std::string &from,
                const std::string &to) {
  if (from.empty())
    return;
  size_t start_pos = 0;
  while ((start_pos = str.find(from, start_pos)) != std::string::npos) {
    str.replace(start_pos, from.length(), to);
    start_pos += to.length(); // In case 'to' contains 'from', like replacing
                              // 'x' with 'yx'
  }
}
} // namespace

const std::string generate_file_name(const SetBenchmarkConfig &config) {
  std::stringstream file_name;
  file_name << std::string("M:") + get_reclaimer_name(config.base.reclaimer)
            << " A:" << get_allocator_name(config.base.allocator)
            << std::string("R:") + get_generator_name(config.base.generator)
            << " T:" << config.base.num_threads << " S:" << config.table_size
            << " U:" << config.updates << " L:" << config.load_factor
            << std::string(".txt");
  std::string human_file_name = file_name.str();
  replaceAll(human_file_name, " ", "_");
  return human_file_name;
}

// Sick
template <class Random,
          template <class, template <class> class, class...> class Table,
          class Allocator, template <class> class Reclaimer>
bool run_and_save(const SetBenchmarkConfig &config) {
  typedef std::intptr_t Key;
  typedef Table<Allocator, Reclaimer, Key> HashTable;

  const boost::filesystem::path prefix_path =
      config.base.results_directory / BENCH_PATH;
  std::string table_name = get_table_name(config.table);
  replaceAll(table_name, " ", "");
  const boost::filesystem::path table_path = prefix_path / table_name;
  if (!boost::filesystem::exists(table_path)) {
    if (!boost::filesystem::create_directory(table_path)) {
      std::cout << "Failed to create directory: \"" << table_path.string()
                << "\"" << std::endl;
      set_print_help_and_exit();
    }
  }
  TableBenchmark<Random, HashTable, Key, Allocator> benchmark(config);
  produce_summary(config, benchmark.bench(),
                  table_path.string() + "/" + generate_file_name(config),
                  prefix_path.string() + "/" + "set_keys.csv",
                  prefix_path.string() + "/" + "set_results.csv");

  return true;
}

template <class Random, class Allocator, template <class> class Reclaimer>
bool fix_table(const SetBenchmarkConfig &config) {
  switch (config.table) {
  case HashTable::PH_QP_SET:
    return run_and_save<Random, PH_QP_Set, Allocator, Reclaimer>(config);
  case HashTable::HSBM_SERIAL_SET:
    return run_and_save<Random, HopscotchBitMap_Serial_Set, Allocator,
                        Reclaimer>(config);
  case HashTable::HSBM_LF_SET:
    return run_and_save<Random, HopscotchBitMap_LF_Set, Allocator, Reclaimer>(
        config);
  case HashTable::HSC_LOCKED_SET:
    return run_and_save<Random, HopscotchCompact_TicketLock_Set, Allocator,
                        Reclaimer>(config);
  case HashTable::HS_LOCKED_SET:
    return run_and_save<Random, Hopscotch_TicketLock_Set, Allocator, Reclaimer>(
        config);
  case HashTable::HSBM_LOCKED_SET:
    return run_and_save<Random, HopscotchBitMap_SpinLock_Set, Allocator,
                        Reclaimer>(config);
  case HashTable::HS_TRANS_SET:
    return run_and_save<Random, Hopscotch_Transaction_Set, Allocator,
                        Reclaimer>(config);
  default:
    return false;
  }
}

template <class Random, template <class> class Reclaimer>
bool fix_allocator(const SetBenchmarkConfig &config) {
  switch (config.base.allocator) {
  case Allocator::Glibc:
    return fix_table<Random, GlibcAllocator, Reclaimer>(config);
  case Allocator::JeMalloc:
    return fix_table<Random, JeMallocAllocator, Reclaimer>(config);
  case Allocator::Intel:
    return fix_table<Random, IntelAllocator, Reclaimer>(config);
  default:
    return false;
  }
}

template <class Random> bool fix_reclaimer(const SetBenchmarkConfig &config) {
  return fix_allocator<Random, LeakyReclaimer>(config);
}

bool run(const SetBenchmarkConfig &config) {
  switch (config.base.generator) {
  case RandomGenerator::LCGBSD:
    return fix_reclaimer<LCGBSD>(config);
  case RandomGenerator::PCG:
    return fix_reclaimer<PCG>(config);
  case RandomGenerator::XOR_SHIFT_32:
    return fix_reclaimer<XORShift32>(config);
  case RandomGenerator::XOR_SHIFT_64:
    return fix_reclaimer<XORShift64>(config);
  default:
    return false;
  }
}

int main(int argc, char *argv[]) {
  const SetBenchmarkConfig config = parse_set_args(argc, argv);
  if (config.base.papi_active) {
    assert(PAPI_library_init(PAPI_VER_CURRENT) == PAPI_VER_CURRENT and
           "Couldn't initialise PAPI library. Check installation.");
    int res = PAPI_thread_init(pthread_self);
    assert(res == PAPI_OK);
  }
  config.print(std::cout);
  if (!boost::filesystem::exists(config.base.results_directory)) {
    if (!boost::filesystem::create_directory(config.base.results_directory)) {
      std::cout << "Failed to create directory: \""
                << config.base.results_directory.string() << "\"" << std::endl;
      set_print_help_and_exit();
    }
  }
  if (!boost::filesystem::exists(config.base.results_directory / BENCH_PATH)) {
    if (!boost::filesystem::create_directory(config.base.results_directory /
                                             BENCH_PATH)) {
      std::cout << "Failed to create directory: \""
                << (config.base.results_directory.string() +
                    BENCH_PATH.string())
                << "\"" << std::endl;
      set_print_help_and_exit();
    }
  }
  if (!run(config)) {
    set_print_help_and_exit();
  }
  std::cout << "Finished." << std::endl;

  return 0;
}
