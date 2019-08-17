#include "benchmark_summary.h"

/*
Summarises benchmark data into files.
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

#include "mem-reclaimer/reclaimer.h"
#include "random.h"
#include "table.h"
#include <algorithm>
#include <fstream>
#include <sstream>

namespace concurrent_data_structures {

template <class T>
void write_field(std::ofstream &human_file, std::ofstream &csv_key_file,
                 std::ofstream &csv_data_file, const std::string &field_name,
                 const T &field_value, bool write_keys, bool last = false,
                 bool last_key = false) {
  human_file << "(" << field_name << " -> " << field_value
             << (last ? ")" : ") ");
  if (write_keys) {
    csv_key_file << field_name << (last_key ? "" : ",");
  }
  csv_data_file << field_value << (last ? "" : ",");
}

void config_summary(const BenchmarkConfig &config, std::ofstream &human_file,
                    std::ofstream &csv_key_file, std::ofstream &csv_data_file,
                    bool write_keys) {
  write_field(human_file, csv_key_file, csv_data_file, "Num Threads",
              config.num_threads, write_keys);
  write_field(human_file, csv_key_file, csv_data_file, "Duration",
              config.duration.count(), write_keys);
  write_field(human_file, csv_key_file, csv_data_file, "Reclaimer Name",
              get_reclaimer_name(config.reclaimer), write_keys);
  write_field(human_file, csv_key_file, csv_data_file, "Allocator Name",
              get_allocator_name(config.allocator), write_keys);
  write_field(human_file, csv_key_file, csv_data_file, "Generator Name",
              get_generator_name(config.generator), write_keys);
  write_field(human_file, csv_key_file, csv_data_file, "Papi Enabled",
              config.papi_active ? "True" : "False", write_keys);
  write_field(human_file, csv_key_file, csv_data_file, "Hyperthreading Enabled",
              config.hyperthreading ? "True" : "False", write_keys);
}

void set_config_summary(const SetBenchmarkConfig &config,
                        std::ofstream &human_file, std::ofstream &csv_key_file,
                        std::ofstream &csv_data_file, bool write_keys) {
  write_field(human_file, csv_key_file, csv_data_file, "Table Name",
              get_table_name(config.table), write_keys);
  write_field(human_file, csv_key_file, csv_data_file, "Table Size",
              config.table_size, write_keys);
  write_field(human_file, csv_key_file, csv_data_file, "Load Factor",
              config.load_factor, write_keys);
  write_field(human_file, csv_key_file, csv_data_file, "Updates",
              config.updates, write_keys);
  config_summary(config.base, human_file, csv_key_file, csv_data_file,
                 write_keys);
}

// static const double milliseconds = 1000.0;
static const double microseconds = 1000000.0;

void papi_summary(const std::size_t total_operations,
                  const PapiCounters &papi_counters, std::ofstream &human_file,
                  std::ofstream &csv_key_file, std::ofstream &csv_data_file,
                  bool write_keys) {

  const double total_ops = static_cast<double>(total_operations);
  const double level1_cache_misses_per_op =
      static_cast<double>(
          papi_counters.counters[PAPI_EVENTS::L1_CACHE_MISSES]) /
      total_ops;

  const double level2_cache_misses_per_op =
      static_cast<double>(
          papi_counters.counters[PAPI_EVENTS::L2_CACHE_MISSES]) /
      total_ops;

  const double instruction_stalls_per_op =
      static_cast<double>(
          papi_counters.counters[PAPI_EVENTS::INSTRUCTION_STALLS]) /
      total_ops;

  const double total_instructions_per_op =
      static_cast<double>(
          papi_counters.counters[PAPI_EVENTS::TOTAL_INSTRUCTIONS]) /
      total_ops;

  const double level1_data_cache_misses_per_op =
      static_cast<double>(
          papi_counters.counters[PAPI_EVENTS::L1_DATA_CACHE_MISSES]) /
      total_ops;

  write_field(human_file, csv_key_file, csv_data_file,
              PAPI_EVENTS::get_event_name(PAPI_EVENTS::L1_DATA_CACHE_MISSES),
              level1_data_cache_misses_per_op, write_keys);
  write_field(human_file, csv_key_file, csv_data_file,
              PAPI_EVENTS::get_event_name(PAPI_EVENTS::L1_CACHE_MISSES),
              level1_cache_misses_per_op, write_keys);
  write_field(human_file, csv_key_file, csv_data_file,
              PAPI_EVENTS::get_event_name(PAPI_EVENTS::L2_CACHE_MISSES),
              level2_cache_misses_per_op, write_keys);
  write_field(human_file, csv_key_file, csv_data_file,
              PAPI_EVENTS::get_event_name(PAPI_EVENTS::INSTRUCTION_STALLS),
              instruction_stalls_per_op, write_keys);
  write_field(human_file, csv_key_file, csv_data_file,
              PAPI_EVENTS::get_event_name(PAPI_EVENTS::TOTAL_INSTRUCTIONS),
              total_instructions_per_op, write_keys);
}

void operations_per_thread_summary(const SetThreadBenchmarkResult &result,
                                   std::ofstream &human_file,
                                   std::ofstream &csv_key_file,
                                   std::ofstream &csv_data_file,
                                   const std::chrono::seconds &duration,
                                   bool print_papi, bool write_keys,
                                   bool last = false) {
  const std::size_t total_operations_attempted = result.query_attempts +
                                                 result.addition_attempts +
                                                 result.removal_attempts;

  if (print_papi) {
    papi_summary(total_operations_attempted, result.papi_counters, human_file,
                 csv_key_file, csv_data_file, write_keys);
    human_file << std::endl;
  }

  const double attempted_ops_per_second =
      static_cast<double>(total_operations_attempted) /
      static_cast<double>(duration.count());
  const double attempted_ops_per_microsecond =
      attempted_ops_per_second / microseconds;
  // Operations as a percentage of the work done.
  const double query_attempt_percentage =
      (static_cast<double>(result.query_attempts) /
       static_cast<double>(total_operations_attempted)) *
      100.0;
  const double insertion_attempt_percentage =
      (static_cast<double>(result.addition_attempts) /
       static_cast<double>(total_operations_attempted)) *
      100.0;
  const double removal_attempt_percentage =
      (static_cast<double>(result.removal_attempts) /
       static_cast<double>(total_operations_attempted)) *
      100.0;

  const double query_successes_percentage =
      (static_cast<double>(result.query_successes) /
       static_cast<double>(std::max(result.query_attempts, std::uint64_t(1)))) *
      100.0;
  const double insertion_successes_percentage =
      (static_cast<double>(result.addition_successes) /
       static_cast<double>(
           std::max(result.addition_attempts, std::uint64_t(1)))) *
      100.0;
  const double removal_successes_percentage =
      (static_cast<double>(result.removal_successes) /
       static_cast<double>(
           std::max(result.removal_attempts, std::uint64_t(1)))) *
      100.0;

  write_field(human_file, csv_key_file, csv_data_file, "Queries attempted",
              result.query_attempts, write_keys);
  human_file << "Queries attempted as percentage: " << query_attempt_percentage
             << "%" << std::endl;
  write_field(human_file, csv_key_file, csv_data_file, "Queries succeeded",
              result.query_successes, write_keys);
  human_file << "Queries succeeded as percentage: "
             << query_successes_percentage << "%" << std::endl;
  write_field(human_file, csv_key_file, csv_data_file, "Insertion attempted",
              result.addition_attempts, write_keys);
  human_file << "Insertions attempted as percentage: "
             << insertion_attempt_percentage << "%" << std::endl;
  write_field(human_file, csv_key_file, csv_data_file, "Insertion succeeded",
              result.addition_successes, write_keys);
  human_file << "Insertions succeeded as percentage: "
             << insertion_successes_percentage << "%" << std::endl;
  write_field(human_file, csv_key_file, csv_data_file, "Removes attempted",
              result.removal_attempts, write_keys);
  human_file << "Removes attempted as percentage: "
             << removal_attempt_percentage << "%" << std::endl;
  write_field(human_file, csv_key_file, csv_data_file, "Removes successes",
              result.removal_successes, write_keys);
  human_file << "Removes succeeded as percentage: "
             << removal_successes_percentage << "%" << std::endl;
  write_field(human_file, csv_key_file, csv_data_file,
              "Total Ops per microsecond", attempted_ops_per_microsecond,
              write_keys, last, true);
}

template <class Result>
void operations_summary(const Result &result, std::ofstream &human_file,
                        std::ofstream &csv_key_file,
                        std::ofstream &csv_data_file,
                        const std::chrono::seconds &duration, bool papi) {
  human_file << "TOTAL OPERATIONS FOR ALL THREADS" << std::endl;
  operations_per_thread_summary(result.collate_results(), human_file,
                                csv_key_file, csv_data_file, duration, papi,
                                true);
  human_file << std::endl;
  human_file << "OPERATIONS INDIVIDUAL THREADS" << std::endl;
  for (std::size_t i = 0; i < result.num_threads; i++) {
    human_file << std::string(40, '*') << std::endl;
    human_file << "THREAD: " << result.scheduling_info[i].user_id
               << " operation summary." << std::endl;
    human_file << "Thread was pinned to core id: "
               << result.scheduling_info[i].linux_id << " on L3 cache index: "
               << result.scheduling_info[i].L3_cache_id
               << " with an L2 ID of: " << result.scheduling_info[i].L2_id
               << " and with L2 index of: "
               << result.scheduling_info[i].L2_index << std::endl;
    operations_per_thread_summary(
        *result.per_thread_benchmark_result[result.scheduling_info[i].user_id],
        human_file, csv_key_file, csv_data_file, duration, papi, false,
        (i == (result.num_threads - 1)));
    human_file << std::endl;
  }
}

void produce_summary(const SetBenchmarkConfig &config,
                     const SetBenchmarkResult &result,
                     const std::string &human_filename,
                     const std::string &csv_key_filename,
                     const std::string &csv_data_filename) {
  std::ofstream human_file(human_filename);
  std::ofstream csv_key_file(csv_key_filename,
                             std::ofstream::out | std::ofstream::trunc);
  std::ofstream csv_data_file(csv_data_filename,
                              std::ofstream::out | std::ofstream::app);
  human_file << "CONFIG." << std::endl;
  human_file << std::string(40, '*') << std::endl;
  // Setup.
  set_config_summary(config, human_file, csv_key_file, csv_data_file, true);
  human_file << std::endl;
  // Numbers produced.
  human_file << std::string(40, '*') << std::endl;
  human_file << "OPERATIONS." << std::endl;
  operations_summary(result, human_file, csv_key_file, csv_data_file,
                     config.base.duration, config.base.papi_active);
  csv_key_file << std::endl;
  csv_data_file << std::endl;
}

} // namespace concurrent_hash_tables
