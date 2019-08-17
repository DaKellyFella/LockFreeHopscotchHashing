#pragma once

/*
Hash table benchmarking class.
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

#include "action_generator.h"
#include "benchmark_config.h"
#include "benchmark_results.h"
#include "hash-tables/table_init.h"
#include "primitives/barrier.h"
#include "primitives/cache_utils.h"
#include "thread_papi_wrapper.h"
#include "thread_pinner.h"
#include "utils.h"
#include <atomic>
#include <cstdint>
#include <deque>
#include <pthread.h>
#include <thread>
#include <vector>

namespace concurrent_data_structures {

template <class Random, class Table, class Key, class Allocator>
class TableBenchmark {
private:
  enum class BenchmarkState { RUNNING, STOPPED };

  struct BenchmarkThreadData {
    const std::size_t thread_id;
    std::atomic<BenchmarkState> *state;
    ThreadBarrierWrapper *thread_barrier;
    BenchmarkThreadData(const std::size_t thread_id,
                        std::atomic<BenchmarkState> *state,
                        ThreadBarrierWrapper *thread_barrier)
        : thread_id(thread_id), state(state), thread_barrier(thread_barrier) {}
  };

  const SetBenchmarkConfig m_config;
  SetBenchmarkResult m_results;
  Table *m_table;

  void benchmark_routine(CachePadded<BenchmarkThreadData> *thread_data) {
    SetActionGenerator<Random, Key> action_generator(m_config);
    CachePadded<SetThreadBenchmarkResult> *result =
        m_results.per_thread_benchmark_result[thread_data->thread_id];
    ThreadPapiWrapper papi_wrapper(m_config.base.papi_active);
    bool init = m_table->thread_init(thread_data->thread_id);
    thread_data->thread_barrier->wait();
    assert(init);
    bool papi_start = papi_wrapper.start();
    assert(papi_start);
    while (thread_data->state->load(std::memory_order_relaxed) ==
           BenchmarkState::RUNNING) {
      const SetAction current_action = action_generator.generate_action();
      switch (current_action) {
      case SetAction::Contains: {
        const auto key = action_generator.generate_key();
        result->query_attempts++;
        if (m_table->contains(key, thread_data->thread_id)) {
          result->query_successes++;
        }
      } break;
      case SetAction::Add: {
        auto key = action_generator.generate_key();
        result->addition_attempts++;
        while (!m_table->add(key, thread_data->thread_id)) {
          result->addition_attempts++;
          key = action_generator.generate_key();
        }
        result->addition_successes++;
      } break;
      case SetAction::Remove: {
        result->removal_attempts++;
        auto key = action_generator.generate_key();
        while (!m_table->remove(key, thread_data->thread_id)) {
          result->removal_attempts++;
          key = action_generator.generate_key();
        }
        result->removal_successes++;
      } break;
      }
    }
    bool papi_stop = papi_wrapper.stop(result->papi_counters);
    assert(papi_stop);
  }

public:
  TableBenchmark(const SetBenchmarkConfig &config)
      : m_config(config), m_results(config.base.num_threads) {
    std::cout << "Initialising hash-table." << std::endl;
    m_table = TableInit<Table, Key, Allocator>(config);
    std::cout << "Hash-table initialised." << std::endl;
  }

  ~TableBenchmark() {
    m_table->~Table();
    Allocator::free(m_table);
  }

  SetBenchmarkResult bench() {
    std::cout << "Running maintain benchmark...." << std::endl;
    ThreadBarrierWrapper barrier(m_config.base.num_threads + 1);
    std::atomic<BenchmarkState> benchmark_state{BenchmarkState::RUNNING};
    CachePadded<BenchmarkThreadData> *thread_data =
        static_cast<CachePadded<BenchmarkThreadData> *>(
            Allocator::aligned_alloc(S_CACHE_PADDING,
                                     sizeof(CachePadded<BenchmarkThreadData>) *
                                         m_config.base.num_threads));
    for (std::size_t t = 0; t < m_config.base.num_threads; t++) {
      new (&thread_data[t])
          CachePadded<BenchmarkThreadData>(t, &benchmark_state, &barrier);
    }
    ThreadPinner pinner(m_config.base.hyperthreading);
    std::vector<std::thread *> threads;
    std::cout << "Launching threads." << std::endl;
    for (std::size_t t = 0; t < m_config.base.num_threads; t++) {
      threads.push_back(new std::thread(&TableBenchmark::benchmark_routine,
                                        this, &thread_data[t]));
      bool res = pinner.schedule_thread(threads[t], t);
      assert(res);
    }
    std::cout << "Waiting..." << std::endl;
    // Wait for other threads.
    barrier.wait();
    // Sleep.
    sleep_ignore_signals(m_config.base.duration);
    // End benchmark.
    benchmark_state.store(BenchmarkState::STOPPED);
    std::cout << "Joining threads." << std::endl;
    m_results.scheduling_info = pinner.join();
    for (std::size_t t = 0; t < m_config.base.num_threads; t++) {
      delete threads[t];
    }
    std::cout << "Collating benchmark data." << std::endl;
    return m_results;
  }
};
}
