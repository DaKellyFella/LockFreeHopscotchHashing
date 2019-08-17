#pragma once

/*
A thread pinning class.
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

#include "cpuinfo.h"
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <deque>
#include <iostream>
#include <map>
#include <pthread.h>
#include <thread>
#include <unordered_map>
#include <vector>

namespace concurrent_data_structures {

class ThreadPinner {
public:
  struct ProcessorInfo {
    std::int32_t user_id, linux_id;
    std::uint32_t L3_cache_id, L2_id, L2_index;
  };

  bool m_hyperthreading_before_socket_switch;
  std::unordered_map<std::thread *, const ProcessorInfo *> m_scheduled;
  std::uint32_t m_index;
  std::vector<const cpuinfo_processor *> m_processors;

public:
  ThreadPinner(bool hyperthreading_before_socket_switch)
      : m_hyperthreading_before_socket_switch(
            hyperthreading_before_socket_switch),
        m_index(0) {
    cpuinfo_initialize();
    std::cout << "Processors..." << std::endl;
    const std::uint32_t num_processors = cpuinfo_get_processors_count();
    m_processors.reserve(num_processors);
    const cpuinfo_processor *proc = cpuinfo_get_processors();
    for (std::uint32_t i = 0; i < num_processors; i++) {
      const cpuinfo_processor *cur_proc = proc + i;
      //      std::cout << "Linux id: " << cur_proc->linux_id << std::endl;
      m_processors[cur_proc->linux_id] = cur_proc;
    }
    std::cout << "Processors... End" << std::endl;
  }

  bool schedule_thread(std::thread *thread, std::int32_t user_id) {

    const cpuinfo_processor *cur_proc = m_processors[m_index++];
    //    std::cout << "Pinning to: " << cur_proc->linux_id << std::endl;
    cpu_set_t cpu_set;
    CPU_ZERO(&cpu_set);
    CPU_SET(cur_proc->linux_id, &cpu_set);
    if (pthread_setaffinity_np(thread->native_handle(), sizeof(cpu_set_t),
                               &cpu_set) != 0) {
      return false;
    }

    ProcessorInfo *proc_info = new ProcessorInfo;
    proc_info->user_id = user_id;
    proc_info->linux_id = cur_proc->linux_id;
    proc_info->L2_id =
        std::distance(cpuinfo_get_l2_caches(), cur_proc->cache.l2);
    proc_info->L2_index =
        cur_proc->linux_id - cur_proc->cache.l2->processor_start;
    proc_info->L3_cache_id =
        std::distance(cpuinfo_get_l3_caches(), cur_proc->cache.l3);
    m_scheduled.insert(std::make_pair(thread, proc_info));
    return true;
  }

  std::vector<ProcessorInfo> join() {
    std::vector<ProcessorInfo> info;
    for (auto it : m_scheduled) {
      it.first->join();
      info.push_back(*it.second);
    }
    std::sort(info.begin(), info.end(),
              [](const ProcessorInfo &lhs, const ProcessorInfo &rhs) {
                if (lhs.L2_id < rhs.L2_id) {
                  return true;
                } else if (lhs.L2_id == rhs.L2_id) {
                  return lhs.L2_index < rhs.L2_index;
                } else {
                  return false;
                }
              });
    return info;
  }

  ~ThreadPinner() { cpuinfo_deinitialize(); }
};
}
