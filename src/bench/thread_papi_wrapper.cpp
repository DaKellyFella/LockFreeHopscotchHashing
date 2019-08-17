#include "thread_papi_wrapper.h"

/*
Main body of per-thread PAPI wrapper.
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

#include <cassert>
#include <cstdio>
#include <ctime>
#include <iostream>
#include <map>
#include <papi.h>
#include <string>
#include <unistd.h>

namespace concurrent_data_structures {

namespace {
std::map<PAPI_EVENTS::EVENTS, std::string> event_map = {
    std::make_pair(PAPI_EVENTS::L1_CACHE_MISSES, "Level 1 Cache Misses"),
    std::make_pair(PAPI_EVENTS::L2_CACHE_MISSES, "Level 2 Cache Misses"),
    std::make_pair(PAPI_EVENTS::INSTRUCTION_STALLS, "Instruction Stalls"),
    std::make_pair(PAPI_EVENTS::TOTAL_INSTRUCTIONS, "Total Instructions"),
    std::make_pair(PAPI_EVENTS::L1_DATA_CACHE_MISSES,
                   "Level 1 Data Cache Misses"),
};
}

const std::string PAPI_EVENTS::get_event_name(EVENTS event) {
  std::string event_name = "ERROR: Incorrect event name.";
  auto it = event_map.find(event);
  if (it != event_map.end()) {
    event_name = it->second;
  }
  return event_name;
}

static int PAPI_events[PAPI_EVENTS::TOTAL_PAPI_EVENTS] = {
    PAPI_L1_TCM, PAPI_L2_TCM, PAPI_STL_ICY, PAPI_TOT_INS,
    PAPI_L1_DCM /*PAPI_TOT_CYC*/};

PapiCounters::PapiCounters() {
  for (std::size_t i = 0; i < PAPI_EVENTS::TOTAL_PAPI_EVENTS; i++) {
    counters[i] = 0;
  }
}

ThreadPapiWrapper::ThreadPapiWrapper(bool active) : m_active(active) {
  if (active) {
    int res = PAPI_register_thread();
    assert(res == PAPI_OK);
  }
}

bool ThreadPapiWrapper::start() {
  if (m_active) {
    int res = PAPI_start_counters(PAPI_events, PAPI_EVENTS::TOTAL_PAPI_EVENTS);
    if (res != PAPI_OK) {
      std::cout << res << std::endl;
      return false;
    }
    return true;
  } else {
    return true;
  }
}

bool ThreadPapiWrapper::stop(PapiCounters &counters) {
  if (m_active) {
    return PAPI_stop_counters(counters.counters,
                              PAPI_EVENTS::TOTAL_PAPI_EVENTS) == PAPI_OK;
  } else {
    return true;
  }
}
}
