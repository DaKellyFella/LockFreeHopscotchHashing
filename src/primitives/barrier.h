#pragma once

/*
Simple C++ wrapper around pthread barrier.
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

#include <pthread.h>

namespace concurrent_data_structures {

class ThreadBarrierWrapper {
private:
  pthread_barrier_t m_thread_barrier;

public:
  ThreadBarrierWrapper(const std::size_t num_threads) {
    assert(pthread_barrier_init(&m_thread_barrier, nullptr, num_threads) ==
               0 and
           "Couldn't initialise thread barrier.");
  }
  ~ThreadBarrierWrapper() { pthread_barrier_destroy(&m_thread_barrier); }

  void wait() { pthread_barrier_wait(&m_thread_barrier); }
};
}
