#pragma once

/*
Enums for reclaimers used.
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

#include <cstdlib>
#include <string>

namespace concurrent_data_structures {

enum class Reclaimer {
  Leaky,
};

const std::string get_reclaimer_name(const Reclaimer reclaimer);

template <class Allocator> class ReclaimerAllocator {
public:
  void *malloc(size_t size) { return Allocator::malloc(size); }
  void free(void *ptr) { Allocator::free(ptr); }
  size_t malloc_usable_size(void *ptr) {
    return Allocator::malloc_usable_size(ptr);
  }
};

template <class MemReclaimer> class ReclaimerPin {
private:
  typedef typename MemReclaimer::RecordHandle RecordHandle;
  typedef typename MemReclaimer::RecordArrayHandle RecordArrayHandle;

  MemReclaimer *m_reclaimer;
  std::size_t m_thread_id;

public:
  ReclaimerPin(MemReclaimer *reclaimer, std::size_t thread_id)
      : m_reclaimer(reclaimer), m_thread_id(thread_id) {
    m_reclaimer->enter(m_thread_id);
  }
  ~ReclaimerPin() { m_reclaimer->exit(m_thread_id); }

  RecordHandle get_rec() { return m_reclaimer->get_rec(m_thread_id); }
  RecordArrayHandle get_bulk_rec(const std::size_t size) {
    return m_reclaimer->get_bulk_rec(m_thread_id, size);
  }

  void retire(const RecordHandle &handle) {
    m_reclaimer->retire(handle, m_thread_id);
  }

  void retire(const RecordArrayHandle &handle, const std::size_t index) {
    m_reclaimer->retire(handle, m_thread_id, index);
  }
};
}
