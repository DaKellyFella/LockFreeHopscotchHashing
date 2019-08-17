#pragma once

/*
"Leaky" reclaimer. Does nothing but fit interface for reclaimers.
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

#include "mem-reclaimer/reclaimer.h"
#include "primitives/marked_pointers.h"
#include <atomic>

namespace concurrent_data_structures {

template <class Allocator>
class LeakyReclaimer : public ReclaimerAllocator<Allocator> {
public:
  class LeakyBase {};

  class LeakyHandle {
  public:
    LeakyHandle() {}
    ~LeakyHandle() {}
    LeakyHandle(const LeakyHandle &rhs) = delete;
    LeakyHandle &operator=(const LeakyHandle &rhs) = delete;
    LeakyHandle(LeakyHandle &&rhs) {}
    LeakyHandle &operator=(LeakyHandle &&rhs) { return *this; }

    void set(const LeakyBase *ptr) {}

    template <class PtrType, class AtomicSourceType, typename Func>
    bool try_protect(PtrType &ptr, const AtomicSourceType &src, Func f) {
      return true;
    }
    template <class PtrType, class AtomicSourceType>
    bool try_protect(PtrType &ptr, const AtomicSourceType &src) noexcept {
      return this->try_protect(ptr, src, [](PtrType ptr) { return ptr; });
    }

    template <class PtrType, class AtomicSourceType, typename Func>
    PtrType get_protected(const AtomicSourceType &src, Func f) {}
    template <class PtrType, class AtomicSourceType>
    PtrType get_protected(const AtomicSourceType &src) noexcept {
      return this->get_protected(src, [](PtrType ptr) { return ptr; });
    }
    friend class LeakyReclaimer;
  };

  class LeakyArrayHandle {
  public:
    LeakyArrayHandle() {}
    ~LeakyArrayHandle() {}
    LeakyArrayHandle(const LeakyArrayHandle &rhs) = delete;
    LeakyArrayHandle &operator=(const LeakyArrayHandle &rhs) = delete;
    LeakyArrayHandle(LeakyArrayHandle &&rhs) {}
    LeakyArrayHandle &operator=(LeakyArrayHandle &&rhs) { return *this; }

    void set(const std::size_t index, LeakyBase *ptr) {}

    template <class Inner, typename Func>
    bool try_protect(const std::size_t index, LeakyBase *ptr,
                     const std::atomic<Inner> &src, Func f) {
      return true;
    }

    template <class Inner, typename Func>
    bool try_protect(const std::size_t index, MarkedPointer<Inner> ptr,
                     const AtomicMarkedPointer<Inner> &src, Func f) {
      return true;
    }

    template <class Inner>
    bool try_protect(const std::size_t index, LeakyBase *ptr,
                     const std::atomic<Inner> &src) {
      return this->try_protect(index, ptr, src,
                               [](LeakyBase *ptr) { return ptr; });
    }

    template <class Inner>
    bool try_protect(const std::size_t index, MarkedPointer<Inner> ptr,
                     const AtomicMarkedPointer<Inner> &src) {
      return this->try_protect(index, ptr, src,
                               [](MarkedPointer<Inner> ptr) { return ptr; });
    }

    template <class PtrType, class SourceType, typename Func>
    PtrType get_protected(const std::size_t index,
                          const std::atomic<SourceType> &src, Func f) {
      PtrType ptr = f(src.load());
      while (!try_protect(index, ptr, src, f)) {
        ptr = f(src.load());
      }
      return ptr;
    }
    template <class PtrType>
    PtrType get_protected(const std::size_t index,
                          const std::atomic<PtrType> &src) noexcept {
      return this->get_protected(index, src, [](PtrType ptr) { return ptr; });
    }
    friend class LeakyReclaimer;
  };

  typedef LeakyBase RecordBase;
  typedef LeakyHandle RecordHandle;
  typedef LeakyArrayHandle RecordArrayHandle;

  LeakyReclaimer(const std::size_t num_threads,
                 const std::size_t refs_per_thread) {}
  ~LeakyReclaimer() {}
  bool thread_init(const std::size_t thread_id) { return true; }
  void enter(const std::size_t thread_id) {}
  void exit(const std::size_t thread_id) {}
  LeakyHandle get_rec(const std::size_t thread_id) { return LeakyHandle(); }
  LeakyArrayHandle get_bulk_rec(const std::size_t thread_id,
                                const std::size_t size) {
    return LeakyArrayHandle();
  }
  void retire(const RecordHandle &handle, const std::size_t thread_id) {}
  void retire(const RecordArrayHandle &handle, const std::size_t thread_id,
              const std::size_t index) {}
};
}
