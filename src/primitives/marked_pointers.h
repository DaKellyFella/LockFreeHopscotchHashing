#pragma once

/*
Marked pointer types for ease of concurrent programming.
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

#include <atomic>
#include <cstdint>

namespace concurrent_data_structures {

template <class Type> class MarkedPointer {
private:
  static const std::uintptr_t S_HIGHER_MARK = 0xffff000000000000;
  static const std::uintptr_t S_LOWER_MARK = 0x0000000000000003;
  static const std::uintptr_t S_MARK = S_HIGHER_MARK | S_LOWER_MARK;

  typedef Type *PtrType;
  PtrType m_ptr;

  void mark_inplace_bool(const std::uintptr_t index, const bool mark) {
    if (mark) {
      this->m_ptr = reinterpret_cast<PtrType>(
          reinterpret_cast<std::uintptr_t>(this->m_ptr) |
          (std::uintptr_t(1) << index));
    } else {
      this->m_ptr = reinterpret_cast<PtrType>(
          reinterpret_cast<std::uintptr_t>(this->m_ptr) &
          ~(std::uintptr_t(1) << index));
    }
  }

  void write_counter_inplace(const std::uintptr_t counter) {
    this->m_ptr = reinterpret_cast<PtrType>(
        (reinterpret_cast<std::uintptr_t>(this->m_ptr) & (~S_HIGHER_MARK)) |
        (counter << 48));
  }

public:
  MarkedPointer() noexcept : m_ptr(nullptr) {}
  MarkedPointer(const PtrType ptr) noexcept : m_ptr(ptr) {}
  MarkedPointer(const PtrType ptr, const std::uint16_t counter) noexcept
      : m_ptr(ptr) {
    this->write_counter_inplace(counter);
  }

  PtrType address() const {
    return reinterpret_cast<PtrType>(reinterpret_cast<std::uintptr_t>(m_ptr) &
                                     ~S_MARK);
  }

  PtrType raw_address() const { return m_ptr; }

  Type &operator*() { return *this->address(); }

  PtrType operator->() { return this->address(); }

  bool operator==(const MarkedPointer<Type> &rhs) const {
    return m_ptr == rhs.m_ptr;
  }
  bool operator!=(const MarkedPointer<Type> &rhs) const {
    return m_ptr != rhs.m_ptr;
  }

  const std::uintptr_t lower_mark() const {
    return reinterpret_cast<std::uintptr_t>(m_ptr) & S_LOWER_MARK;
  }

  const std::uint16_t get_counter() const {
    return (reinterpret_cast<std::uintptr_t>(m_ptr) & S_HIGHER_MARK) >> 48;
  }

  const std::uintptr_t full_mark() {
    return reinterpret_cast<std::uintptr_t>(m_ptr) & S_MARK;
  }

  MarkedPointer<Type> write_counter(const std::uint16_t counter) const {
    const std::uintptr_t counter_ptr = counter;
    MarkedPointer<Type> copy = this->m_ptr;
    copy.write_counter_inplace(counter_ptr);
    return copy;
  }

  bool is_marked(const std::uintptr_t index) const {
    return (this->lower_mark() >> index) & 1;
  }

  MarkedPointer<Type> mark(const std::uintptr_t index) const {
    MarkedPointer<Type> copy = this->m_ptr;
    copy.mark_inplace_bool(index, true);
    return copy;
  }

  MarkedPointer<Type> unmark(const std::uintptr_t index) const {
    MarkedPointer<Type> copy = this->m_ptr;
    copy.mark_inplace_bool(index, false);
    return copy;
  }

  MarkedPointer<Type> unmark_all() const {
    MarkedPointer<Type> copy = this->address();
    return copy;
  }

  void mark_inplace(const std::uintptr_t index) {
    mark_inplace_bool(index, true);
  }

  void unmark_inplace(const std::uintptr_t index) {
    mark_inplace_bool(index, false);
  }
};

template <class Type> class AtomicMarkedPointer {
private:
  typedef Type *PtrType;
  union AtomicMarkedUnion {
    AtomicMarkedUnion(const MarkedPointer<Type> &ptr) : m_atomic_rep(ptr) {}
    std::atomic<MarkedPointer<Type>> m_atomic_rep;
    std::atomic_uintptr_t fetch_rep;
  };

  union MarkedUnion {
    MarkedUnion(std::uintptr_t raw_bits) : raw_bits(raw_bits) {}
    std::uintptr_t raw_bits;
    MarkedPointer<Type> marked_pointer;
  };

  AtomicMarkedUnion m_ptr;

public:
  AtomicMarkedPointer() : m_ptr(nullptr) {}
  AtomicMarkedPointer(const MarkedPointer<Type> ptr) : m_ptr(ptr) {}
  AtomicMarkedPointer(const PtrType ptr) : m_ptr(MarkedPointer<Type>(ptr)) {}

  MarkedPointer<Type>
  load(std::memory_order memory_order = std::memory_order_seq_cst) const {
    return m_ptr.m_atomic_rep.load(memory_order);
  }

  void store(MarkedPointer<Type> ptr,
             std::memory_order memory_order = std::memory_order_seq_cst) {
    m_ptr.m_atomic_rep.store(ptr, memory_order);
  }

  bool
  compare_exchange_strong(MarkedPointer<Type> &expected,
                          const MarkedPointer<Type> &desired,
                          std::memory_order success = std::memory_order_seq_cst,
                          std::memory_order fail = std::memory_order_seq_cst) {
    return m_ptr.m_atomic_rep.compare_exchange_strong(expected, desired,
                                                      success, fail);
  }

  bool
  compare_exchange_weak(MarkedPointer<Type> &expected,
                        const MarkedPointer<Type> &desired,
                        std::memory_order success = std::memory_order_seq_cst,
                        std::memory_order fail = std::memory_order_seq_cst) {
    return m_ptr.m_atomic_rep.compare_exchange_weak(expected, desired, success,
                                                    fail);
  }

  MarkedPointer<Type>
  fetch_mark(const std::uintptr_t index,
             std::memory_order memory_order = std::memory_order_seq_cst) {
    return fetch_mark_bool(index, true, memory_order);
  }

  MarkedPointer<Type>
  fetch_unmark(const std::uintptr_t index,
               std::memory_order memory_order = std::memory_order_seq_cst) {
    return fetch_mark_bool(index, false, memory_order);
  }

  MarkedPointer<Type>
  fetch_mark_bool(const std::uintptr_t index, const bool mark,
                  std::memory_order memory_order = std::memory_order_seq_cst) {
    if (mark) {
      MarkedUnion after = m_ptr.fetch_rep.fetch_or(1 << index, memory_order);
      return after.marked_pointer;
    } else {
      MarkedUnion after = m_ptr.fetch_rep.fetch_xor(1 << index, memory_order);
      return after.marked_pointer;
    }
  }
};
}
