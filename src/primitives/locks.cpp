/*
Simple interface for locks with some implementations.
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
#include "locks.h"
#include "cache_utils.h"
#include <atomic>
#include <cstdint>
#include <cstdlib>
#include <immintrin.h>
#include <pthread.h>

namespace concurrent_data_structures {

PthreadSpinLock::PthreadSpinLock() {
  pthread_spin_init(&m_lock, PTHREAD_PROCESS_PRIVATE);
}
void PthreadSpinLock::lock() { pthread_spin_lock(&m_lock); }
void PthreadSpinLock::unlock() { pthread_spin_unlock(&m_lock); }

PthreadMutex::PthreadMutex() { pthread_mutex_init(&m_lock, nullptr); }
void PthreadMutex::lock() { pthread_mutex_lock(&m_lock); }
void PthreadMutex::unlock() { pthread_mutex_unlock(&m_lock); }

TicketLock::TicketLock() : m_entry_ticket(0), m_exit_ticket(0) {}
void TicketLock::lock() {
  std::uintptr_t my_entry_ticket =
      m_entry_ticket.fetch_add(1, std::memory_order_relaxed);
  while (m_exit_ticket.load(std::memory_order_acquire) != my_entry_ticket) {
  }
}
void TicketLock::unlock() {
  m_exit_ticket.fetch_add(1, std::memory_order_release);
}

ElidedLock::ElidedLock() { m_lock.store(false, std::memory_order_relaxed); }
void ElidedLock::lock() {
  for (std::size_t i = 0; i < MAX_RETRIES; i++) {
    unsigned int status = _xbegin();
    if (status == _XBEGIN_STARTED) {
      if (!m_lock.load(std::memory_order_relaxed)) {
        return;
      } else {
        _xabort(0xff);
      }
    }
    if ((status & _XABORT_EXPLICIT) && _XABORT_CODE(status) == 0xff) {
      // Wait for lock to be free.
      while (m_lock.load(std::memory_order_relaxed)) {
        _mm_pause();
      }
    }
    if (!(status & _XABORT_RETRY) and
        !((status & _XABORT_EXPLICIT) and _XABORT_CODE(status) == 0xff)) {
      break;
    }
  }

  while (true) {
    bool value = m_lock.load(std::memory_order_relaxed);
    if (!value and
        m_lock.compare_exchange_weak(value, true, std::memory_order_acquire,
                                     std::memory_order_relaxed)) {
      return;
    }
    _mm_pause();
  }
}

void ElidedLock::unlock() {
  if (!m_lock.load(std::memory_order_relaxed) and _xtest()) {
    _xend();
  } else {
    m_lock.store(false, std::memory_order_release);
  }
}

ElidedLock TransactionalLock::S_TRANS_LOCK;

TransactionalLock::TransactionalLock() {}
TransactionalLock::~TransactionalLock() {}
void TransactionalLock::lock() { S_TRANS_LOCK.lock(); }
void TransactionalLock::unlock() { S_TRANS_LOCK.unlock(); }
}
