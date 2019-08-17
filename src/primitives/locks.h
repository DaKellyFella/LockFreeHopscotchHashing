#pragma once
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

#include "cache_utils.h"
#include <atomic>
#include <cstdint>
#include <pthread.h>

namespace concurrent_data_structures {

class alignas(S_CACHE_PADDING) PthreadSpinLock {
private:
  pthread_spinlock_t m_lock;

public:
  PthreadSpinLock();
  void lock();
  void unlock();
};

class alignas(S_CACHE_PADDING) PthreadMutex {

private:
  pthread_mutex_t m_lock;

public:
  PthreadMutex();
  void lock();
  void unlock();
};

class alignas(S_CACHE_PADDING) TicketLock {
private:
  CachePadded<std::atomic_uintptr_t> m_entry_ticket, m_exit_ticket;

public:
  TicketLock();
  void lock();
  void unlock();
};

class alignas(S_CACHE_PADDING) ElidedLock {
private:
  static const std::size_t MAX_RETRIES = 20;
  std::atomic_bool m_lock;

public:
  ElidedLock();

  void lock();
  void unlock();
};

class TransactionalLock {
private:
  static ElidedLock S_TRANS_LOCK;

public:
  TransactionalLock();
  ~TransactionalLock();
  void lock();
  void unlock();
};
}
