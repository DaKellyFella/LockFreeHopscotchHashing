#pragma once

#include "common.h"
#include "primitives/cache_utils.h"
#include "primitives/locks.h"
#include <atomic>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <string.h>

namespace concurrent_data_structures {

template <class Allocator, template <class> class Reclaimer, class Lock,
          class K, class KT = KeyTraits<K>>
class HopscotchBitMap_Locked_Set {
private:
  struct Segment {
    CachePadded<std::atomic<std::uint64_t>> timestamp;
    Lock lock;
  };

  typedef std::uint64_t bit_mask_t;

  struct Bucket {
    std::atomic<bit_mask_t> bit_mask;
    std::atomic_bool in_use;
    std::atomic<K> key;
  };

  std::size_t m_size_mask, m_segment_shift;
  CachePadded<Segment> *m_segments;
  Bucket *m_table;

  static const std::size_t S_BIT_MASK_RANGE = 64,
                           S_MAX_INSERTION_RANGE = 4 * 1024;

  bool find_closer_free_backet(const std::size_t free_segment,
                               std::size_t &free_bucket,
                               std::size_t &free_distance) {
  loopBegin:
    std::size_t distance = S_BIT_MASK_RANGE - 1;
    for (std::size_t current_bucket = free_bucket - distance;
         current_bucket < free_bucket; current_bucket++, distance--) {
      bit_mask_t bit_mask =
          m_table[current_bucket].bit_mask.load(std::memory_order_relaxed);
      while (bit_mask > 0) {
        const std::size_t moved_offset = ffsll(bit_mask) - 1;
        const std::size_t index = current_bucket + moved_offset;
        if (index >= free_bucket) {
          break;
        }
        const std::size_t current_segment = index >> m_segment_shift;
        if (free_segment != current_segment) {
          m_segments[current_segment].lock.lock();
        }
        const std::size_t bit_mask_after =
            m_table[current_bucket].bit_mask.load(std::memory_order_relaxed);
        // Entry we want to push up has moved.
        if (bit_mask_after != bit_mask) {
          if (free_segment != current_segment) {
            m_segments[current_segment].lock.lock();
          }
          goto loopBegin;
        }
        const K current_key =
            m_table[index].key.load(std::memory_order_relaxed);
        m_table[free_bucket].key.store(current_key, std::memory_order_relaxed);
        m_table[current_bucket].bit_mask.fetch_or(bit_mask_t(1U) << distance,
                                                  std::memory_order_relaxed);
        m_segments[current_segment].timestamp.fetch_add(
            1, std::memory_order_relaxed);
        //        m_table[index].key.store(free_key, std::memory_order_relaxed);
        m_table[current_bucket].bit_mask.fetch_and(
            ~(bit_mask_t(1U) << moved_offset));
        free_distance -= (free_bucket - index);
        free_bucket = index;
        if (free_segment != current_segment) {
          m_segments[current_segment].lock.unlock();
        }
        return true;
      }
    }
    return false;
  }

public:
  HopscotchBitMap_Locked_Set(const std::size_t size, const std::size_t threads)
      : m_size_mask(nearest_power_of_two(size) - 1),
        m_table(static_cast<Bucket *>(Allocator::aligned_alloc(
            S_CACHE_PADDING,
            (m_size_mask + 1 + S_MAX_INSERTION_RANGE) * sizeof(Bucket)))) {
    std::size_t num_timestamps = nearest_power_of_two(threads);
    std::uint8_t num_timestamp_bits = 0;
    for (std::size_t timestamp = num_timestamps; timestamp > 0;
         timestamp >>= 1, num_timestamp_bits++) {
    }
    std::uint8_t num_size_bits = 0;
    for (std::size_t size = m_size_mask + 1; size > 0;
         size >>= 1, num_size_bits++) {
    }
    std::uint8_t timestamp_shift = num_size_bits - num_timestamp_bits;
    m_segment_shift = timestamp_shift;
    m_segments = static_cast<CachePadded<Segment> *>(Allocator::aligned_alloc(
        S_CACHE_PADDING, num_timestamps * sizeof(CachePadded<Segment>)));

    for (std::size_t s = 0; s < num_timestamps; s++) {
      m_segments[s].timestamp.store(bit_mask_t(0), std::memory_order_relaxed);
      new (&m_segments[s].lock) Lock();
    }

    for (std::size_t b = 0; b <= (m_size_mask + S_MAX_INSERTION_RANGE); b++) {
      m_table[b].bit_mask.store(bit_mask_t(0), std::memory_order_relaxed);
      m_table[b].in_use.store(false, std::memory_order_relaxed);
      m_table[b].key.store(KT::NullKey, std::memory_order_relaxed);
    }
  }

  ~HopscotchBitMap_Locked_Set() {
    Allocator::free(m_table);
    Allocator::free(m_segments);
  }

  bool thread_init(const std::size_t thread_id) { return true; }

  bool contains(const K key, const std::size_t thread_id) {
    const std::size_t hash = KT::hash(key);
    const std::size_t original_bucket = hash & m_size_mask;
    const std::size_t original_segment = original_bucket >> m_segment_shift;

    std::intptr_t timestamp_before =
        m_segments[original_segment].timestamp.load();
    while (true) {
      bit_mask_t bit_mask =
          m_table[original_bucket].bit_mask.load(std::memory_order_relaxed);
      while (bit_mask > 0) {
        const std::size_t lowest_set = ffsll(bit_mask) - 1;
        const std::size_t current_index = (hash + lowest_set) & m_size_mask;
        const K current_key =
            m_table[current_index].key.load(std::memory_order_relaxed);
        if (key == current_key) {
          return true;
        }
        bit_mask &= ~(std::size_t(1) << lowest_set);
      }
      const std::intptr_t timestamp_after =
          m_segments[original_segment].timestamp.load();
      if (timestamp_before != timestamp_after) {
        timestamp_before = timestamp_after;
        continue;
      }
      return false;
    }
  }

  bool add(const K key, const std::size_t thread_id) {
    const std::size_t hash = KT::hash(key);
    const std::size_t original_bucket = hash & m_size_mask;
    const std::size_t original_segment = original_bucket >> m_segment_shift;

    std::lock_guard<Lock> guard(m_segments[original_segment].lock);

    bit_mask_t bit_mask =
        m_table[original_bucket].bit_mask.load(std::memory_order_relaxed);
    while (bit_mask > 0) {
      const std::size_t lowest_set = ffsll(bit_mask) - 1;
      const std::size_t current_index = (hash + lowest_set) & m_size_mask;
      const K current_key =
          m_table[current_index].key.load(std::memory_order_relaxed);
      if (key == current_key) {
        return false;
      }
      bit_mask &= ~(std::size_t(1) << lowest_set);
    }

    std::size_t offset = 0;
    std::size_t reserved_bucket = original_bucket;
    for (; offset < S_MAX_INSERTION_RANGE; reserved_bucket++, offset++) {
      // If not in use and when claiming the bucket it was still not in use...
      if (!m_table[reserved_bucket].in_use.load(std::memory_order_relaxed) and
          m_table[reserved_bucket].in_use.exchange(
              true, std::memory_order_relaxed) == false) {
        break;
      }
    }

    if (offset < S_MAX_INSERTION_RANGE) {
      while (offset >= S_BIT_MASK_RANGE) {
        bool moved =
            find_closer_free_backet(original_segment, reserved_bucket, offset);
        if (!moved) {
          m_table[reserved_bucket].key.store(KT::NullKey,
                                             std::memory_order_relaxed);
          m_table[reserved_bucket].in_use.store(false,
                                                std::memory_order_release);
          return false;
        }
      }
      m_table[reserved_bucket].key.store(key, std::memory_order_relaxed);
      m_table[original_bucket].bit_mask.fetch_or(bit_mask_t(1) << offset);
      return true;
    } else {
      //      std::cerr << "Couldn't find a bucket within the range." <<
      //      std::endl;
      return false;
    }
  }

  bool remove(const K key, const std::size_t thread_id) {
    const std::size_t hash = KT::hash(key);
    const std::size_t original_bucket = hash & m_size_mask;
    const std::size_t original_segment = original_bucket >> m_segment_shift;

    std::lock_guard<Lock> guard(m_segments[original_segment].lock);
    bit_mask_t bit_mask =
        m_table[original_bucket].bit_mask.load(std::memory_order_relaxed);
    while (bit_mask > 0) {
      const std::size_t lowest_set = ffsll(bit_mask) - 1;
      const std::size_t current_index = (hash + lowest_set) & m_size_mask;
      const K current_key =
          m_table[current_index].key.load(std::memory_order_relaxed);
      if (key == current_key) {
        m_table[current_index].key.store(KT::NullKey,
                                         std::memory_order_relaxed);
        m_table[current_index].in_use.store(false, std::memory_order_release);
        m_table[original_bucket].bit_mask.fetch_xor(
            std::size_t(1) << lowest_set, std::memory_order_relaxed);
        return true;
      }
      bit_mask &= ~(std::size_t(1) << lowest_set);
    }
    return false;
  }

  std::size_t size() {
    std::size_t counter = 0;
    return counter;
  }
};

template <class Allocator, template <class> class Reclaimer, class K>
using HopscotchBitMap_SpinLock_Set =
    HopscotchBitMap_Locked_Set<Allocator, Reclaimer, PthreadSpinLock, K>;

template <class Allocator, template <class> class Reclaimer, class K>
using HopscotchBitMap_Mutex_Set =
    HopscotchBitMap_Locked_Set<Allocator, Reclaimer, PthreadMutex, K>;

template <class Allocator, template <class> class Reclaimer, class K>
using HopscotchBitMap_TicketLock_Set =
    HopscotchBitMap_Locked_Set<Allocator, Reclaimer, TicketLock, K>;
}
