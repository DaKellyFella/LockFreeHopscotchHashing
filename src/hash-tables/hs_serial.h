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

template <class Allocator, template <class> class Reclaimer, class K,
          class KT = KeyTraits<K>>
class HopscotchBitMap_Serial_Set {
private:
  typedef std::uint64_t bit_mask_t;
  struct Bucket {
    bit_mask_t bit_mask;
    K key;
  };

  std::size_t m_size_mask;
  Bucket *m_table;

  static const std::size_t S_BIT_MASK_RANGE = 64,
                           S_MAX_INSERTION_RANGE = 4 * 1024;

  bool find_closer_free_backet(std::size_t &free_bucket,
                               std::size_t &free_distance) {
    std::size_t distance = S_BIT_MASK_RANGE - 1;
    for (std::size_t current_bucket = free_bucket - distance;
         current_bucket < free_bucket; current_bucket++, distance--) {
      bit_mask_t bit_mask = m_table[current_bucket].bit_mask;
      while (bit_mask > 0) {
        const std::size_t moved_offset = ffsll(bit_mask) - 1;
        const std::size_t index = current_bucket + moved_offset;
        if (index >= free_bucket) {
          break;
        }
        // Entry we want to push up has moved.
        const K current_key = m_table[index].key;
        m_table[free_bucket].key = current_key;
        m_table[current_bucket].bit_mask |= bit_mask_t(1U) << distance;
        m_table[current_bucket].bit_mask &= ~(bit_mask_t(1U) << moved_offset);
        free_distance -= (free_bucket - index);
        free_bucket = index;
        return true;
      }
    }
    return false;
  }

public:
  HopscotchBitMap_Serial_Set(const std::size_t size, const std::size_t threads)
      : m_size_mask(nearest_power_of_two(size) - 1),
        m_table(static_cast<Bucket *>(Allocator::aligned_alloc(
            S_CACHE_PADDING,
            (m_size_mask + 1 + S_MAX_INSERTION_RANGE) * sizeof(Bucket)))) {
    for (std::size_t b = 0; b <= (m_size_mask + S_MAX_INSERTION_RANGE); b++) {
      m_table[b].bit_mask = bit_mask_t(0);
      m_table[b].key = KT::NullKey;
    }
  }

  ~HopscotchBitMap_Serial_Set() { Allocator::free(m_table); }

  bool thread_init(const std::size_t thread_id) { return true; }

  bool contains(const K key, const std::size_t thread_id) {
    const std::size_t hash = KT::hash(key);
    const std::size_t original_bucket = hash & m_size_mask;

    bit_mask_t bit_mask = m_table[original_bucket].bit_mask;
    while (bit_mask > 0) {
      const std::size_t lowest_set = ffsll(bit_mask) - 1;
      const std::size_t current_index = (hash + lowest_set) & m_size_mask;
      const K current_key = m_table[current_index].key;
      if (key == current_key) {
        return true;
      }
      bit_mask &= ~(std::size_t(1) << lowest_set);
    }
    return false;
  }

  bool add(const K key, const std::size_t thread_id) {
    const std::size_t hash = KT::hash(key);
    const std::size_t original_bucket = hash & m_size_mask;

    bit_mask_t bit_mask = m_table[original_bucket].bit_mask;
    while (bit_mask > 0) {
      const std::size_t lowest_set = ffsll(bit_mask) - 1;
      const std::size_t current_index = (hash + lowest_set) & m_size_mask;
      const K current_key = m_table[current_index].key;
      if (key == current_key) {
        return false;
      }
      bit_mask &= ~(std::size_t(1) << lowest_set);
    }

    std::size_t offset = 0;
    std::size_t reserved_bucket = original_bucket;
    for (; offset < S_MAX_INSERTION_RANGE; reserved_bucket++, offset++) {
      // If not in use and when claiming the bucket it was still not in use...
      if (m_table[reserved_bucket].key == KT::NullKey) {
        break;
      }
    }

    if (offset < S_MAX_INSERTION_RANGE) {
      while (offset >= S_BIT_MASK_RANGE) {
        bool moved = find_closer_free_backet(reserved_bucket, offset);
        if (!moved) {
          m_table[reserved_bucket].key = KT::NullKey;
          return false;
        }
      }
      m_table[reserved_bucket].key = key;
      m_table[original_bucket].bit_mask |= bit_mask_t(1) << offset;
      return true;
    } else {
      return false;
    }
  }

  bool remove(const K key, const std::size_t thread_id) {
    const std::size_t hash = KT::hash(key);
    const std::size_t original_bucket = hash & m_size_mask;

    bit_mask_t bit_mask = m_table[original_bucket].bit_mask;
    while (bit_mask > 0) {
      const std::size_t lowest_set = ffsll(bit_mask) - 1;
      const std::size_t current_index = (hash + lowest_set) & m_size_mask;
      const K current_key = m_table[current_index].key;
      if (key == current_key) {
        m_table[current_index].key = KT::NullKey;
        m_table[original_bucket].bit_mask ^= std::size_t(1) << lowest_set;
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
}
