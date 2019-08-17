#pragma once
/*
Lock-Free Hopscotch Hashing with bitmask buckets.
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

#include "common.h"
#include "primitives/brown_kcas.h"
//#include "primitives/brown_original_kcas.h"
#include <atomic>
#include <bitset>
#include <iostream>
#include <string.h>

namespace concurrent_data_structures {

template <class Allocator, template <class> class Reclaimer, class K,
          class KT = KeyTraits<K>>
class HopscotchBitMap_LF_Set {
private:
  typedef std::uintptr_t VersionState;
  typedef std::uintptr_t State;
  typedef std::uintptr_t Version;

  static const std::uintptr_t S_EMPTY = 0, S_BUSY = 1, S_INSERTING = 2,
                              S_COLLIDED = 3, S_MEMBER = 4;
  static const std::size_t S_STATE_BITS = 3, S_VERSION_BITS = 59;
  static const std::uintptr_t S_STATE_MASK =
      (std::uintptr_t(1) << S_STATE_BITS) - 1;
  static const std::uintptr_t S_VERSION_MASK =
      (std::uintptr_t(1) << S_VERSION_BITS) - 1;
  static VersionState create_version_state(const std::uintptr_t version,
                                           const std::uintptr_t state) {
    return (version << S_STATE_BITS) | state;
  }

  static std::uintptr_t get_version(const VersionState vs) {
    return (vs >> S_STATE_BITS) & S_VERSION_MASK;
  }

  static std::uintptr_t get_state(const VersionState vs) {
    return vs & S_STATE_MASK;
  }

  static const std::size_t S_BIT_MASK_RANGE = 64,
                           S_MAX_INSERTION_RANGE = 4 * 1024;
  typedef std::intptr_t timestamp_t;
  typedef std::uint64_t bit_mask_t;

  typedef Reclaimer<Allocator> MemReclaimer;
  typedef Brown_KCAS<Allocator, MemReclaimer, std::size_t(64)> KCAS;
  //  typedef BrownOriginal<Allocator, MemReclaimer, std::size_t(3)> KCAS;
  typedef typename KCAS::KCASDescriptor Descriptor;

  struct Bucket {
    typename KCAS::template KCASEntry<timestamp_t> timestamp;
    std::atomic<bit_mask_t> bit_mask;
    typename KCAS::template KCASEntry<VersionState> vs;
    std::atomic<K> key;
  };

  enum class AssistCode {
    Succeeded,
    Failed,
    Interrupted,
  };

  const std::size_t m_size, m_size_mask;
  Bucket *m_table;
  MemReclaimer m_reclaimer;
  KCAS m_kcas;

  AssistCode assist(const std::size_t thread_id,
                    ReclaimerPin<MemReclaimer> &pin, const K &key,
                    const std::size_t hash, const std::size_t my_offset,
                    const Version my_version) {
    VersionState my_expected_vs = create_version_state(my_version, S_INSERTING);
    const std::size_t my_index = hash + my_offset;
    Bucket &my_bucket = m_table[my_index];
    timestamp_t timestamp_before = m_kcas.read_value(
        thread_id, pin, &m_table[hash].timestamp, std::memory_order_acquire);
    bit_mask_t bit_mask =
        m_table[hash].bit_mask.load(std::memory_order_relaxed);
    while (bit_mask > 0) {
      const std::size_t offset = ffsll(bit_mask) - 1;
      const std::size_t index = hash + offset;
      if (offset != my_offset) {
        Bucket &bucket_j = m_table[index];
        VersionState vs_j = m_kcas.read_value(thread_id, pin, &bucket_j.vs,
                                              std::memory_order_acquire);
        State s_j = get_state(vs_j);
        // Found someone else inserting who is also our key
        if (s_j == S_INSERTING and
            bucket_j.key.load(std::memory_order_relaxed) == key) {
          // He comes before us, we are to be "snapped".
          if (offset < my_offset) {
            if (m_kcas.read_value(thread_id, pin, &bucket_j.vs,
                                  std::memory_order_acquire) == vs_j) {
              m_kcas.compare_exchange_strong_value(
                  thread_id, pin, &my_bucket.vs, my_expected_vs,
                  create_version_state(my_version, S_COLLIDED),
                  std::memory_order_release, std::memory_order_acquire);
              // Assist with the rest of the probe chain.
              return assist(thread_id, pin, key, hash, offset,
                            get_version(vs_j));
            } else {
              // Later boy found, snap him instead.
              if (m_kcas.read_value(thread_id, pin, &my_bucket.vs,
                                    std::memory_order_acquire) ==
                  create_version_state(my_version, S_INSERTING)) {
                m_kcas.compare_exchange_strong_value(
                    thread_id, pin, &bucket_j.vs, vs_j,
                    create_version_state(get_version(vs_j), S_COLLIDED),
                    std::memory_order_release, std::memory_order_acquire);
              }
            }
          }
        }
        vs_j = m_kcas.read_value(thread_id, pin, &bucket_j.vs,
                                 std::memory_order_acquire);
        Version v_j = get_version(vs_j);
        if (get_state(vs_j) == S_MEMBER and
            bucket_j.key.load(std::memory_order_relaxed) == key) {
          if (m_kcas.read_value(thread_id, pin, &bucket_j.vs,
                                std::memory_order_acquire) ==
              create_version_state(v_j, S_MEMBER)) {
            m_kcas.compare_exchange_strong_value(
                thread_id, pin, &my_bucket.vs, my_expected_vs,
                create_version_state(my_version, S_COLLIDED),
                std::memory_order_release, std::memory_order_acquire);
            return AssistCode::Failed;
          }
        }
      }
      bit_mask &= ~(bit_mask_t(1) << offset);
    }
    timestamp_t timestamp_after = m_kcas.read_value(
        thread_id, pin, &m_table[hash].timestamp, std::memory_order_acquire);
    if (timestamp_before != timestamp_after) {
      return AssistCode::Interrupted;
    }
    m_kcas.compare_exchange_strong_value(
        thread_id, pin, &my_bucket.vs, my_expected_vs,
        create_version_state(my_version, S_MEMBER), std::memory_order_release,
        std::memory_order_acquire);
    timestamp_after = m_kcas.read_value(
        thread_id, pin, &m_table[hash].timestamp, std::memory_order_acquire);
    return AssistCode::Succeeded;
  }

  bool find_closer_bucket(const std::size_t thread_id,
                          ReclaimerPin<MemReclaimer> &pin,
                          const std::size_t original_bucket,
                          std::size_t &offset, VersionState &free_bucket_vs) {

    const std::size_t free_bucket = original_bucket + offset;
  loadBegin:
    std::size_t distance = S_BIT_MASK_RANGE - 1;
    for (std::size_t current_bucket = free_bucket - distance;
         current_bucket < free_bucket; current_bucket++, distance--) {
      timestamp_t timestamp_before =
          m_kcas.read_value(thread_id, pin, &m_table[current_bucket].timestamp,
                            std::memory_order_relaxed);
      //          m_table[current_bucket].timestamp.load(std::memory_order_relaxed);
      const bit_mask_t current_bit_mask =
          m_table[current_bucket].bit_mask.load(std::memory_order_relaxed);
      bit_mask_t bit_mask = current_bit_mask;
      while (bit_mask > 0) {
        const std::size_t lowest_set = ffsll(bit_mask) - 1;
        const std::size_t index = current_bucket + lowest_set;
        if (index >= free_bucket) {
          break;
        }
        const Bucket &bucket = m_table[index];
        VersionState current_vs = m_kcas.read_value(thread_id, pin, &bucket.vs,
                                                    std::memory_order_acquire);
        State state = get_state(current_vs);
        if (state == S_MEMBER) {
          const K key = bucket.key.load(std::memory_order_relaxed);
          m_table[free_bucket].key.store(key, std::memory_order_relaxed);
          m_table[current_bucket].bit_mask.fetch_or(bit_mask_t(1) << distance,
                                                    std::memory_order_relaxed);
          Descriptor *desc = m_kcas.create_descriptor(3, thread_id);
          VersionState new_version =
              create_version_state(get_version(current_vs) + 1, S_BUSY);
          desc->add_value(&m_table[index].vs, current_vs, new_version);
          desc->add_value(
              &m_table[free_bucket].vs, free_bucket_vs,
              create_version_state(get_version(free_bucket_vs) + 1, S_MEMBER));
          desc->add_value(&m_table[current_bucket].timestamp, timestamp_before,
                          timestamp_before + 1);
          if (!m_kcas.cas(thread_id, pin, desc)) {
            m_table[current_bucket].bit_mask.fetch_xor(
                bit_mask_t(1) << distance, std::memory_order_relaxed);
            goto loadBegin;
          }
          // We can not stop the algorithm from checking this bucket.
          m_table[current_bucket].bit_mask.fetch_xor(
              bit_mask_t(1) << lowest_set, std::memory_order_relaxed);
          offset -= (free_bucket - index);
          free_bucket_vs = new_version;
          return true;
        }
        bit_mask ^= bit_mask_t(1) << lowest_set;
      }
      const timestamp_t timestamp_after =
          //          m_table[current_bucket].timestamp.load(std::memory_order_relaxed);
          m_kcas.read_value(thread_id, pin, &m_table[current_bucket].timestamp,
                            std::memory_order_relaxed);
      if (timestamp_before != timestamp_after) {
        timestamp_before = timestamp_after;
        goto loadBegin;
      }
    }
    return false;
  }

  void print_bucket(const std::size_t bucket_idx) {
    const std::size_t bitmask = m_table[bucket_idx].bit_mask.load();
    std::cout << "Reversed mask: " << std::bitset<64>(bitmask) << std::endl;
    for (std::size_t i = 0, mask = 1; i < S_BIT_MASK_RANGE; i++, mask <<= 1) {
      const Bucket &cur_bucket = m_table[(bucket_idx + i) & m_size_mask];
      //      const VersionState vw = cur_bucket.vw.load();
      const K cur_key = cur_bucket.key.load();
      const std::size_t original_bucket = KT::hash(cur_key) & m_size_mask;
      std::cout << i << ": " << cur_bucket.key.load()
                << (original_bucket == bucket_idx ? " HOME " : " AWAY ")
                << ", State: ";
      //      print_state(vw.state);
    }
  }

public:
  HopscotchBitMap_LF_Set(const std::size_t size, const std::size_t threads)
      : m_size(nearest_power_of_two(size)), m_size_mask(m_size - 1),
        m_table(static_cast<Bucket *>(Allocator::aligned_alloc(
            S_CACHE_SIZE, sizeof(Bucket) * (m_size + S_MAX_INSERTION_RANGE)))),
        m_reclaimer(threads, 4), m_kcas(threads, &m_reclaimer) {
    for (std::size_t i = 0; i < m_size + S_MAX_INSERTION_RANGE; i++) {
      m_kcas.write_value(0, &m_table[i].timestamp, timestamp_t(0),
                         std::memory_order_relaxed);
      //      m_table[i].timestamp.store(std::uintptr_t(0),
      //      std::memory_order_relaxed);
      m_table[i].bit_mask.store(bit_mask_t(0), std::memory_order_relaxed);
      m_kcas.write_value(0, &m_table[i].vs, create_version_state(0, S_EMPTY),
                         std::memory_order_relaxed);
      K null_key = KT::NullKey;
      m_table[i].key.store(null_key, std::memory_order_relaxed);
    }
  }

  ~HopscotchBitMap_LF_Set() { Allocator::free(m_table); }

  bool thread_init(const std::size_t thread_id) { return true; }

  bool contains(const K &key, std::size_t thread_id) {
    ReclaimerPin<MemReclaimer> pin(&m_reclaimer, thread_id);
    const std::size_t hash = KT::hash(key) & m_size_mask;

    timestamp_t timestamp_before =
        //        m_table[hash].timestamp.load(std::memory_order_relaxed);
        m_kcas.read_value(thread_id, pin, &m_table[hash].timestamp,
                          std::memory_order_relaxed);
    while (true) {
      // Snapshot
      bit_mask_t bit_mask =
          m_table[hash].bit_mask.load(std::memory_order_relaxed);
      while (bit_mask > 0) {
        const std::size_t lowest_set = ffsll(bit_mask) - 1;
        const std::size_t index = hash + lowest_set;
        const Bucket &bucket = m_table[index];
        VersionState vs = m_kcas.read_value(thread_id, pin, &bucket.vs,
                                            std::memory_order_acquire);
        State state = get_state(vs);
        if (state == S_MEMBER) {
          if (bucket.key.load(std::memory_order_relaxed) == key) {
            if (m_kcas.read_value(thread_id, pin, &bucket.vs,
                                  std::memory_order_acquire) == vs) {
              return true;
            }
          }
        }
        bit_mask ^= bit_mask_t(1) << lowest_set;
      }
      const timestamp_t timestamp_after =
          //          m_table[hash].timestamp.load(std::memory_order_relaxed);
          m_kcas.read_value(thread_id, pin, &m_table[hash].timestamp,
                            std::memory_order_relaxed);
      if (timestamp_before != timestamp_after) {
        timestamp_before = timestamp_after;
        continue;
      }
      return false;
    }
  }

  bool add(const K &key, std::size_t thread_id) {
    ReclaimerPin<MemReclaimer> pin(&m_reclaimer, thread_id);
    const std::size_t hash = KT::hash(key) & m_size_mask;
    // Check if key is already in table. Won't catch all, but
    // what it does catch is right. Optional.
    timestamp_t timestamp_before =
        //        m_table[hash].timestamp.load(std::memory_order_relaxed);
        m_kcas.read_value(thread_id, pin, &m_table[hash].timestamp,
                          std::memory_order_relaxed);
    bit_mask_t original_mask =
        m_table[hash].bit_mask.load(std::memory_order_relaxed);
    while (true) {
      // Snapshot
      bit_mask_t bit_mask = original_mask;
      while (bit_mask > 0) {
        // Naive search for bit set
        const std::size_t lowest_set = ffsll(bit_mask) - 1;
        const std::size_t index = hash + lowest_set;
        const Bucket &bucket = m_table[index];
        VersionState vs = m_kcas.read_value(thread_id, pin, &bucket.vs,
                                            std::memory_order_acquire);
        State state = get_state(vs);
        if (state == S_MEMBER) {
          if (bucket.key.load(std::memory_order_relaxed) == key) {
            if (m_kcas.read_value(thread_id, pin, &bucket.vs,
                                  std::memory_order_acquire) == vs) {
              return false;
            }
          }
        }
        bit_mask ^= bit_mask_t(1) << lowest_set;
      }
      const timestamp_t timestamp_after =
          //          m_table[hash].timestamp.load(std::memory_order_relaxed);
          m_kcas.read_value(thread_id, pin, &m_table[hash].timestamp,
                            std::memory_order_relaxed);

      if (timestamp_before != timestamp_after) {
        original_mask = m_table[hash].bit_mask.load(std::memory_order_relaxed);
        timestamp_before = timestamp_after;
        continue;
      } else {
        break;
      }
    }
    std::size_t offset = 0;
    Version version;
    bool claimed_bucket = false;
    for (; offset < S_MAX_INSERTION_RANGE; offset++) {
      const std::size_t index = hash + offset;
      Bucket &current_bucket = m_table[index];
    loadBegin:
      VersionState vs = m_kcas.read_value(thread_id, pin, &current_bucket.vs,
                                          std::memory_order_acquire);
      State state = get_state(vs);
      version = get_version(vs);
      if (state == S_EMPTY) {
        if (m_kcas.compare_exchange_weak_value(
                thread_id, pin, &current_bucket.vs, vs,
                create_version_state(version, S_BUSY),
                std::memory_order_release, std::memory_order_acquire)) {
          claimed_bucket = true;
          break;
        } else {
          goto loadBegin;
        }
      }
    }
    if (!claimed_bucket) {
      return false;
    }
    VersionState vs = create_version_state(version, S_BUSY);
    while (offset >= S_BIT_MASK_RANGE) {
      std::size_t offset_before = offset;
      bool found_closer = find_closer_bucket(thread_id, pin, hash, offset, vs);
      if (offset_before == offset) {
        const K null_key = KT::NullKey;
        const std::size_t bucket_index = hash + offset;
        Bucket &final_bucket = m_table[bucket_index];
        final_bucket.key.store(null_key, std::memory_order_relaxed);
        m_kcas.write_value(thread_id, &final_bucket.vs,
                           create_version_state(version + 1, S_EMPTY),
                           std::memory_order_release);
        return false;
      }
      assert(found_closer);
    }
    version = get_version(vs);
    // Mark the entry as reachable.
    const std::size_t bucket_index = hash + offset;
    Bucket &final_bucket = m_table[bucket_index];
    const bit_mask_t mask = bit_mask_t(1) << offset;
    final_bucket.key.store(key, std::memory_order_relaxed);
    m_table[hash].bit_mask.fetch_or(mask, std::memory_order_relaxed);

    while (true) {
      m_kcas.write_value(thread_id, &final_bucket.vs,
                         create_version_state(version, S_INSERTING),
                         std::memory_order_release);
      AssistCode r = assist(thread_id, pin, key, hash, offset, version);
      VersionState vs = m_kcas.read_value(thread_id, pin, &final_bucket.vs,
                                          std::memory_order_acquire);
      VersionState bad_vs = create_version_state(version, S_COLLIDED);
      if (r != AssistCode::Interrupted and vs != bad_vs) {
        return true;
      }
      if (r == AssistCode::Failed) {
        const K null_key = KT::NullKey;
        final_bucket.key.store(null_key, std::memory_order_relaxed);
        m_table[hash].bit_mask.fetch_xor(mask, std::memory_order_relaxed);
        m_kcas.write_value(thread_id, &final_bucket.vs,
                           create_version_state(version + 1, S_EMPTY),
                           std::memory_order_release);
        return false;
      }
      version++;
    }
  }

  bool remove(const K &key, std::size_t thread_id) {
    ReclaimerPin<MemReclaimer> pin(&m_reclaimer, thread_id);
    const std::size_t hash = KT::hash(key) & m_size_mask;
    timestamp_t timestamp_before = m_kcas.read_value(
        thread_id, pin, &m_table[hash].timestamp, std::memory_order_relaxed);
    while (true) {
      // Snapshot
      bit_mask_t original_mask =
          m_table[hash].bit_mask.load(std::memory_order_relaxed);
      bit_mask_t bit_mask = original_mask;
      while (bit_mask > 0) {
        const std::size_t lowest_set = ffsll(bit_mask) - 1;
        const std::size_t index = hash + lowest_set;
        Bucket &bucket = m_table[index];
        VersionState vs = m_kcas.read_value(thread_id, pin, &bucket.vs,
                                            std::memory_order_acquire);
        State state = get_state(vs);
        if (state == S_MEMBER) {

          if (bucket.key.load(std::memory_order_relaxed) == key and
              m_kcas.compare_exchange_strong_value(
                  thread_id, pin, &bucket.vs, vs,
                  create_version_state(get_version(vs), S_BUSY),
                  std::memory_order_relaxed, std::memory_order_relaxed)) {
            // Move back keys (optional)
            //            bit_mask ^= std::size_t(1) << lowest_set;
            //            if (bit_mask > 0) {
            //              const std::size_t furthest_set = 64 -
            //              __builtin_clzll(bit_mask);
            //              const std::size_t furthest_index = hash +
            //              furthest_set;
            //              VersionState furthest_vs =
            //                  m_kcas.read_value(thread_id, pin,
            //                  &m_table[furthest_index].vs,
            //                                    std::memory_order_acquire);
            //              State state = get_state(furthest_vs);
            //              if (state == S_MEMBER) {
            //                const K furthest_key =
            //                m_table[furthest_index].key;
            //                const std::size_t furthest_hash =
            //                    KT::hash(furthest_key) & m_size_mask;
            //                if (hash == furthest_hash) {
            //                  bucket.key.store(furthest_key,
            //                  std::memory_order_relaxed);
            //                  Descriptor *desc = m_kcas.create_descriptor(3,
            //                  thread_id);
            //                  VersionState expected_vs =
            //                      create_version_state(get_version(vs),
            //                      S_BUSY);
            //                  desc->add_value(&bucket.vs, expected_vs,
            //                                  create_version_state(
            //                                      get_version(expected_vs) +
            //                                      1, S_MEMBER));
            //                  desc->add_value(&m_table[furthest_index].vs,
            //                  furthest_vs,
            //                                  create_version_state(
            //                                      get_version(furthest_vs) +
            //                                      1, S_BUSY));
            //                  desc->add_value(&m_table[hash].timestamp,
            //                  timestamp_before,
            //                                  timestamp_before + 1);
            //                  if (m_kcas.cas(thread_id, pin, desc)) {
            //                    const K null_key = KT::NullKey;
            //                    m_table[furthest_index].key.store(
            //                        null_key, std::memory_order_relaxed);
            //                    m_table[hash].bit_mask.fetch_xor(std::size_t(1)
            //                                                         <<
            //                                                         furthest_set,
            //                                                     std::memory_order_relaxed);
            //                    Version version = get_version(furthest_vs);
            //                    m_kcas.write_value(
            //                        thread_id, &m_table[furthest_index].vs,
            //                        create_version_state(version + 1,
            //                        S_EMPTY),
            //                        std::memory_order_release);
            //                    return true;
            //                  }
            //                }
            //              }
            //            }
            const K null_key = KT::NullKey;
            bucket.key.store(null_key, std::memory_order_relaxed);
            m_table[hash].bit_mask.fetch_xor(bit_mask_t(1) << lowest_set,
                                             std::memory_order_relaxed);
            Version version = get_version(vs);
            m_kcas.write_value(thread_id, &bucket.vs,
                               create_version_state(version + 1, S_EMPTY),
                               std::memory_order_release);

            return true;
          }
        }
        bit_mask ^= bit_mask_t(1) << lowest_set;
      }
      const timestamp_t timestamp_after =
          //          m_table[hash].timestamp.load(std::memory_order_relaxed);
          m_kcas.read_value(thread_id, pin, &m_table[hash].timestamp,
                            std::memory_order_relaxed);
      if (timestamp_before != timestamp_after) {
        timestamp_before = timestamp_after;
        continue;
      }
      return false;
    }
  }

  void print_table() {}
};
}
