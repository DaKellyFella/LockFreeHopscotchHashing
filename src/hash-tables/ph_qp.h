#pragma once

/*
Lock-Free quadratic probing with physical deletion based on
"Non-blocking hash tables with open addressing" by Chris Purcell
and Tim Harris.
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
#include <atomic>

namespace concurrent_data_structures {

template <class Allocator, template <class> class Reclaimer, class K,
          class KT = KeyTraits<K>>
class PH_QP_Set {
private:
  struct Bucket {
    std::atomic_uintptr_t vs;
    std::atomic<K> key;
  };
  std::size_t m_size, m_size_mask;
  std::atomic_size_t *m_probe_bounds;
  Bucket *m_table;

  static const std::uintptr_t S_EMPTY = 0, S_INSERTING = 1, S_MEMBER = 2,
                              S_VISIBLE = 3, S_BUSY = 4, S_COLLIDED = 5;

  Bucket *get_bucket(std::size_t hash, std::size_t index) {
    return &m_table[(hash + index * (index + 1) / 2) & m_size_mask];
    //    return &m_table[(hash + index) & m_size_mask];
  }

  std::uintptr_t get_version(std::uintptr_t vs) { return vs >> 32; }

  std::uintptr_t embed_version(std::uintptr_t vs, std::uintptr_t v) {
    return get_state(vs) | (v << 32);
  }

  std::uintptr_t get_state(std::uintptr_t vs) {
    return vs & ~(get_version(vs) << 32);
  }

  std::uintptr_t embed_state(std::uintptr_t v, std::uintptr_t s) {
    return (v << 32) | s;
  }

  bool contains_collision(std::size_t hash, std::size_t index) {
    std::uintptr_t vs1 = get_bucket(hash, index)->vs.load();
    std::uintptr_t state1 = get_state(vs1);
    if (state1 == S_VISIBLE or state1 == S_INSERTING or state1 == S_MEMBER) {
      if ((KT::hash(
               get_bucket(hash, index)->key.load(std::memory_order_relaxed)) &
           m_size_mask) == hash) {
        std::uintptr_t vs2 = get_bucket(hash, index)->vs.load();
        std::uintptr_t state2 = get_state(vs2);
        if (state2 == S_VISIBLE or state2 == S_INSERTING or
            state2 == S_MEMBER) {
          if (get_version(vs1) == get_version(vs2)) {
            return true;
          }
        }
      }
    }
    return false;
  }

  std::uintptr_t get_probe_bound(const std::uintptr_t pb) { return pb >> 1; }
  bool get_scanning(const std::uintptr_t pb) { return (pb & 1) == 1; }
  std::uintptr_t embed_scanning(const std::uintptr_t b, const bool scanning) {
    return (b << 1) | (scanning ? 1 : 0);
  }
  void cond_raise_bound(std::size_t hash, std::size_t index) {
    std::uintptr_t pb, new_pb;
    do {
      pb = m_probe_bounds[hash].load();
      new_pb = std::max(get_probe_bound(pb), index);
    } while (!m_probe_bounds[hash].compare_exchange_strong(
        pb, embed_scanning(new_pb, false)));
  }

  void cond_lower_bound(std::size_t hash, std::size_t index) {
    std::uintptr_t pb = m_probe_bounds[hash].load();
    if (get_scanning(pb)) {
      m_probe_bounds[hash].compare_exchange_strong(
          pb, embed_scanning(get_probe_bound(pb), false));
    }
    if (index > 0) {
      pb = embed_scanning(index, false);
      while (m_probe_bounds[hash].compare_exchange_strong(
          pb, embed_scanning(index, true))) {
        std::size_t i = index - 1;
        while (i > 0 and !contains_collision(hash, i)) {
          i--;
        }
        std::size_t expected_index = embed_scanning(index, true);
        m_probe_bounds[hash].compare_exchange_strong(expected_index,
                                                     embed_scanning(i, false));
      }
    }
  }

  bool assist(const K &key, const std::size_t hash, const std::size_t index,
              std::size_t ver_i) {
    std::uintptr_t pb = m_probe_bounds[hash].load();
    std::size_t max = get_probe_bound(pb);
    for (std::size_t j = 0; j <= max; j++) {
      if (j != index) {
        Bucket *bucket_j = get_bucket(hash, j);
        std::uintptr_t vs_j = bucket_j->vs.load();
        std::uintptr_t s_j = get_state(vs_j);
        std::uintptr_t expected_ver_i = embed_state(ver_i, S_INSERTING);
        if (s_j == S_INSERTING and
            bucket_j->key.load(std::memory_order_relaxed) == key) {
          if (j < index) {
            if (bucket_j->vs.load() == vs_j) {
              /*bool _ =*/get_bucket(hash, index)
                  ->vs.compare_exchange_strong(expected_ver_i,
                                               embed_state(ver_i, S_COLLIDED));
              return assist(key, hash, j, get_version(vs_j));
            } else {
              if (get_bucket(hash, index)->vs.load() ==
                  embed_state(ver_i, S_INSERTING)) {
                //                bool _ =
                get_bucket(hash, index)
                    ->vs.compare_exchange_strong(
                        expected_ver_i, embed_state(ver_i, S_COLLIDED));
              }
            }
          }
        }
        vs_j = bucket_j->vs.load();
        std::uintptr_t v_j = get_version(vs_j);
        if (get_state(vs_j) == S_MEMBER and
            bucket_j->key.load(std::memory_order_relaxed) == key) {
          if (bucket_j->vs.load() == embed_state(v_j, S_MEMBER)) {
            /*bool _ = */ get_bucket(hash, index)
                ->vs.compare_exchange_strong(expected_ver_i,
                                             embed_state(ver_i, S_COLLIDED));
            return false;
          }
        }
      }
    }
    ver_i = embed_state(ver_i, S_INSERTING);
    //    bool _ =
    get_bucket(hash, index)
        ->vs.compare_exchange_strong(ver_i, embed_state(ver_i, S_MEMBER));
    return true;
  }

public:
  PH_QP_Set(const std::size_t size, const std::size_t threads)
      : m_size(nearest_power_of_two(size)), m_size_mask(m_size - 1),
        m_probe_bounds(static_cast<std::atomic_size_t *>(
            Allocator::malloc(sizeof(std::atomic_size_t) * m_size))),
        m_table(
            static_cast<Bucket *>(Allocator::malloc(sizeof(Bucket) * m_size))) {
    for (std::size_t i = 0; i < m_size; i++) {
      m_table[i].vs.store(0, std::memory_order_relaxed);
      m_table[i].key.store(KT::NullKey, std::memory_order_relaxed);
      m_probe_bounds[i].store(0, std::memory_order_relaxed);
    }
  }
  ~PH_QP_Set() {
    Allocator::free(m_probe_bounds);
    Allocator::free(m_table);
    /*print_table(); */
  }

  bool thread_init(const std::size_t thread_id) { return true; }

  bool contains(const K &key, std::size_t thread_id) {
    const std::size_t hash = KT::hash(key) & m_size_mask;
    std::uintptr_t pb = m_probe_bounds[hash].load();
    std::size_t max = get_probe_bound(pb);
    for (std::size_t i = 0; i <= max; i++) {
      const Bucket *bucket = get_bucket(hash, i);
      std::uintptr_t vs = bucket->vs.load();
      std::uintptr_t state = get_state(vs);
      if (state == S_MEMBER and
          bucket->key.load(std::memory_order_relaxed) == key) {
        if (bucket->vs.load() == vs) {
          return true;
        }
      }
    }
    return false;
  }

  bool add(const K &key, std::size_t thread_id) {
    const std::size_t hash = KT::hash(key) & m_size_mask;
    std::size_t i = 0;
    std::uintptr_t version;
    for (; i < m_size; i++) {
      Bucket *current_bucket = get_bucket(hash, i);
      std::uintptr_t vs = current_bucket->vs.load();
    loadBegin:
      std::uintptr_t state = get_state(vs);
      version = get_version(vs);
      if (state != S_EMPTY) {
        continue;
      } else if (state == S_EMPTY) {
        if (current_bucket->vs.compare_exchange_strong(
                vs, embed_state(vs, S_BUSY))) {
          break;
        } else {
          goto loadBegin;
        }
      }
    }
    Bucket *current_bucket = get_bucket(hash, i);
    current_bucket->key.store(key, std::memory_order_relaxed);

    while (true) {
      current_bucket->vs.store(embed_state(version, S_VISIBLE));
      cond_raise_bound(hash, i);
      current_bucket->vs.store(embed_state(version, S_INSERTING));
      bool r = assist(key, hash, i, version);
      std::uintptr_t vs = current_bucket->vs.load();
      std::uintptr_t expected_vs = embed_state(version, S_COLLIDED);
      if (vs != expected_vs) {
        return true;
      }
      if (!r) {
        cond_lower_bound(hash, i);
        current_bucket->key.store(KT::NullKey, std::memory_order_relaxed);
        current_bucket->vs.store(embed_state(version + 1, S_EMPTY));
        return false;
      }
      version++;
    }
  }

  bool remove(const K &key, std::size_t thread_id) {
    const std::size_t hash = KT::hash(key) & m_size_mask;
    std::uintptr_t pb = m_probe_bounds[hash].load();
    std::size_t max = get_probe_bound(pb);
    for (std::size_t i = 0; i <= max; i++) {
      Bucket *current_bucket = get_bucket(hash, i);
      std::uintptr_t vs = current_bucket->vs.load();
      std::uintptr_t state = get_state(vs);
      const K cur_key = current_bucket->key.load(std::memory_order_relaxed);
      if (state == S_MEMBER and cur_key == key) {
        if (current_bucket->vs.compare_exchange_strong(
                vs, embed_state(vs, S_BUSY))) {
          cond_lower_bound(hash, i);
          std::uintptr_t version = get_version(vs);
          current_bucket->key.store(KT::NullKey, std::memory_order_relaxed);
          current_bucket->vs.store(embed_state(version + 1, S_EMPTY));
          return true;
        }
      }
    }
    return false;
  }

  void print_table() {
    std::cout << "*********************" << std::endl;
    for (std::size_t i = 0; i < m_size; i++) {
      Bucket *current_bucket = &m_table[i];
      const K key = current_bucket->key.load();
      const std::uintptr_t vs = current_bucket->vs.load();
      const std::size_t hash = (KT::hash(key) & m_size_mask);
      bool empty = key == KT::NullKey;
      const std::uintptr_t pb = m_probe_bounds[i].load();
      if (empty) {
        std::cout << i << " -  Key: Empty";
      } else {
        std::cout << i << " -  Key: " << key;
      }
      std::cout << " version: " << get_version(vs)
                << " state: " << get_state(vs);

      if (empty) {
        std::cout << " hash: NA";
      } else {
        std::cout << " hash: " << hash;
      }
      std::cout << " raw bound: " << pb
                << " probe bound: " << get_probe_bound(pb)
                << " scanning: " << get_scanning(pb) << std::endl;
    }
    std::cout << "*********************" << std::endl;
  }
};
}
