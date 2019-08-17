#pragma once

/*
An implementation of original K-CAS based on
"A practical multi-word compare-and-swap operation"""
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
#include <algorithm>
#include <atomic>
#include <cassert>
#include <cstdint>
#include <type_traits>

namespace concurrent_data_structures {

template <class Allocator, class MemReclaimer, std::size_t N>
class Harris_KCAS {
public:
  class KCASDescriptor;

private:
  struct RDCSSDescriptor;
  typedef typename MemReclaimer::RecordHandle RecordHandle;
  typedef typename MemReclaimer::RecordBase RecordBase;

  static const std::intptr_t S_KCAS_BIT = 0x1;
  static const std::intptr_t S_RDCSS_BIT = 0x2;

  // Horrific type to represent a word (pointer or shifted value) or some kind
  // of descriptor.
  union DescriptorUnion {

    std::intptr_t bits;
    RDCSSDescriptor *rdcss_descriptor;
    KCASDescriptor *kcas_descriptor;

    DescriptorUnion() noexcept : bits(0) {}
    DescriptorUnion(std::intptr_t bits) noexcept : bits(bits) {}
    DescriptorUnion(RDCSSDescriptor *desc) noexcept : rdcss_descriptor(desc) {}
    DescriptorUnion(KCASDescriptor *desc) noexcept : kcas_descriptor(desc) {}

    static bool is_rdcss(DescriptorUnion desc) {
      return (desc.bits & S_RDCSS_BIT) == S_RDCSS_BIT;
    }

    static bool is_kcas(DescriptorUnion desc) {
      return (desc.bits & S_KCAS_BIT) == S_KCAS_BIT;
    }

    static bool is_bits(DescriptorUnion desc) {
      return !is_kcas(desc) and !is_rdcss(desc);
    }

    static DescriptorUnion mask_bits(DescriptorUnion desc) {
      return DescriptorUnion{(desc.bits & ~(S_KCAS_BIT | S_RDCSS_BIT))};
    }

    static DescriptorUnion make_rdcss(DescriptorUnion desc) {
      return DescriptorUnion{DescriptorUnion::mask_bits(desc).bits |
                             S_RDCSS_BIT};
    }

    static DescriptorUnion from_rdcss(RDCSSDescriptor *desc) {
      return make_rdcss(DescriptorUnion(desc));
    }

    static DescriptorUnion make_kcas(DescriptorUnion desc) {
      return DescriptorUnion{DescriptorUnion::mask_bits(desc).bits |
                             S_KCAS_BIT};
    }

    static DescriptorUnion from_kcas(KCASDescriptor *desc) {
      return make_kcas(DescriptorUnion(desc));
    }

    static DescriptorUnion make_bits(DescriptorUnion desc) {
      return DescriptorUnion::mask_bits(desc);
    }
  };

  enum class KCASDescriptorStatus {
    UNDECIDED,
    SUCCEEDED,
    FAILED,
  };

  struct RDCSSDescriptor : public RecordBase {
    std::atomic<DescriptorUnion> *data_location;
    DescriptorUnion before, kcas_desc;
    std::atomic<KCASDescriptorStatus> *status_location;

    RDCSSDescriptor(std::atomic<DescriptorUnion> *data_location,
                    DescriptorUnion before, DescriptorUnion kcas_desc,
                    std::atomic<KCASDescriptorStatus> *status_location)
        : data_location(data_location), before(before), kcas_desc(kcas_desc),
          status_location(status_location) {}

    // It's implied we are already protecting "desc" with
    // some memory relcaiming system.
    static DescriptorUnion rdcss(DescriptorUnion input_desc,
                                 RecordHandle &found_rdcss) {
      assert(DescriptorUnion::is_rdcss(input_desc));
      RDCSSDescriptor *input_rdcss =
          DescriptorUnion::mask_bits(input_desc).rdcss_descriptor;
      assert(DescriptorUnion::is_bits(input_rdcss->before.bits));
      while (true) {
        DescriptorUnion current = input_rdcss->data_location->load();
        if (DescriptorUnion::is_rdcss(current)) {
          if (!found_rdcss.try_protect(
                  current.rdcss_descriptor, *input_rdcss->data_location,
                  [](DescriptorUnion desc) { return desc.rdcss_descriptor; })) {
            continue;
          }
          rdcss_complete(current);
          continue;
        }
        if (current.bits != input_rdcss->before.bits) {
          assert(!DescriptorUnion::is_rdcss(current));
          return current;
        }

        bool success = input_rdcss->data_location->compare_exchange_strong(
            current, input_desc);
        if (success) {
          rdcss_complete(input_desc);
          return input_rdcss->before;
        }
      }
    }

    static DescriptorUnion
    rdcss_read(const std::atomic<DescriptorUnion> *location,
               const std::memory_order memory_order,
               RecordHandle &found_rdcss) {
      while (true) {
        DescriptorUnion current = location->load(memory_order);
        if (DescriptorUnion::is_rdcss(current)) {
          if (!found_rdcss.try_protect(
                  current.rdcss_descriptor, *location,
                  [](DescriptorUnion desc) { return desc.rdcss_descriptor; })) {
            continue;
          }
          rdcss_complete(current);
        } else {
          return current;
        }
      }
    }

    static void rdcss_complete(DescriptorUnion input_desc) {
      assert(DescriptorUnion::is_rdcss(input_desc));
      RDCSSDescriptor *input_rdcss_desc =
          DescriptorUnion::mask_bits(input_desc).rdcss_descriptor;
      KCASDescriptorStatus status = input_rdcss_desc->status_location->load();
      if (status == KCASDescriptorStatus::UNDECIDED) {
        /*bool _ = */ input_rdcss_desc->data_location->compare_exchange_strong(
            input_desc, input_rdcss_desc->kcas_desc);
      } else {
        /*bool _ =*/input_rdcss_desc->data_location->compare_exchange_strong(
            input_desc, input_rdcss_desc->before);
      }
    }
  };

  struct EntryPayload {
    std::atomic<DescriptorUnion> *data_location;
    DescriptorUnion before, desired;
  };

  static const std::size_t KCASShift = 2;
  MemReclaimer *m_reclaimer;

  bool cas_internal(const std::size_t thread_id, const DescriptorUnion desc,
                    ReclaimerPin<MemReclaimer> &pin, RecordHandle &found_kcas,
                    RecordHandle &our_rdcss, RecordHandle &found_rdcss,
                    bool help = false) {
    assert(DescriptorUnion::is_kcas(desc));
    KCASDescriptor *kcas_descriptor =
        DescriptorUnion::mask_bits(desc).kcas_descriptor;
    // If there is work to do, go ahead!
    const std::size_t num_entries = kcas_descriptor->m_num_entries;
    KCASDescriptorStatus status = KCASDescriptorStatus::SUCCEEDED;
    if (kcas_descriptor->m_status.load() == KCASDescriptorStatus::UNDECIDED) {
      for (std::size_t i = help ? 1 : 0;
           i < num_entries and status == KCASDescriptorStatus::SUCCEEDED; i++) {
        EntryPayload &payload = kcas_descriptor->m_descriptors[i];
      retry:
        RDCSSDescriptor *rdcss_desc =
            new (m_reclaimer->malloc(sizeof(RDCSSDescriptor)))
                RDCSSDescriptor(payload.data_location, payload.before, desc,
                                &kcas_descriptor->m_status);
        DescriptorUnion rdcss = DescriptorUnion::from_rdcss(rdcss_desc);
        assert(DescriptorUnion::mask_bits(rdcss).bits ==
               (std::intptr_t)rdcss_desc);
        our_rdcss.set(rdcss_desc);
        DescriptorUnion value = RDCSSDescriptor::rdcss(rdcss, found_rdcss);
        // Did it work? Is there a K-CAS?
        if (DescriptorUnion::is_kcas(value)) {
          // Is it our K-CAS? Why? It could be someone helping us.
          if (value.bits != desc.bits) {
            if (!found_kcas.try_protect(value.kcas_descriptor,
                                        *payload.data_location,
                                        [](DescriptorUnion desc) {
                                          return desc.kcas_descriptor;
                                        })) {
              continue;
            }
            m_reclaimer->free(rdcss_desc);
            cas_internal(thread_id, value, pin, found_kcas, our_rdcss,
                         found_rdcss, true);
            goto retry;
          } else {
            // Someone put our K-CAS there, free our RDCSS desc.
            assert(value.bits == desc.bits);
            m_reclaimer->free(rdcss_desc);
          }
        } else if (value.bits != rdcss_desc->before.bits) {
          // We didn't win, free our one and exit.
          m_reclaimer->free(rdcss_desc);
          status = KCASDescriptorStatus::FAILED;
        } else {
          // We won, retire our descriptor as it could still be viewed.
          pin.retire(our_rdcss);
        }
      }
      KCASDescriptorStatus expected_status = KCASDescriptorStatus::UNDECIDED;
      kcas_descriptor->m_status.compare_exchange_strong(expected_status,
                                                        status);
    }
    // Phase 2.
    bool succeeded =
        kcas_descriptor->m_status.load() == KCASDescriptorStatus::SUCCEEDED;
    for (std::size_t i = 0; i < num_entries; i++) {
      DescriptorUnion new_value =
          succeeded ? kcas_descriptor->m_descriptors[i].desired
                    : kcas_descriptor->m_descriptors[i].before;
      DescriptorUnion expected = desc;
      kcas_descriptor->m_descriptors[i].data_location->compare_exchange_strong(
          expected, new_value);
    }
    return succeeded;
  }

public:
  // Class to wrap around the types being KCAS'd
  template <class Type> class KCASEntry {
  private:
    union ItemBitsUnion {
      Type item;
      std::intptr_t raw_bits;
    };

    std::atomic<DescriptorUnion> m_entry;
    static std::intptr_t from_union(DescriptorUnion desc) {
      return (ItemBitsUnion{desc.bits}).item;
    }
    static DescriptorUnion to_union(const std::intptr_t &inner) {
      return DescriptorUnion{(ItemBitsUnion{inner}).raw_bits};
    }

    //    union ItemBitsUnion {
    //      ItemBitsUnion() : raw_bits(0) {}
    //      Type item;
    //      std::intptr_t raw_bits;
    //    };

    //    std::atomic<DescriptorUnion> m_entry;
    //    static Type to_union(std::intptr_t raw_bits) {
    //      ItemBitsUnion ibu;
    //      ibu.raw_bits = raw_bits;
    //      return ibu.item;
    //    }
    //    static std::intptr_t from_union(const DescriptorUnion &inner) {
    //      ItemBitsUnion ibu;
    //      ibu.item = inner.bits;
    //      return ibu.raw_bits;
    //    }
    friend class Harris_KCAS;
    friend class KCASDescriptor;
  };

  class KCASDescriptor : public RecordBase {
  private:
    std::size_t state, thread_id;
    std::atomic<KCASDescriptorStatus> m_status;
    std::size_t m_num_entries;
    EntryPayload m_descriptors[N];

    KCASDescriptor() = delete;
    KCASDescriptor(const KCASDescriptor &rhs) = delete;
    KCASDescriptor &operator=(const KCASDescriptor &rhs) = delete;
    KCASDescriptor &operator=(KCASDescriptor &&rhs) = delete;
    KCASDescriptor(const std::size_t descriptor_size)
        : state(0), m_status(KCASDescriptorStatus::UNDECIDED),
          m_num_entries(0) {}

  public:
    template <class ValType>
    void add_value(KCASEntry<ValType> *location, const ValType &before,
                   const ValType &desired) {
      DescriptorUnion before_desc = KCASEntry<ValType>::to_union(before);
      before_desc.bits <<= KCASShift;
      assert(DescriptorUnion::is_bits(before_desc));
      DescriptorUnion desired_desc = KCASEntry<ValType>::to_union(desired);
      desired_desc.bits <<= KCASShift;
      assert(DescriptorUnion::is_bits(desired_desc));
      const std::size_t cur_entry = m_num_entries++;
      assert(cur_entry < N);
      m_descriptors[cur_entry].before = before_desc;
      m_descriptors[cur_entry].desired = desired_desc;
      m_descriptors[cur_entry].data_location = &location->m_entry;
    }
    template <class PtrType>
    void
    add_ptr(const KCASEntry<PtrType> *location, const PtrType &before,
            const PtrType &desired,
            const std::memory_order success_order = std::memory_order_seq_cst,
            const std::memory_order fail_order = std::memory_order_seq_cst) {
      const DescriptorUnion before_desc = KCASEntry<PtrType>::to_union(before);
      assert(DescriptorUnion::is_bits(before_desc));
      const DescriptorUnion desired_desc =
          KCASEntry<PtrType>::to_union(desired);
      assert(DescriptorUnion::is_bits(desired_desc));
      const std::size_t cur_entry = m_num_entries++;
      assert(cur_entry < N);
      m_descriptors[cur_entry].before = before_desc;
      m_descriptors[cur_entry].desired = desired_desc;
      m_descriptors[cur_entry].data_location = &location->m_entry;
    }
    friend class Harris_KCAS;
  };

  // 4 ref explanation: 1 for K-CAS descriptor, 1 for any K-CAS descriptor
  // we find, 1 for our RDCSS, nd 1 for any RDCSS descriptors
  // we find trying to install our own.
  Harris_KCAS(const std::size_t threads, MemReclaimer *reclaimer)
      : m_reclaimer(reclaimer) {}

  KCASDescriptor *create_descriptor(const std::size_t descriptor_size,
                                    const std::size_t thread_id) {
    KCASDescriptor *desc = new (m_reclaimer->malloc(sizeof(KCASDescriptor)))
        KCASDescriptor(descriptor_size);
    desc->thread_id = thread_id;
    return desc;
  }

  void free_descriptor(KCASDescriptor *desc) {
    desc->state = 2;
    m_reclaimer->free(desc);
  }

  bool cas(const std::size_t thread_id, ReclaimerPin<MemReclaimer> &pin,
           KCASDescriptor *desc, bool record = false) {
    std::sort(desc->m_descriptors, desc->m_descriptors + desc->m_num_entries,
              [](const EntryPayload &lhs, const EntryPayload &rhs) -> bool {
                return lhs.data_location < rhs.data_location;
              });
    // 4 references during our K-CAS
    RecordHandle our_kcas = pin.get_rec(), found_kcas = pin.get_rec(),
                 our_rdcss = pin.get_rec(), found_rdcss = pin.get_rec();
    our_kcas.set(desc);
    bool res = cas_internal(thread_id, DescriptorUnion::from_kcas(desc), pin,
                            found_kcas, our_rdcss, found_rdcss);
    desc->state = 1;
    pin.retire(our_kcas);
    return res;
  }

  template <class ValType>
  ValType
  read_value(const std::size_t thread_id, ReclaimerPin<MemReclaimer> &pin,
             const KCASEntry<ValType> *location,
             const std::memory_order memory_order = std::memory_order_seq_cst) {
    static_assert(!std::is_pointer<ValType>::value and
                      !std::is_reference<ValType>::value,
                  "Type is not a value.");
    RecordHandle found_kcas = pin.get_rec(), our_rdcss = pin.get_rec(),
                 found_rdcss = pin.get_rec();
    while (true) {
      DescriptorUnion desc = RDCSSDescriptor::rdcss_read(
          &location->m_entry, memory_order, found_kcas);
      // Could still be a K-CAS descriptor.
      if (DescriptorUnion::is_kcas(desc)) {
        cas_internal(thread_id, desc, pin, found_kcas, our_rdcss, found_rdcss,
                     true);
        continue;
      }
      assert(DescriptorUnion::is_bits(desc));
      desc.bits >>= KCASShift;
      return KCASEntry<ValType>::from_union(desc);
    }
  }

  template <class PtrType>
  PtrType
  read_ptr(const std::size_t thread_id, ReclaimerPin<MemReclaimer> &pin,
           const KCASEntry<PtrType> *location,
           const std::memory_order memory_order = std::memory_order_seq_cst) {
    static_assert(std::is_pointer<PtrType>::value, "Type is not a pointer.");
    while (true) {
      RecordHandle found_kcas = pin.get_rec(), our_rdcss = pin.get_rec(),
                   found_rdcss = pin.get_rec();
      DescriptorUnion desc =
          RDCSSDescriptor::rdcss_read(location, memory_order, found_kcas);
      // Could still be a K-CAS descriptor.
      if (DescriptorUnion::is_kcas(desc)) {
        cas_internal(thread_id, desc, pin, found_kcas, our_rdcss, found_rdcss,
                     true);
        continue;
      }
      assert(DescriptorUnion::is_bits(desc));
      return KCASEntry<PtrType>::from_union(desc);
    }
  }

  template <class ValType>
  void write_value(const std::size_t thread_id, KCASEntry<ValType> *location,
                   const ValType &val, const std::memory_order memory_order =
                                           std::memory_order_seq_cst) {
    static_assert(!std::is_pointer<ValType>::value and
                      !std::is_reference<ValType>::value,
                  "Type is not a value.");
    DescriptorUnion desc = KCASEntry<ValType>::to_union(val);
    desc.bits <<= KCASShift;
    assert(DescriptorUnion::is_bits(desc));
    location->m_entry.store(desc);
  }

  template <class PtrType>
  void
  write_ptr(const std::size_t thread_id, KCASEntry<PtrType> *location,
            const PtrType &ptr,
            const std::memory_order memory_order = std::memory_order_seq_cst) {
    static_assert(std::is_pointer<PtrType>::value, "Type is not a pointer.");
    DescriptorUnion desc = KCASEntry<PtrType>::to_union(ptr);
    assert(DescriptorUnion::is_bits(desc));
    location->m_entry.store(desc, memory_order);
  }
};
}
