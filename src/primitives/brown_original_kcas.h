/**
 * Multiword compare and swap implementation from single word CAS,
 * based on Tim Harris' algorithm.
 * Copyright 2016 Trevor Brown (me [at] tbrown [dot] pro).
 * Modified by Robert Kelly.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef KCAS_REUSE_H
#define KCAS_REUSE_H

#include "mem-reclaimer/reclaimer.h"
#include "primitives/cache_utils.h"
#include <algorithm>
#include <atomic>
#include <cassert>
#include <cstring>
#include <sstream>
#include <stdint.h>
#include <string>
#include <type_traits>

typedef intptr_t tagptr_t;
typedef intptr_t seqbits_t;

#define DESC_INIT_ALL(descArray, macro_seqBitsNew, numProcesses)               \
  {                                                                            \
    for (std::size_t i = 0; i < (numProcesses); ++i) {                         \
      (descArray)[i].seqBits = macro_seqBitsNew(0);                            \
    }                                                                          \
  }

///**
// * pointer to descriptor (listed least to most significant):
// * 1-bit: is descriptor
// * 10-bits: thread id
// * remaining bits: sequence number (for avoiding the ABA problem)
// */
// typedef intptr_t tagptr_t;
// typedef intptr_t seqbits_t;

/**
 * seqbits_t corresponds to the seqBits field of the descriptor.
 * it contains the mutable fields of the descriptor and a sequence number.
 * the width, offset and mask for the sequence number is defined below.
 * this sequence number width, offset and mask are also shared by tagptr_t.
 *
 * in particular, for any tagptr_t x and seqbits_t y, the sequence numbers
 * in x and y are equal iff x&MASK_SEQ == y&MASK_SEQ (despite the differing
 * types of x and y).
 *
 * tagptr_t consists of a triple <seq, tid, testbit>.
 * these three fields are defined by the TAGPTR_ macros below.
 */

#ifndef WIDTH_SEQ
#define WIDTH_SEQ 48
#endif
#define OFFSET_SEQ 14
#define MASK_SEQ                                                               \
  ((uintptr_t)((1LL << WIDTH_SEQ) - 1)                                         \
   << OFFSET_SEQ) /* cast to avoid signed bit shifting */
#define UNPACK_SEQ(tagptrOrSeqbits)                                            \
  (((uintptr_t)(tagptrOrSeqbits)) >> OFFSET_SEQ)

#define TAGPTR_OFFSET_USER 0
#define TAGPTR_OFFSET_TID 3
#define TAGPTR_MASK_USER                                                       \
  ((1 << TAGPTR_OFFSET_TID) - 1) /* assumes TID is next field after USER */
#define TAGPTR_MASK_TID (((1 << OFFSET_SEQ) - 1) & (~(TAGPTR_MASK_USER)))
#define TAGPTR_UNPACK_TID(tagptr)                                              \
  ((int)((((tagptr_t)(tagptr)) & TAGPTR_MASK_TID) >> TAGPTR_OFFSET_TID))
#define TAGPTR_UNPACK_PTR(descArray, tagptr)                                   \
  (&(descArray)[TAGPTR_UNPACK_TID((tagptr))])
#define TAGPTR_NEW(tid, seqBits, userBits)                                     \
  ((tagptr_t)(((UNPACK_SEQ(seqBits)) << OFFSET_SEQ) |                          \
              ((tid) << TAGPTR_OFFSET_TID) |                                   \
              (tagptr_t)(userBits) << TAGPTR_OFFSET_USER))
// assert: there is no thread with tid DUMMY_TID that ever calls TAGPTR_NEW
#define LAST_TID (TAGPTR_MASK_TID >> TAGPTR_OFFSET_TID)
#define TAGPTR_STATIC_DESC(id) ((tagptr_t)TAGPTR_NEW(LAST_TID - 1 - id, 0))
#define TAGPTR_DUMMY_DESC(id) ((tagptr_t)TAGPTR_NEW(LAST_TID, id << OFFSET_SEQ))

#define comma ,

#define SEQBITS_UNPACK_FIELD(seqBits, mask, offset)                            \
  ((((seqbits_t)(seqBits)) & (mask)) >> (offset))
// TODO: make more efficient version "SEQBITS_CAS_BIT"
// TODO: change sequence # unpacking to masking for quick comparison
// note: if there is only one subfield besides seq#, then the third if-block is
// redundant, and you should just return false if the cas fails, since the only
// way the cas fails and the field being cas'd contains still old is if the
// sequence number has changed.
#define SEQBITS_CAS_FIELD(successBit, fldSeqBits, snapSeqBits, oldval, val,    \
                          mask, offset)                                        \
  {                                                                            \
    seqbits_t __v = (fldSeqBits);                                              \
    while (1) {                                                                \
      if (UNPACK_SEQ(__v) != UNPACK_SEQ((snapSeqBits))) {                      \
        (successBit) = false;                                                  \
        break;                                                                 \
      }                                                                        \
      if (((successBit) = __sync_bool_compare_and_swap(                        \
               &(fldSeqBits), (__v & ~(mask)) | (oldval),                      \
               (__v & ~(mask)) | ((val) << (offset))))) {                      \
        break;                                                                 \
      }                                                                        \
      __v = (fldSeqBits);                                                      \
      if (SEQBITS_UNPACK_FIELD(__v, (mask), (offset)) != (oldval)) {           \
        (successBit) = false;                                                  \
        break;                                                                 \
      }                                                                        \
    }                                                                          \
  }
// TODO: change sequence # unpacking to masking for quick comparison
// note: SEQBITS_FAA_FIELD would be very similar to SEQBITS_CAS_FIELD; i think
// one would simply delete the last if block and change the new val from
// (val)<<offset to (val&mask)+1.
#define SEQBITS_WRITE_FIELD(fldSeqBits, snapSeqBits, val, mask, offset)        \
  {                                                                            \
    seqbits_t __v = (fldSeqBits);                                              \
    while (UNPACK_SEQ(__v) == UNPACK_SEQ((snapSeqBits)) &&                     \
           SEQBITS_UNPACK_FIELD(__v, (mask), (offset)) != (val) &&             \
           !__sync_bool_compare_and_swap(                                      \
               &(fldSeqBits), __v, (__v & ~(mask)) | ((val) << (offset)))) {   \
      __v = (fldSeqBits);                                                      \
    }                                                                          \
  }
#define SEQBITS_WRITE_BIT(fldSeqBits, snapSeqBits, mask)                       \
  {                                                                            \
    seqbits_t __v = (fldSeqBits);                                              \
    while (                                                                    \
        UNPACK_SEQ(__v) == UNPACK_SEQ((snapSeqBits)) && !(__v & (mask)) &&     \
        !__sync_bool_compare_and_swap(&(fldSeqBits), __v, (__v | (mask)))) {   \
      __v = (fldSeqBits);                                                      \
    }                                                                          \
  }

// WARNING: uses a GCC extension "({ })". to get rid of this, use an inline
// function.
#define DESC_SNAPSHOT(descType, descArray, descDest, tagptr, sz)               \
  ({                                                                           \
    descType *__src = TAGPTR_UNPACK_PTR((descArray), (tagptr));                \
    memcpy((descDest), __src, (sz));                                           \
    __asm__ __volatile__(                                                      \
        "" ::                                                                  \
            : "memory"); /* prevent compiler from reordering read of           \
                            __src->seqBits before (at least the reading        \
                            portion of) the memcpy */                          \
    (UNPACK_SEQ(__src->seqBits) == UNPACK_SEQ((tagptr)));                      \
  })
#define DESC_READ_FIELD(successBit, fldSeqBits, tagptr, mask, offset)          \
  ({                                                                           \
    seqbits_t __seqBits = (fldSeqBits);                                        \
    successBit = (__seqBits & MASK_SEQ) == ((tagptr)&MASK_SEQ);                \
    SEQBITS_UNPACK_FIELD(__seqBits, (mask), (offset));                         \
  })
#define DESC_NEW(descArray, macro_seqBitsNew, tid)                             \
  &(descArray)[(tid)];                                                         \
  { /* note: only the process invoking this following macro can change the     \
       sequence# */                                                            \
    seqbits_t __v = (descArray)[(tid)].seqBits;                                \
    (descArray)[(tid)].seqBits = macro_seqBitsNew(__v);                        \
    /*__sync_synchronize();*/                                                  \
  }
#define DESC_INITIALIZED(descArray, tid)                                       \
  (descArray)[(tid)].seqBits += (1 << OFFSET_SEQ);

#define kcastagptr_t uintptr_t
#define rdcsstagptr_t uintptr_t
#define rdcssptr_t rdcssdesc_t *
#define casword_t uintptr_t

#define KCAS_STATE_UNDECIDED 0
#define KCAS_STATE_SUCCEEDED 4
#define KCAS_STATE_FAILED 8

#define KCAS_LEFTSHIFT 2

#define RDCSS_TAGBIT 0x1
#define KCAS_TAGBIT 0x2

static inline bool isRdcss(casword_t val) { return (val & RDCSS_TAGBIT); }

static inline bool isKcas(casword_t val) { return (val & KCAS_TAGBIT); }

#define BOOL_CAS __sync_bool_compare_and_swap
#define VAL_CAS __sync_val_compare_and_swap

namespace concurrent_data_structures {

struct rdcssdesc_t {
  volatile seqbits_t seqBits;
  casword_t volatile *addr1;
  casword_t old1;
  casword_t volatile *addr2;
  casword_t old2;
  casword_t new2;
  const static int size = sizeof(seqBits) + sizeof(addr1) + sizeof(old1) +
                          sizeof(addr2) + sizeof(old2) + sizeof(new2);
  volatile char padding[128 + ((64 - size % 64) %
                               64)]; // add padding to prevent false sharing
};

struct kcasentry_t { // just part of kcasdesc_t, not a standalone descriptor
  casword_t volatile *addr;
  casword_t oldval;
  casword_t newval;
};

template <class Allocator, class MemReclaimer, std::size_t N>
class BrownOriginal {
  /**
   * Data definitions
   */
public:
  template <class Type> class KCASEntry {
  private:
    union ItemBitsUnion {
      ItemBitsUnion() : raw_bits(0) {}
      //      ItemBitsUnion(const Type &type) : item(type) {}
      //      ItemBitsUnion(const std::uintptr_t bits) : raw_bits(bits) {}
      Type item;
      std::uintptr_t raw_bits;
    };

    casword_t volatile m_entry;
    static Type from_raw_bits(std::intptr_t raw_bits) {
      ItemBitsUnion ibu;
      ibu.raw_bits = raw_bits;
      return ibu.item;
    }
    static std::intptr_t to_raw_bits(const Type &inner) {
      ItemBitsUnion ibu;
      ibu.item = inner;
      return ibu.raw_bits;
    }
    friend class BrownOriginal;
  };

  class KCASDescriptor {

  public:
    static const std::size_t S_NUM_THREADS = 144;
    volatile seqbits_t seqBits;
    casword_t numEntries;
    kcasentry_t entries[N];
    const static int size =
        sizeof(seqBits) + sizeof(numEntries) + sizeof(entries);
    volatile char padding[128 + ((64 - size % 64) %
                                 64)]; // add padding to prevent false sharing

    template <class ValType>
    void add_value(KCASEntry<ValType> *location, const ValType &before,
                   const ValType &desired) {
      std::uintptr_t before_raw_bits = KCASEntry<ValType>::to_raw_bits(before);
      casword_t before_desc{before_raw_bits << KCAS_LEFTSHIFT};
      std::uintptr_t desired_raw_bits =
          KCASEntry<ValType>::to_raw_bits(desired);
      casword_t desired_desc{desired_raw_bits << KCAS_LEFTSHIFT};
      const std::size_t cur_entry = numEntries++;
      //      assert(cur_entry < m_descriptor_size);
      entries[cur_entry].oldval = before_desc;
      entries[cur_entry].newval = desired_desc;
      entries[cur_entry].addr = &location->m_entry;
    }
    template <class PtrType>
    void add_ptr(const KCASEntry<PtrType> *location, const PtrType &before,
                 const PtrType &desired) {
      std::uintptr_t before_raw_bits = KCASEntry<PtrType>::to_raw_bits(before);
      casword_t before_desc{before_raw_bits};
      std::uintptr_t desired_raw_bits =
          KCASEntry<PtrType>::to_raw_bits(desired);
      casword_t desired_desc{desired_raw_bits};
      const std::size_t cur_entry = numEntries++;
      entries[cur_entry].oldval = before_desc;
      entries[cur_entry].newval = desired_desc;
      entries[cur_entry].addr = &location->m_entry;
    }

    //    void addValAddr(casword_t *addr, casword_t oldval, casword_t newval) {
    //      entries[numEntries].addr = addr;
    //      entries[numEntries].oldval = oldval << KCAS_LEFTSHIFT;
    //      entries[numEntries].newval = newval << KCAS_LEFTSHIFT;
    //      ++numEntries;
    //      // std::cout<<"addr="<<(size_t) addr<<" oldval="<<oldval<<"
    //      // newval="<<newval<<" numEntries="<<std::endl;
    //    }

    //    void addPtrAddr(casword_t *addr, casword_t oldval, casword_t newval) {
    //      entries[numEntries].addr = addr;
    //      entries[numEntries].oldval = oldval;
    //      entries[numEntries].newval = newval;
    //      ++numEntries;
    //    }
  };

private:
// descriptor reduction algorithm
#define KCAS_SEQBITS_OFFSET_STATE 0
#define KCAS_SEQBITS_MASK_STATE 0xf
#define KCAS_SEQBITS_NEW(seqBits)                                              \
  ((((seqBits)&MASK_SEQ) + (1 << OFFSET_SEQ)) |                                \
   (KCAS_STATE_UNDECIDED << KCAS_SEQBITS_OFFSET_STATE))
#define RDCSS_SEQBITS_NEW(seqBits) (((seqBits)&MASK_SEQ) + (1 << OFFSET_SEQ))
  volatile char __padding_desc[128];
  KCASDescriptor kcasDescriptors[KCASDescriptor::S_NUM_THREADS]
      __attribute__((aligned(64)));
  rdcssdesc_t rdcssDescriptors[KCASDescriptor::S_NUM_THREADS]
      __attribute__((aligned(64)));
  volatile char __padding_desc3[128];

  void internalWritePtr(casword_t volatile *addr, casword_t val);
  void internalWriteVal(casword_t volatile *addr, casword_t val);
  casword_t internalReadPtr(const int tid, const casword_t volatile *addr);
  casword_t internalReadVal(const int tid, const casword_t volatile *addr);

  /**
   * Function declarations
   */
public:
  // Class to wrap around the types being KCAS'd

  BrownOriginal(const std::size_t num_threads, MemReclaimer *reclaimer);
  int cas(const int tid, ReclaimerPin<MemReclaimer> &pin, KCASDescriptor *ptr,
          bool debug = false);
  void free_descriptor(KCASDescriptor *desc) {}
  KCASDescriptor *create_descriptor(const std::size_t descriptor_size,
                                    const int tid);

  template <class ValType>
  ValType
  read_value(const std::size_t thread_id, ReclaimerPin<MemReclaimer> &pin,
             const KCASEntry<ValType> *location,
             const std::memory_order memory_order = std::memory_order_seq_cst) {
    static_assert(!std::is_pointer<ValType>::value, "Type is not a value.");
    return KCASEntry<ValType>::from_raw_bits(
        internalReadVal(thread_id, &location->m_entry));
  }

  template <class PtrType>
  PtrType
  read_ptr(const std::size_t thread_id, ReclaimerPin<MemReclaimer> &pin,
           const KCASEntry<PtrType> *location,
           const std::memory_order memory_order = std::memory_order_seq_cst) {
    static_assert(std::is_pointer<PtrType>::value, "Type is not a pointer.");
    return KCASEntry<PtrType>::from_raw_bits(
        internalReadPtr(thread_id, &location->m_entry));
  }

  template <class ValType>
  void write_value(const std::size_t thread_id, KCASEntry<ValType> *location,
                   const ValType &val, const std::memory_order memory_order =
                                           std::memory_order_seq_cst) {
    static_assert(!std::is_pointer<ValType>::value, "Type is not a value.");
    intptr_t raw_bits = KCASEntry<ValType>::to_raw_bits(val);
    internalWriteVal(&location->m_entry, raw_bits);
  }

  template <class PtrType>
  void
  write_ptr(const std::size_t thread_id, KCASEntry<PtrType> *location,
            const PtrType &ptr,
            const std::memory_order memory_order = std::memory_order_seq_cst) {
    static_assert(std::is_pointer<PtrType>::value, "Type is not a pointer.");
    intptr_t raw_bits = KCASEntry<PtrType>::to_raw_bits(ptr);
    internalWritePtr(&location->m_entry, raw_bits);
  }

  template <class ValType>
  bool compare_exchange_weak_value(
      const std::size_t thread_id, ReclaimerPin<MemReclaimer> &pin,
      KCASEntry<ValType> *location, ValType &expected, const ValType &desired,
      std::memory_order success = std::memory_order_seq_cst,
      std::memory_order fail = std::memory_order_seq_cst) {
    casword_t before_raw_bits = KCASEntry<ValType>::to_raw_bits(expected);
    casword_t before_desc{before_raw_bits << KCAS_LEFTSHIFT};
    casword_t desired_raw_bits = KCASEntry<ValType>::to_raw_bits(desired);
    casword_t desired_desc{desired_raw_bits << KCAS_LEFTSHIFT};
    bool ret = BOOL_CAS(&location->m_entry, before_desc, desired_desc);
    if (!ret) {
      while (true) {
        if (isRdcss(before_desc)) {
          rdcssdesc_t newSnapshot;
          const int sz = rdcssdesc_t::size;
          if (DESC_SNAPSHOT(rdcssdesc_t, rdcssDescriptors, &newSnapshot,
                            before_desc, sz)) {
            rdcssHelp(before_desc, &newSnapshot, true);
          }
          before_desc = location->m_entry;
          continue;
        }
        // Could still be a K-CAS descriptor.
        if (isKcas(before_desc)) {
          helpOther(thread_id, (kcastagptr_t)before_desc);
          before_desc = location->m_entry;
          continue;
        }
        expected =
            KCASEntry<ValType>::from_raw_bits(before_desc >> KCAS_LEFTSHIFT);
        return false;
      }
    }
    return true;
  }

  template <class ValType>
  bool compare_exchange_strong_value(
      const std::size_t thread_id, ReclaimerPin<MemReclaimer> &pin,
      KCASEntry<ValType> *location, ValType &expected, const ValType &desired,
      std::memory_order success = std::memory_order_seq_cst,
      std::memory_order fail = std::memory_order_seq_cst) {
    casword_t before_raw_bits = KCASEntry<ValType>::to_raw_bits(expected);
    casword_t before_desc{before_raw_bits << KCAS_LEFTSHIFT};
    casword_t desired_raw_bits = KCASEntry<ValType>::to_raw_bits(desired);
    casword_t desired_desc{desired_raw_bits << KCAS_LEFTSHIFT};
    bool ret = BOOL_CAS(&location->m_entry, before_desc, desired_desc);
    if (!ret) {
      while (true) {
        if (isRdcss(before_desc)) {
          rdcssdesc_t newSnapshot;
          const int sz = rdcssdesc_t::size;
          if (DESC_SNAPSHOT(rdcssdesc_t, rdcssDescriptors, &newSnapshot,
                            before_desc, sz)) {
            rdcssHelp(before_desc, &newSnapshot, true);
          }
          before_desc = location->m_entry;
          continue;
        }
        // Could still be a K-CAS descriptor.
        if (isKcas(before_desc)) {
          helpOther(thread_id, (kcastagptr_t)before_desc);
          before_desc = location->m_entry;
          continue;
        }
        expected =
            KCASEntry<ValType>::from_raw_bits(before_desc >> KCAS_LEFTSHIFT);
        return false;
      }
    }
    return true;
  }

  static void kcasdesc_sort(KCASDescriptor *ptr) {
    std::sort(ptr->entries, ptr->entries + ptr->numEntries,
              [](const kcasentry_t &lhs, const kcasentry_t &rhs) -> bool {
                return lhs.addr < rhs.addr;
              });
  }

private:
  bool help(const int tid, kcastagptr_t tagptr, KCASDescriptor *ptr,
            bool helpingOther);
  void helpOther(const int tid, kcastagptr_t tagptr);
  casword_t rdcssRead(const int tid, const casword_t volatile *addr);
  casword_t rdcss(const int tid, rdcssptr_t ptr, rdcsstagptr_t tagptr);
  void rdcssHelp(rdcsstagptr_t tagptr, rdcssptr_t snapshot, bool helpingOther);
  void rdcssHelpOther(rdcsstagptr_t tagptr);
  std::string tagptrToString(casword_t tagptr);
};

class HPInfo {
public:
  void *obj;
  casword_t *ptrToObj;
  HPInfo(void *_obj, casword_t *_ptrToObj) : obj(_obj), ptrToObj(_ptrToObj) {}
};

template <class Allocator, class MemReclaimer, std::size_t N>
void BrownOriginal<Allocator, MemReclaimer, N>::rdcssHelp(rdcsstagptr_t tagptr,
                                                          rdcssptr_t snapshot,
                                                          bool helpingOther) {
  bool readSuccess;
  casword_t v =
      DESC_READ_FIELD(readSuccess, *snapshot->addr1, snapshot->old1,
                      KCAS_SEQBITS_MASK_STATE, KCAS_SEQBITS_OFFSET_STATE);
  if (!readSuccess)
    v = KCAS_STATE_SUCCEEDED; // return;
  // this fix is ad-hoc, but we need to say something general about it

  if (v == KCAS_STATE_UNDECIDED) { // q here (step 4.5)
    BOOL_CAS(snapshot->addr2, (casword_t)tagptr, snapshot->new2);
  } else {
    // the "fuck it i'm done" action (the same action you'd take if the kcas
    // descriptor hung around indefinitely)
    BOOL_CAS(snapshot->addr2, (casword_t)tagptr, snapshot->old2);
  }
}

template <class Allocator, class MemReclaimer, std::size_t N>
void BrownOriginal<Allocator, MemReclaimer, N>::rdcssHelpOther(
    rdcsstagptr_t tagptr) {
  rdcssdesc_t newSnapshot;
  const int sz = rdcssdesc_t::size;
  if (DESC_SNAPSHOT(rdcssdesc_t, rdcssDescriptors, &newSnapshot, tagptr, sz)) {
    rdcssHelp(tagptr, &newSnapshot, true);
  } else {
    // TRACE COUTATOMICTID("helpOther unable to get snapshot of
    // "<<tagptrToString(tagptr)<<endl);
  }
}

template <class Allocator, class MemReclaimer, std::size_t N>
casword_t
BrownOriginal<Allocator, MemReclaimer, N>::rdcss(const int tid, rdcssptr_t ptr,
                                                 rdcsstagptr_t tagptr) {
  casword_t r;
  do {
    r = VAL_CAS(ptr->addr2, ptr->old2, (casword_t)tagptr);
    if (isRdcss(r)) {
      rdcssHelpOther((rdcsstagptr_t)r);
    }
  } while (isRdcss(r));
  // note: we already hold a hp on ptr (/tagptr) by the invariant
  if (r == ptr->old2)
    rdcssHelp(tagptr, ptr, false); // finish our own operation
  return r;
}

template <class Allocator, class MemReclaimer, std::size_t N>
casword_t BrownOriginal<Allocator, MemReclaimer, N>::rdcssRead(
    const int tid, const casword_t volatile *addr) {
  casword_t r;
  do {
    r = *addr;
    //        for (int i=0;i<100;++i) {
    //            r = *addr;
    //            if (!isRdcss(r)) break;
    //        }
    if (isRdcss(r)) {
      rdcssHelpOther((rdcsstagptr_t)r);
    }
  } while (isRdcss(r));
  return r;
}

template <class Allocator, class MemReclaimer, std::size_t N>
BrownOriginal<Allocator, MemReclaimer, N>::BrownOriginal(
    const std::size_t num_threads, MemReclaimer *reclaimer) {
  DESC_INIT_ALL(kcasDescriptors, KCAS_SEQBITS_NEW,
                KCASDescriptor::S_NUM_THREADS);
  DESC_INIT_ALL(rdcssDescriptors, RDCSS_SEQBITS_NEW,
                KCASDescriptor::S_NUM_THREADS);
}

template <class Allocator, class MemReclaimer, std::size_t N>
void BrownOriginal<Allocator, MemReclaimer, N>::helpOther(const int tid,
                                                          kcastagptr_t tagptr) {
  KCASDescriptor newSnapshot;
  const int sz = KCASDescriptor::size;
  // cout<<"size of kcas descriptor is "<<sizeof(kcasdesc_t<K,NPROC>)<<" and
  // sz="<<sz<<endl;
  if (DESC_SNAPSHOT(KCASDescriptor, kcasDescriptors, &newSnapshot, tagptr,
                    sz)) {
    help(tid, tagptr, &newSnapshot, true);
  } else {
    // TRACE COUTATOMICTID("helpOther unable to get snapshot of
    // "<<tagptrToString(tagptr)<<endl);
  }
}

template <class Allocator, class MemReclaimer, std::size_t N>
bool BrownOriginal<Allocator, MemReclaimer, N>::help(const int tid,
                                                     kcastagptr_t tagptr,
                                                     KCASDescriptor *snapshot,
                                                     bool helpingOther) {
  //    TRACE COUTATOMICTID("help tagptr="<<tagptrToString(tagptr)<<"
  //    helpingOther="<<helpingOther<<endl);

  // phase 1: "locking" addresses for this kcas
  int newstate;

  // read state field
  KCASDescriptor *ptr = TAGPTR_UNPACK_PTR(kcasDescriptors, tagptr);
  bool successBit;
  int state =
      DESC_READ_FIELD(successBit, ptr->seqBits, tagptr, KCAS_SEQBITS_MASK_STATE,
                      KCAS_SEQBITS_OFFSET_STATE);
  if (!successBit) {
    // cout<<"failed to read state field "<<tagptrToString(tagptr)<<endl;
    assert(helpingOther);
    return false;
  }

  if (state == KCAS_STATE_UNDECIDED) {
    newstate = KCAS_STATE_SUCCEEDED;
    for (std::size_t i = helpingOther; i < snapshot->numEntries; i++) {
    retry_entry:
      // prepare rdcss descriptor and run rdcss
      rdcssdesc_t *rdcssptr =
          DESC_NEW(rdcssDescriptors, RDCSS_SEQBITS_NEW, tid);
      rdcssptr->addr1 = (casword_t *)&ptr->seqBits;
      rdcssptr->old1 = tagptr; // pass the sequence number (as part of tagptr)
      rdcssptr->old2 = snapshot->entries[i].oldval;
      rdcssptr->addr2 = snapshot->entries[i].addr; // p stopped here (step 2)
      rdcssptr->new2 = (casword_t)tagptr;
      DESC_INITIALIZED(rdcssDescriptors, tid);

      casword_t val;
      val = rdcss(tid, rdcssptr,
                  TAGPTR_NEW(tid, rdcssptr->seqBits, RDCSS_TAGBIT));

      // check for failure of rdcss and handle it
      if (isKcas(val)) {
        // if rdcss failed because of a /different/ kcas, we help it
        if (val != (casword_t)tagptr) {
          helpOther(tid, (kcastagptr_t)val);
          goto retry_entry;
        }
      } else {
        if (val != snapshot->entries[i].oldval) {
          newstate = KCAS_STATE_FAILED;
          break;
        }
      }
    }
    //        SEQBITS_WRITE_FIELD(ptr->seqBits, snapshot->seqBits, newstate,
    //        KCAS_SEQBITS_MASK_STATE, KCAS_SEQBITS_OFFSET_STATE);
    SEQBITS_CAS_FIELD(successBit, ptr->seqBits, snapshot->seqBits,
                      KCAS_STATE_UNDECIDED, newstate, KCAS_SEQBITS_MASK_STATE,
                      KCAS_SEQBITS_OFFSET_STATE);
  }

  // phase 2 (all addresses are now "locked" for this kcas)
  state = DESC_READ_FIELD(successBit, ptr->seqBits, tagptr,
                          KCAS_SEQBITS_MASK_STATE, KCAS_SEQBITS_OFFSET_STATE);
  if (!successBit)
    return false;

  bool succeeded = (state == KCAS_STATE_SUCCEEDED);
  for (std::size_t i = 0; i < snapshot->numEntries; i++) {
    casword_t newval =
        succeeded ? snapshot->entries[i].newval : snapshot->entries[i].oldval;
    BOOL_CAS(snapshot->entries[i].addr, (casword_t)tagptr, newval);
  }
  return succeeded;
}

// TODO: replace crappy bubblesort with something fast for large K
// (maybe even use insertion sort for small K)

template <class Allocator, class MemReclaimer, std::size_t N>
int BrownOriginal<Allocator, MemReclaimer, N>::cas(
    const int tid, ReclaimerPin<MemReclaimer> &pin, KCASDescriptor *ptr,
    bool debug) {
  // sort entries in the kcas descriptor to guarantee progress
  kcasdesc_sort(ptr);
  DESC_INITIALIZED(kcasDescriptors, tid);
  kcastagptr_t tagptr = TAGPTR_NEW(tid, ptr->seqBits, KCAS_TAGBIT);

  // perform the kcas and retire the old descriptor
  bool result = help(tid, tagptr, ptr, false);
  return result;
}

template <class Allocator, class MemReclaimer, std::size_t N>
casword_t BrownOriginal<Allocator, MemReclaimer, N>::internalReadPtr(
    const int tid, const casword_t volatile *addr) {
  casword_t r;
  do {
    r = rdcssRead(tid, addr);
    //        for (int i=0;i<100;++i) {
    //            r = rdcssRead(tid, addr);
    //            if (!isKcas(r)) break;
    //        }
    if (isKcas(r)) {
      helpOther(tid, (kcastagptr_t)r);
    }
  } while (isKcas(r));
  return r;
}

template <class Allocator, class MemReclaimer, std::size_t N>
casword_t BrownOriginal<Allocator, MemReclaimer, N>::internalReadVal(
    const int tid, const casword_t volatile *addr) {
  return ((casword_t)internalReadPtr(tid, addr)) >> KCAS_LEFTSHIFT;
}

template <class Allocator, class MemReclaimer, std::size_t N>
void BrownOriginal<Allocator, MemReclaimer, N>::internalWritePtr(
    casword_t volatile *addr, casword_t ptr) {
  *addr = ptr;
  // note: enter/leaveQstate calls are not needed here,
  // because the writePtr/Val functions do not access fields of descriptors!
}

template <class Allocator, class MemReclaimer, std::size_t N>
void BrownOriginal<Allocator, MemReclaimer, N>::internalWriteVal(
    casword_t volatile *addr, casword_t val) {
  internalWritePtr(addr, val << KCAS_LEFTSHIFT);
  // note: enter/leaveQstate calls are not needed here,
  // because the writePtr/Val functions do not access fields of descriptors!
}

template <class Allocator, class MemReclaimer, std::size_t N>
typename BrownOriginal<Allocator, MemReclaimer, N>::KCASDescriptor *
BrownOriginal<Allocator, MemReclaimer, N>::create_descriptor(
    const std::size_t descriptor_size, const int tid) {
  // allocate a new kcas descriptor
  BrownOriginal::KCASDescriptor *ptr =
      DESC_NEW(kcasDescriptors, KCAS_SEQBITS_NEW, tid);
  ptr->numEntries = 0;
  return ptr;
}

template <class Allocator, class MemReclaimer, std::size_t N>
std::string
BrownOriginal<Allocator, MemReclaimer, N>::tagptrToString(uintptr_t tagptr) {
  std::stringstream ss;
  if (tagptr) {
    BrownOriginal::KCASDescriptor *ptr;
    ss << "<seq=" << UNPACK_SEQ(tagptr) << ",tid=" << TAGPTR_UNPACK_TID(tagptr)
       << ">";
    ptr = TAGPTR_UNPACK_PTR(kcasDescriptors, tagptr);

    // print contents of actual scx record
    intptr_t seqBits = ptr->seqBits;
    ss << "[";
    ss << "state=" << SEQBITS_UNPACK_FIELD(seqBits, KCAS_SEQBITS_MASK_STATE,
                                           KCAS_SEQBITS_OFFSET_STATE);
    ss << ",";
    ss << "seq=" << UNPACK_SEQ(seqBits);
    ss << "]";
  } else {
    ss << "null";
  }
  return ss.str();
}
}

#endif /* KCAS_REUSE_H */
