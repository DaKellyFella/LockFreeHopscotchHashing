#pragma once

/*
Some common hash table functions.
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

#include <cstdint>
#include <functional>
#include <limits>

namespace concurrent_data_structures {

// TODO: Please just destroy this file.

struct SizetHash {
  SizetHash() {}
  std::size_t operator()(const std::size_t arg) const {
    std::size_t key = arg;
    key = (key ^ (key >> 30)) * UINT64_C(0xbf58476d1ce4e5b9);
    key = (key ^ (key >> 27)) * UINT64_C(0x94d049bb133111eb);
    key = key ^ (key >> 31);
    return key;
  }
};

struct IntelHashCompare {
  static std::size_t hash(const std::size_t arg) { return SizetHash{}(arg); }
  static bool equal(const std::size_t lhs, const std::size_t rhs) {
    return lhs == rhs;
  }
};

template <typename T> T nearest_power_of_two(T num) {
  std::size_t actual_size = sizeof(num);
  if (num == 0)
    return std::size_t(0);
  num--;
  // Single byte switches
  num |= num >> 1;
  num |= num >> 2;
  num |= num >> 4;
  std::size_t current_shift = 8;
  for (std::size_t i = 1; i < actual_size; i *= 2, current_shift *= 2) {
    num |= num >> current_shift;
  }
  return ++num;
}

template <class T> struct KeyTraits {
  typedef T Key;
  typedef typename std::hash<T> Hash;
  static const Key NullKey = std::numeric_limits<Key>::max() >> 4;
  static const Key Tombstone = NullKey - 1;
  static std::size_t hash(T key) { return Hash{}(key); }
  static std::size_t hash2(T key) { return hash(hash(key)); }
};

template <class T> struct ValueTraits {
  typedef T Value;
  static const T NullValue = std::numeric_limits<Value>::max();
};

template <> struct KeyTraits<std::size_t> {
  typedef std::size_t Key;
  typedef typename std::hash<std::uint64_t> Hash;
  // TODO: Change this please
  static const Key NullKey = std::numeric_limits<std::uint32_t>::max() >> 2;
  static const Key Tombstone = NullKey - 1;
  static std::size_t hash(Key key) {
    key = (key ^ (key >> 30)) * UINT64_C(0xbf58476d1ce4e5b9);
    key = (key ^ (key >> 27)) * UINT64_C(0x94d049bb133111eb);
    key = key ^ (key >> 31);
    return key;
  }
  static std::size_t hash2(Key key) { return hash(hash(key)); }
};

template <> struct KeyTraits<std::intptr_t> {
  typedef std::intptr_t Key;
  typedef typename std::hash<std::intptr_t> Hash;
  // TODO: Change this please
  static const Key NullKey = std::numeric_limits<std::uint32_t>::max() >> 2;
  static const Key Tombstone = NullKey - 1;
  static std::size_t hash(Key key) {
    key = (key ^ (key >> 30)) * INT64_C(0xbf58476d1ce4e5b9);
    key = (key ^ (key >> 27)) * INT64_C(0x94d049bb133111eb);
    key = key ^ (key >> 31);
    return key;
  }
  static std::size_t hash2(Key key) { return hash(hash(key)); }
};
}
