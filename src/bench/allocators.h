#pragma once

/*
Allocators used with a C++ template.
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
#include <cstdlib>
#include <limits>
#include <memory>
#include <string>

namespace concurrent_data_structures {

enum class Allocator {
  JeMalloc,
  Glibc,
  SuperMalloc,
  Intel,
};

template <class GivenAllocator, typename T> class AllocatorInterface {
  template <typename Z> struct allocator_type { typedef Z value_type; };

public:
  typedef typename allocator_type<T>::value_type value_type;
  typedef value_type *pointer;
  typedef const value_type *const_pointer;
  typedef value_type &reference;
  typedef const value_type &const_reference;
  typedef std::size_t size_type;
  typedef std::ptrdiff_t difference_type;
  template <class U> struct rebind {
    typedef AllocatorInterface<GivenAllocator, U> other;
  };

  AllocatorInterface() throw() {}
  AllocatorInterface(const AllocatorInterface &) throw() {}
  template <typename U>
  AllocatorInterface(const AllocatorInterface<GivenAllocator, U> &) throw() {}

  pointer address(reference x) const { return &x; }
  const_pointer address(const_reference x) const { return &x; }

  pointer allocate(size_type n, const void * /*hint*/ = 0) {
    return static_cast<pointer>(GivenAllocator::malloc(n * sizeof(value_type)));
  }

  void deallocate(pointer p, size_type) { GivenAllocator::free(p); }

  size_type max_size() const throw() {
    size_type absolutemax = static_cast<size_type>(-1) / sizeof(value_type);
    return (absolutemax > 0 ? absolutemax : 1);
  }
  template <typename U, typename... Args>
  void construct(U *p, Args &&... args) {
    ::new ((void *)p) U(std::forward<Args>(args)...);
  }
  void destroy(pointer p) { p->~value_type(); }
};

const std::string get_allocator_name(const Allocator allocator);
}
