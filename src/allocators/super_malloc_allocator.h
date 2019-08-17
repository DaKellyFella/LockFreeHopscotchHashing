#pragma once

/*
Simple wrapper around the mangled SuperMalloc allocator.
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

#include <cstdlib>

extern "C" void *__super_malloc(size_t);
extern "C" void *__super_aligned_alloc(size_t, size_t);
extern "C" void __super_free(void *);
extern "C" size_t __super_malloc_usable_size(void *);

struct SuperMallocAllocator {
  static void *malloc(size_t size) { return __super_malloc(size); }
  static void *aligned_alloc(size_t alignment, size_t size) {
    return __super_aligned_alloc(alignment, size);
  }
  static void free(void *ptr) { return __super_free(ptr); }
  static size_t malloc_usable_size(void *ptr) {
    return __super_malloc_usable_size(ptr);
  }
};
