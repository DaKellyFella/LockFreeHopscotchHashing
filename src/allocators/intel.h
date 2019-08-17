#pragma once

/*
Simple wrapper around Intel scalable allocator.
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

#include <tbb/scalable_allocator.h>

struct IntelAllocator {
  static void *malloc(size_t size) { return scalable_malloc(size); }
  static void *aligned_alloc(size_t alignment, size_t size) {
    return scalable_aligned_malloc(alignment, size);
  }
  static void free(void *ptr) { scalable_free(ptr); }
  static size_t malloc_usable_size(void *ptr) { return scalable_msize(ptr); }
};
