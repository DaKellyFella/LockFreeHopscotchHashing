#pragma once

/*
Simple wrapper around standard malloc.
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
#include <malloc.h>

struct GlibcAllocator {
  static void *malloc(size_t size) { return std::malloc(size); }
  static void *aligned_alloc(size_t alignment, size_t size) {
    return ::aligned_alloc(alignment, size);
  }
  static void free(void *ptr) { std::free(ptr); }
  static size_t malloc_usable_size(void *ptr) {
    return ::malloc_usable_size(ptr);
  }
};
