#pragma once

/*
Some utilities to cache align classes and structs.
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

#include <cstdint>
#include <cstdlib>

namespace concurrent_data_structures {

static const std::size_t S_CACHE_PADDING = 128;
static const std::size_t S_CACHE_SIZE = 64;

namespace {

template <class Object> class DummyCachePadded {
private:
  std::uint8_t padding[S_CACHE_PADDING - (sizeof(Object) % S_CACHE_PADDING)];

public:
  template <typename... _Args>
  DummyCachePadded(_Args &&... __args) : Object(__args...) {}
};
}

template <class Object>
class alignas(alignof(DummyCachePadded<Object>) > S_CACHE_PADDING
                  ? alignof(DummyCachePadded<Object>)
                  : S_CACHE_PADDING) CachePadded : public Object {
private:
  std::uint8_t m_padding[S_CACHE_PADDING - (sizeof(Object) % S_CACHE_PADDING)];

public:
  template <typename... _Args>
  CachePadded(_Args &&... __args) : Object(__args...) {}
};
}
