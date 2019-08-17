#include "allocators.h"

/*
Maps allocators to human strings.
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

#include <map>
namespace concurrent_data_structures {

namespace {
static const std::map<Allocator, std::string> allocator_map{
    std::make_pair(Allocator::JeMalloc, "Je-Malloc"),
    std::make_pair(Allocator::Glibc, "Glibc"),
    std::make_pair(Allocator::SuperMalloc, "Super-Malloc"),
    std::make_pair(Allocator::Intel, "Intel")};
}

const std::string get_allocator_name(const Allocator table) {
  std::string allocator_name = "ERROR: Incorrect allocator name.";
  auto it = allocator_map.find(table);
  if (it != allocator_map.end()) {
    allocator_name = it->second;
  }
  return allocator_name;
}
}
