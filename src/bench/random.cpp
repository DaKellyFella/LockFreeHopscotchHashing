#include "random.h"

/*
Mapping enums to string descriptions of random generators.
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
static const std::map<RandomGenerator, std::string> random_map{
    std::make_pair(RandomGenerator::LCGBSD, "BSD LCG"),
    std::make_pair(RandomGenerator::PCG, "PCG"),
    std::make_pair(RandomGenerator::XOR_SHIFT_32, "XOR 32"),
    std::make_pair(RandomGenerator::XOR_SHIFT_64, "XOR 64"),
};
}

const std::string get_generator_name(const RandomGenerator random_generator) {
  std::string random_name = "ERROR: Incorrect random generator name.";
  auto it = random_map.find(random_generator);
  if (it != random_map.end()) {
    random_name = it->second;
  }
  return random_name;
}
}
