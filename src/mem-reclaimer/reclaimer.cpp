#include "reclaimer.h"

/*
Maps reclaimers to human readable strings.
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

#include <map>

namespace concurrent_data_structures {

namespace {
static const std::map<Reclaimer, std::string> reclaimer_map{
    std::make_pair(Reclaimer::Leaky, "Leaky"),
};
}

const std::string get_reclaimer_name(const Reclaimer reclaimer) {
  std::string reclaimer_name = "ERROR: Incorrect reclaimer name.";
  auto it = reclaimer_map.find(reclaimer);
  if (it != reclaimer_map.end()) {
    reclaimer_name = it->second;
  }
  return reclaimer_name;
}
}
