#include "table.h"

/*
Mapping enums to string descriptions of hash tables.
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
static const std::map<HashTable, std::string> table_map{
    std::make_pair(HashTable::HS_TRANS_SET, "Hopscotch Transactional Set"),
    std::make_pair(HashTable::HS_LOCKED_SET, "Hopscotch Locked Set"),
    std::make_pair(HashTable::HSC_LOCKED_SET, "Hopscotch Compact Locked Set"),
    std::make_pair(HashTable::HSBM_LOCKED_SET, "Hopscotch Bitmap Locked Set"),
    std::make_pair(HashTable::HSBM_SERIAL_SET, "Hopscotch Bitmap Serial Set"),
    std::make_pair(HashTable::HSBM_LF_SET, "Hopscotch Bitmap Lock-Free Set"),
    std::make_pair(HashTable::PH_QP_SET, "Purcell Harris QP Set"),
};
}

const std::string get_table_name(const HashTable table) {
  std::string table_name = "ERROR: Incorrect hash-table name.";
  auto it = table_map.find(table);
  if (it != table_map.end()) {
    table_name = it->second;
  }
  return table_name;
}
}
