#pragma once
/*
Summarises benchmark data into files.
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
#include "benchmark_config.h"
#include "benchmark_results.h"
#include <string>
namespace concurrent_data_structures {
// Produce summary for file.
void produce_summary(const SetBenchmarkConfig &config,
                     const SetBenchmarkResult &result,
                     const std::string &human_filename,
                     const std::string &csv_key_filename,
                     const std::string &csv_data_filename);

} // namespace concurrent_hash_tables
