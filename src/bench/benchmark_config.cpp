#include "benchmark_config.h"

/*
Configuration class for benchmark.
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

#include <iostream>

namespace concurrent_data_structures {

void BenchmarkConfig::print(std::ostream &os) const {
  os << "Number of threads: " << num_threads << "\n"
     << "Benchmark duration: " << duration.count() << "\n"
     << "Results directory: " << results_directory.string() << "\n"
     << "Memory reclaimer: " << get_reclaimer_name(reclaimer) << "\n"
     << "Memory allocator: " << get_allocator_name(allocator) << "\n"
     << "Random generator: " << get_generator_name(generator) << "\n"
     << "PAPI Enabled: " << (papi_active ? "true" : "false") << "\n"
     << "Hypthreading before socket switch: "
     << (hyperthreading ? "true" : "false") << std::endl;
}

void SetBenchmarkConfig::print(std::ostream &os) const {
  base.print(os);
  os << "Load factor: " << load_factor << "\n"
     << "Table size: " << table_size << "\n"
     << "Update percentage: " << updates << "\n"
     << "Table name: " << get_table_name(table) << "\n";
}
}
