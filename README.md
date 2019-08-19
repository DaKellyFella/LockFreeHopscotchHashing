# Lock-Free Hopscotch Hashing

## Tables present
1. Lock-Free Hopscotch Hashing
2. Locked Hopscotch (Bit-map variant)
3. Locked Hopscotch (Relative offset variant)
4. Purcell-Harris lock-free quadratic programming.

## Build instruction
These benchmarks require a number of dependencies.
* [A CPU topology library for smart thread pinning](https://github.com/Maratyszcza/cpuinfo)
* [JeMalloc](https://github.com/jemalloc/jemalloc)
* CMake (Installed via system package manager)
* PAPI (Installed via system package manager)
* Boost (Installed via system package manager)

Our code was tested and ran on Ubuntu 14.04, 16.04, and 18.04. The code itself uses the CMake build system. When testing make sure to compile the code in release! Or at least *our code*...

## Run instructions
Once built the binary takes a number of arguments at command-line parameters and through standard input.

Brief explanation of the command.
* -T ==> Number of threads
* -L ==> Load factor 1.0 is full, 0.0 is empty, 0.4 is 40% full, etc.
* -S ==> Table size as a power of 2.
* -D ==> Number of seconds to run the benchmark procedure for.
* -U ==> Percentage updates
* -B ==> Table to benchmark
* -M ==> Memory reclaimer
* -A ==> What allocator to use.
* -P ==> Whether PAPI is turned on.
* -R ==> Choice of random number generator 

Here are some example commands. All parameters have default values if none are provided.
 
* ./concurrent_hash_tables -T 1 -L 0.6 -S 23 -D 10 -U 10 -P true -M leaky -A je -R xor_shift_64 -B hsbm_lf_set
* ./concurrent_hash_tables -T 4 -L 0.6 -S 23 -D 10 -U 30 -P true -M leaky -A je -R xor_shift_64 -B hsbm_locked_set
* ./concurrent_hash_tables -T 8 -L 0.8 -S 23 -D 10 -U 40 -P true -M leaky -A je -R xor_shift_64 -B ph_qp_set

The results are put into two csv files, one containing the keys and the other containing the specific info.