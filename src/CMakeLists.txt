project(concurrent_data_structures)
cmake_minimum_required(VERSION 3.1)


include_directories(.)

find_package(Boost COMPONENTS system filesystem REQUIRED)
FILE(GLOB_RECURSE HeaderCFiles "*.h")


aux_source_directory(allocators COMMON_LIST)
aux_source_directory(bench COMMON_LIST)
aux_source_directory(mem-reclaimer COMMON_LIST)
aux_source_directory(primitives COMMON_LIST)
aux_source_directory(sets HASH_TABLE)
aux_source_directory(random RANDOM_GENERATORS)



set(HASH_TABLE_EXE "concurrent_hash_tables")

set(LIBS
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    -latomic
    -lpapi
    -ljemalloc
    -lsupermalloc
    -lrt
    -lcpuinfo
    -ltbb
    -ltbbmalloc
    -lpthread)

add_executable(${HASH_TABLE_EXE} hash-tables/main.cpp ${COMMON_LIST} ${HeaderCFiles})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -mrtm -Wall -fno-omit-frame-pointer -flto")


TARGET_LINK_LIBRARIES(${HASH_TABLE_EXE} ${LIBS})
