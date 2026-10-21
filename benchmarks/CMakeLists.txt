######################################################
# BENCHMARKS
#
# Enabled with -DBTCPP_BUILD_BENCHMARKS=ON. See benchmarks/README.md.

include(FetchContent)

find_package(benchmark 1.6 QUIET)
if(NOT benchmark_FOUND)
  message(STATUS "Google Benchmark not found: fetching v1.9.1")
  set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "" FORCE)
  set(BENCHMARK_ENABLE_GTEST_TESTS OFF CACHE BOOL "" FORCE)
  set(BENCHMARK_ENABLE_INSTALL OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(benchmark
    GIT_REPOSITORY https://github.com/google/benchmark.git
    GIT_TAG        v1.9.1)
  FetchContent_MakeAvailable(benchmark)
endif()

if(NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "Debug")
  message(WARNING "Benchmarks should be built with -DCMAKE_BUILD_TYPE=Release")
endif()

# Time benchmarks (Google Benchmark)
add_executable(bt_bench
  bench_tick.cpp
  bench_ports.cpp
  bench_blackboard.cpp
  bench_load.cpp)
target_link_libraries(bt_bench PRIVATE ${BTCPP_LIBRARY} benchmark::benchmark_main)
target_include_directories(bt_bench PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Memory / allocation report (global operator new is instrumented)
add_executable(bt_membench bench_memory.cpp)
target_link_libraries(bt_membench PRIVATE ${BTCPP_LIBRARY})
target_include_directories(bt_membench PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Per-tick latency distribution under cold-cache / periodic / fragmented conditions
add_executable(bt_latency bench_latency.cpp)
target_link_libraries(bt_latency PRIVATE ${BTCPP_LIBRARY})
target_include_directories(bt_latency PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Per-tick latency of an application's XML trees, with stub nodes
add_executable(bt_xml_latency bench_xml_latency.cpp)
target_link_libraries(bt_xml_latency PRIVATE ${BTCPP_LIBRARY} tinyxml2::tinyxml2)
target_include_directories(bt_xml_latency PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
if(BTCPP_GROOT_INTERFACE)
  target_compile_definitions(bt_xml_latency PRIVATE BENCH_WITH_GROOT2)
endif()
