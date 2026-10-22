cmake_minimum_required(VERSION 3.16)
project(SwarmGrid VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Single-config generators (Ninja/Makefiles) default to Release.
# Multi-config generators (Visual Studio) pick the config at build time: --config Release
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(SWARMGRID_BUILD_TESTS "Build unit tests" ON)
option(SWARMGRID_WERROR "Treat warnings as errors" OFF)
option(SWARMGRID_SANITIZE "Enable ASan + UBSan (GCC/Clang only)" OFF)

# ---- warnings: one interface target, linked PRIVATE so doctest isn't affected ----
add_library(swarmgrid_warnings INTERFACE)
if(MSVC)
  target_compile_options(swarmgrid_warnings INTERFACE /W4 /permissive-)
  if(SWARMGRID_WERROR)
    target_compile_options(swarmgrid_warnings INTERFACE /WX)
  endif()
else()
  target_compile_options(swarmgrid_warnings INTERFACE -Wall -Wextra -Wpedantic -Wshadow)
  if(SWARMGRID_WERROR)
    target_compile_options(swarmgrid_warnings INTERFACE -Werror)
  endif()
endif()

if(SWARMGRID_SANITIZE AND NOT MSVC)
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address,undefined)
endif()

# ---- core library: every milestone adds its .cpp here ----
add_library(swarmgrid_core
  src/grid.cpp
  src/map_io.cpp
  src/warehouse_gen.cpp
  src/ascii.cpp
  src/heuristic.cpp
  src/astar.cpp
)
target_include_directories(swarmgrid_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(swarmgrid_core PRIVATE swarmgrid_warnings)

# ---- CLI ----
add_executable(swarmgrid src/main.cpp)
target_link_libraries(swarmgrid PRIVATE swarmgrid_core swarmgrid_warnings)

# ---- tests ----
if(SWARMGRID_BUILD_TESTS)
  enable_testing()
  add_executable(swarmgrid_tests
    tests/test_main.cpp
    tests/test_grid.cpp
    tests/test_map_io.cpp
    tests/test_warehouse_gen.cpp
    tests/test_ascii.cpp
    tests/test_heuristic.cpp
    tests/test_astar.cpp
  )
  target_link_libraries(swarmgrid_tests PRIVATE swarmgrid_core swarmgrid_warnings)
  target_include_directories(swarmgrid_tests SYSTEM PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/third_party)
  # Absolute path so tests work no matter which directory ctest runs them from.
  target_compile_definitions(swarmgrid_tests PRIVATE
    SWARMGRID_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")
  add_test(NAME swarmgrid_tests COMMAND swarmgrid_tests)
endif()
