cmake_minimum_required(VERSION 3.20)

project(aero_control_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(AERO_CONTROL_SIM_BUILD_TESTS "Build tests" ON)

add_executable(aero_control_sim
    src/main.cpp
)

target_compile_features(aero_control_sim PRIVATE cxx_std_17)

if(MSVC)
  target_compile_options(aero_control_sim PRIVATE /W4 /permissive-)
else()
  target_compile_options(aero_control_sim PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(AERO_CONTROL_SIM_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

