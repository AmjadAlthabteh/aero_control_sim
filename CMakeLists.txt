<<<<<<< HEAD
cmake_minimum_required(VERSION 3.16)

project(aero-control-system-sim VERSION 0.1.0 LANGUAGES CXX)
=======
cmake_minimum_required(VERSION 3.20)

project(aero_control_sim LANGUAGES CXX)
>>>>>>> ff9e5a64303f6786744c89cf3ac13bfab0040f04

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

<<<<<<< HEAD
option(ACS_BUILD_EXAMPLES "build demo executable" ON)

add_library(acs
  src/math/Vector3.cpp
  src/math/Matrix3.cpp
  src/math/Quaternion.cpp
  src/physics/Atmosphere.cpp
  src/physics/Gravity.cpp
  src/physics/RK4.cpp
  src/physics/RigidBody6Dof.cpp
  src/aero/AeroModel.cpp
  src/gnc/PID.cpp
  src/gnc/FlightController.cpp
  src/simulation/Noise.cpp
  src/simulation/Sensors.cpp
  src/simulation/Telemetry.cpp
  src/simulation/Aircraft.cpp
  src/simulation/Wind.cpp
  src/simulation/Actuators.cpp
  src/simulation/Simulator.cpp
  src/profiling/PerformanceProfiler.cpp
  src/analysis/Statistics.cpp
  src/monitoring/PerformanceMonitor.cpp
)

target_include_directories(acs PUBLIC include)
target_compile_features(acs PUBLIC cxx_std_17)

if(MSVC)
  target_compile_options(acs PRIVATE /W4)
else()
  target_compile_options(acs PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(ACS_BUILD_EXAMPLES)
  add_executable(aero-control-system-sim src/main.cpp)
  target_link_libraries(aero-control-system-sim PRIVATE acs)

  add_executable(acs-benchmark src/benchmark.cpp)
  target_link_libraries(acs-benchmark PRIVATE acs)

  add_executable(acs-analyzer src/analyzer.cpp)
  target_link_libraries(acs-analyzer PRIVATE acs)

  add_executable(acs-tests src/tests.cpp)
  target_link_libraries(acs-tests PRIVATE acs)
=======
option(AERO_CONTROL_SIM_BUILD_TESTS "Build tests" ON)

add_executable(aero_control_sim
    src/main.cpp
)

target_compile_features(aero_control_sim PRIVATE cxx_std_17)
target_include_directories(aero_control_sim PRIVATE include)

if(MSVC)
  target_compile_options(aero_control_sim PRIVATE /W4 /permissive-)
else()
  target_compile_options(aero_control_sim PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(AERO_CONTROL_SIM_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
>>>>>>> ff9e5a64303f6786744c89cf3ac13bfab0040f04
endif()
