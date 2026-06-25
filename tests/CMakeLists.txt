add_executable(aero_control_sim_test_sanity
    test_sanity.cpp
)

add_test(NAME aero_control_sim_test_sanity COMMAND aero_control_sim_test_sanity)

add_executable(aero_control_sim_test_pid
    test_pid.cpp
)

target_include_directories(aero_control_sim_test_pid PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

add_test(NAME aero_control_sim_test_pid COMMAND aero_control_sim_test_pid)

add_executable(aero_control_sim_test_analyzer
    test_analyzer.cpp
)

target_include_directories(aero_control_sim_test_analyzer PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

target_link_libraries(aero_control_sim_test_analyzer PRIVATE acs)

add_test(NAME aero_control_sim_test_analyzer COMMAND aero_control_sim_test_analyzer)

add_executable(aero_control_sim_test_math_constants
    test_math_constants.cpp
)

target_include_directories(aero_control_sim_test_math_constants PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

add_test(NAME aero_control_sim_test_math_constants COMMAND aero_control_sim_test_math_constants)

add_executable(aero_control_sim_test_json_config
    test_json_config.cpp
)

target_include_directories(aero_control_sim_test_json_config PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

add_test(NAME aero_control_sim_test_json_config COMMAND aero_control_sim_test_json_config)

add_executable(aero_control_sim_test_config_loader
    test_config_loader.cpp
)

target_include_directories(aero_control_sim_test_config_loader PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

target_link_libraries(aero_control_sim_test_config_loader PRIVATE acs)

add_test(NAME aero_control_sim_test_config_loader COMMAND aero_control_sim_test_config_loader)

add_executable(aero_control_sim_test_noise
    test_noise.cpp
)

target_include_directories(aero_control_sim_test_noise PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

target_link_libraries(aero_control_sim_test_noise PRIVATE acs)

add_test(NAME aero_control_sim_test_noise COMMAND aero_control_sim_test_noise)

add_executable(aero_control_sim_test_telemetry
    test_telemetry.cpp
)

target_include_directories(aero_control_sim_test_telemetry PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

target_link_libraries(aero_control_sim_test_telemetry PRIVATE acs)

add_test(NAME aero_control_sim_test_telemetry COMMAND aero_control_sim_test_telemetry)

add_executable(aero_control_sim_test_statistics_utils
    test_statistics_utils.cpp
)

target_include_directories(aero_control_sim_test_statistics_utils PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

target_link_libraries(aero_control_sim_test_statistics_utils PRIVATE acs)

add_test(NAME aero_control_sim_test_statistics_utils COMMAND aero_control_sim_test_statistics_utils)

add_executable(aero_control_sim_test_actuators
    test_actuators.cpp
)

target_include_directories(aero_control_sim_test_actuators PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

target_link_libraries(aero_control_sim_test_actuators PRIVATE acs)

add_test(NAME aero_control_sim_test_actuators COMMAND aero_control_sim_test_actuators)
