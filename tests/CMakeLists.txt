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
