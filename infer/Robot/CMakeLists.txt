cmake_minimum_required(VERSION 3.16)

project(AutoRUSS_RM75 LANGUAGES CXX)

include(CTest)

find_package(Threads REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_library(HIREDIS_LIB hiredis REQUIRED)

option(BUILD_LEGACY_MAIN_NOMOVE
       "Deprecated: legacy six-axis sources are retained as references only"
       OFF)
option(BUILD_MAINTENANCE_TOOLS
       "Build retained robot diagnostics and sensor calibration utilities"
       OFF)
if(BUILD_LEGACY_MAIN_NOMOVE)
    message(WARNING
        "tests/legacy/six_axis is retained as an original six-axis reference; "
        "it is not built against the RM75 seven-axis libraries.")
endif()

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(robot_project_options INTERFACE)
target_include_directories(robot_project_options INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(robot_project_options INTERFACE Eigen3::Eigen)
target_compile_definitions(robot_project_options INTERFACE
    OPENCV_DISABLE_EIGEN_TENSOR_SUPPORT
)
target_compile_options(robot_project_options INTERFACE
    $<$<COMPILE_LANG_AND_ID:CXX,GNU,Clang>:-Wall;-Wextra;-Wpedantic>
)

# RM75 communication and seven-axis motion.
add_library(realman_transport STATIC
    src/realman_command.cpp
)
target_link_libraries(realman_transport PUBLIC
    robot_project_options
    Threads::Threads
)

add_library(rm75_motion STATIC
    src/realman_kinematics.cpp
    src/rm75_control.cpp
)
target_link_libraries(rm75_motion PUBLIC robot_project_options)

add_library(rm75_runtime_config STATIC
    src/rm75_runtime_config.cpp
)
target_link_libraries(rm75_runtime_config PUBLIC
    robot_project_options
    rm75_motion
)

# Haptron sensor, calibration compensation and contact estimation.
add_library(robot_sensor STATIC
    src/force_sensor.cpp
    src/contact_sensing.cpp
)
target_link_libraries(robot_sensor PUBLIC
    robot_project_options
    Threads::Threads
    OpenSSL::Crypto
)

# Immutable Base -> Arm_Tip -> Tool/Sensor -> Probe TCP transform chain.
add_library(rm75_frame_chain STATIC
    src/calibrated_frame_chain.cpp
)
target_link_libraries(rm75_frame_chain PUBLIC
    robot_project_options
    realman_transport
    robot_sensor
)

# Non-blocking CSV recording and versioned end-of-run summary construction.
add_library(rm75_runtime_logging STATIC
    src/rm75_runtime_logging.cpp
)
target_link_libraries(rm75_runtime_logging PUBLIC
    robot_project_options
    realman_transport
    rm75_motion
    robot_sensor
    Threads::Threads
)

if(BUILD_MAINTENANCE_TOOLS)
    # Diagnostics and calibration utilities are separated from production src.
    add_executable(arm_read_state tests/tools/arm_read_state.cpp)
    target_link_libraries(arm_read_state PRIVATE realman_transport)

    add_executable(arm_preset_pose tests/tools/arm_preset_pose.cpp)
    target_link_libraries(arm_preset_pose PRIVATE realman_transport)

    add_executable(rm75_servoj_diagnostic
        tests/tools/arm_servoj_line_test.cpp
    )
    target_link_libraries(rm75_servoj_diagnostic PRIVATE
        realman_transport
        rm75_motion
    )

    # Sensor calibration utilities retained for future physical re-calibration.
    add_executable(force_sensor_calibrate
        tests/tools/force_sensor_calibrate.cpp
    )
    target_link_libraries(force_sensor_calibrate PRIVATE robot_sensor)

    add_executable(force_sensor_calibration_capture
        tests/tools/force_sensor_calibration_capture.cpp
    )
    target_include_directories(force_sensor_calibration_capture PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/tools
    )
    target_link_libraries(force_sensor_calibration_capture PRIVATE
        realman_transport
        robot_sensor
    )
endif()

# Single migrated RM75 production controller. Keep the entry/orchestration
# separate from Redis I/O while producing only one runtime executable.
add_executable(main_rm75
    src/main_rm75.cpp
    src/redis_bridge.cpp
)
target_link_libraries(main_rm75 PRIVATE
    rm75_frame_chain
    rm75_runtime_config
    rm75_runtime_logging
    realman_transport
    rm75_motion
    robot_sensor
    Threads::Threads
    ${HIREDIS_LIB}
)

if(BUILD_TESTING)
    add_executable(redis_bridge_test
        tests/offline/redis_bridge_test.cpp
        src/redis_bridge.cpp
    )
    target_link_libraries(redis_bridge_test PRIVATE
        robot_project_options
        rm75_motion
        Threads::Threads
        ${HIREDIS_LIB}
    )
    add_test(NAME redis_bridge_test COMMAND redis_bridge_test)

    add_executable(sensor_parser_test
        tests/offline/sensor_parser_test.cpp
    )
    target_link_libraries(sensor_parser_test PRIVATE robot_sensor)
    add_test(NAME sensor_parser_test COMMAND sensor_parser_test)

    add_executable(motion_logic_test
        tests/offline/motion_logic_test.cpp
    )
    target_link_libraries(motion_logic_test PRIVATE rm75_motion)
    add_test(NAME motion_logic_test COMMAND motion_logic_test)

    add_executable(runtime_schema_test
        tests/offline/runtime_schema_test.cpp
    )
    target_link_libraries(runtime_schema_test PRIVATE rm75_runtime_logging)
    add_test(NAME runtime_schema_test COMMAND runtime_schema_test)

    add_executable(runtime_config_test
        tests/offline/runtime_config_test.cpp
    )
    target_link_libraries(runtime_config_test PRIVATE rm75_runtime_config)
    add_test(NAME runtime_config_test COMMAND runtime_config_test)

    add_executable(frame_chain_test
        tests/offline/frame_chain_test.cpp
    )
    target_link_libraries(frame_chain_test PRIVATE rm75_frame_chain)
    add_test(NAME frame_chain_test COMMAND frame_chain_test)

    add_custom_target(robot_offline_tests
        DEPENDS
            redis_bridge_test
            sensor_parser_test
            motion_logic_test
            runtime_schema_test
            runtime_config_test
            frame_chain_test
    )
endif()
