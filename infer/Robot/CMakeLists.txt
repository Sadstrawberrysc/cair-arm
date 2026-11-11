cmake_minimum_required(VERSION 3.16)

project(AutoRUSS_RM75 LANGUAGES CXX)

# Seed a fresh build with the calibrated KWR57 profile. Runtime recalibration
# owns an existing deployment file; configuring/building must not overwrite it.
if(NOT EXISTS "${CMAKE_CURRENT_BINARY_DIR}/rm75_force_calibration.json")
    configure_file(config/force_calibration.kwr57.json
        "${CMAKE_CURRENT_BINARY_DIR}/rm75_force_calibration.json" COPYONLY)
endif()

include(CTest)

find_package(Threads REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_library(HIREDIS_LIB hiredis REQUIRED)

option(BUILD_MAINTENANCE_TOOLS
       "Build retained robot diagnostics and sensor calibration utilities"
       OFF)
option(BUILD_CALIBRATION_SELF_COLLISION "Enable FCL guards and offline calibration filtering" OFF)

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

# Force/torque sensors, calibration compensation and contact estimation.
add_library(robot_sensor STATIC
    src/force_sensor.cpp
    src/contact_sensing.cpp
)
target_link_libraries(robot_sensor PUBLIC
    robot_project_options
    Threads::Threads
    OpenSSL::Crypto
)

if(BUILD_MAINTENANCE_TOOLS OR BUILD_TESTING)
    add_library(force_calibration_path STATIC
        tests/tools/force_calibration_path.cpp
    )
    target_include_directories(force_calibration_path PUBLIC tests/tools)
    target_link_libraries(force_calibration_path PUBLIC rm75_motion robot_sensor)
endif()

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
    target_link_libraries(arm_preset_pose PRIVATE
        rm75_frame_chain
        rm75_motion
        realman_transport
        robot_sensor
    )
    if(BUILD_TESTING)
        add_test(
            NAME artery_target_inspection_test
            COMMAND arm_preset_pose
                --artery-path-base
                ${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/artery_target_base.txt
                --target-calibration
                ${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/artery_camera_calibration.json
                --tool-calibration
                ${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/tool_chain_verified.json
                --inspect-target-only
        )
    endif()

    add_executable(arm_probe_pose tests/tools/arm_probe_pose.cpp)
    target_link_libraries(arm_probe_pose PRIVATE
        rm75_frame_chain
        rm75_motion
        realman_transport
        robot_sensor
    )

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
        force_calibration_path
    )
    if(BUILD_CALIBRATION_SELF_COLLISION)
        find_package(fcl 0.7 REQUIRED CONFIG)
        set(BUILD_COLLISION_CHECKING ON)
        set(BUILD_COLLISION_MONITOR OFF)
        set(BUILD_COLLISION_PLANNING OFF)
        set(BUILD_REALSENSE_SOURCE OFF)
        set(BUILD_MONITOR_VIEWER OFF)
        add_subdirectory(../Collision collision_calibration)
        target_compile_definitions(force_sensor_calibration_capture PRIVATE CALIBRATION_HAS_SELF_COLLISION=1)
        target_link_libraries(force_sensor_calibration_capture PRIVATE collision_checking)
        add_executable(force_calibration_filter tests/tools/force_calibration_filter.cpp)
        target_include_directories(force_calibration_filter PRIVATE tests/tools)
        target_link_libraries(force_calibration_filter PRIVATE force_calibration_path collision_checking)
        # The local FCL prefix includes transitive libraries (e.g. libccd).
        # DT_RPATH lets standalone commands resolve those without shell setup.
        foreach(calibration_target force_sensor_calibration_capture force_calibration_filter)
            set_property(TARGET ${calibration_target} PROPERTY BUILD_RPATH "$<TARGET_FILE_DIR:fcl>")
            target_link_options(${calibration_target} PRIVATE "LINKER:--disable-new-dtags")
        endforeach()
    endif()
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
            force_calibration_path_test
            transport_test
    )
    add_executable(force_calibration_path_test tests/offline/force_calibration_path_test.cpp)
    target_link_libraries(force_calibration_path_test PRIVATE force_calibration_path)
    add_test(NAME force_calibration_path_test COMMAND force_calibration_path_test)

    add_executable(transport_test tests/offline/transport_test.cpp)
    target_link_libraries(transport_test PRIVATE realman_transport)
    add_test(NAME transport_test COMMAND transport_test)

    if(BUILD_MAINTENANCE_TOOLS)
        find_package(Python3 REQUIRED COMPONENTS Interpreter)
        add_test(NAME probe_reach_test
            COMMAND ${Python3_EXECUTABLE} -B
                ${CMAKE_CURRENT_SOURCE_DIR}/tests/offline/probe_reach_test.py
                $<TARGET_FILE:arm_probe_pose>
                ${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/tool_chain_verified.json)
        add_dependencies(robot_offline_tests arm_probe_pose)
        add_test(NAME force_runtime_calibration_test
            COMMAND ${Python3_EXECUTABLE} -B
                ${CMAKE_CURRENT_SOURCE_DIR}/tests/offline/force_runtime_calibration_test.py
                --controller $<TARGET_FILE:main_rm75>
                --calibration ${CMAKE_CURRENT_SOURCE_DIR}/config/force_calibration.kwr57.json
                --probe ${CMAKE_CURRENT_SOURCE_DIR}/model/Lprobe-IFS.STL)
        add_dependencies(robot_offline_tests main_rm75)
        add_test(NAME force_calibration_robust_test
            COMMAND ${Python3_EXECUTABLE} -B
                ${CMAKE_CURRENT_SOURCE_DIR}/tests/offline/force_calibration_robust_test.py
                --fit $<TARGET_FILE:force_sensor_calibrate>)
        if(BUILD_CALIBRATION_SELF_COLLISION)
            add_test(NAME force_calibration_self_collision_test
                COMMAND ${Python3_EXECUTABLE} -B
                    ${CMAKE_CURRENT_SOURCE_DIR}/tests/offline/force_calibration_self_collision_test.py
                    --filter $<TARGET_FILE:force_calibration_filter>
                    --capture $<TARGET_FILE:force_sensor_calibration_capture>)
            add_dependencies(robot_offline_tests force_calibration_filter)
        endif()
        add_test(NAME force_calibration_cli_test
            COMMAND ${Python3_EXECUTABLE} -B
                ${CMAKE_CURRENT_SOURCE_DIR}/tests/offline/force_calibration_cli_test.py)
        add_test(NAME force_calibration_resume_test
            COMMAND ${Python3_EXECUTABLE} -B
                ${CMAKE_CURRENT_SOURCE_DIR}/tests/offline/force_calibration_resume_test.py
                --capture $<TARGET_FILE:force_sensor_calibration_capture>)
        add_test(NAME force_calibration_limit_test
            COMMAND ${Python3_EXECUTABLE} -B
                ${CMAKE_CURRENT_SOURCE_DIR}/tests/offline/force_calibration_limit_test.py
                --capture $<TARGET_FILE:force_sensor_calibration_capture>)
        add_test(NAME force_calibration_auto_test
            COMMAND ${Python3_EXECUTABLE} -B
                ${CMAKE_CURRENT_SOURCE_DIR}/tests/offline/force_calibration_auto_test.py
                --capture $<TARGET_FILE:force_sensor_calibration_capture>
                --fit $<TARGET_FILE:force_sensor_calibrate>
        )
        set_tests_properties(force_calibration_auto_test PROPERTIES TIMEOUT 120)
        add_dependencies(robot_offline_tests force_sensor_calibration_capture force_sensor_calibrate)
    endif()
endif()
