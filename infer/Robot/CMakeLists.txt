cmake_minimum_required(VERSION 3.16)

project(AutoRUSS_RM75 LANGUAGES CXX)

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
    realman_transport
    rm75_motion
    robot_sensor
    Threads::Threads
    ${HIREDIS_LIB}
)
