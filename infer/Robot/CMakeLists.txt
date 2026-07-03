cmake_minimum_required (VERSION 3.1)

# set project
project (AutoRUSS)

# find opencv
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV version: ${OpenCV_VERSION}")

# find torch
# set(Torch_DIR /home/cair-jacen/Libtorch/libtorch/share/cmake/Torch)
# find_package(Torch REQUIRED)

# find hiredis
find_library(HIREDIS_LIB hiredis)
option(BUILD_LEGACY_MAIN_NOMOVE "Build legacy 6-axis main_nomove controller" OFF)

# find CUDA
# find_package(CUDA REQUIRED)
# message(STATUS "Cuda cublas: ${CUDA_CUBLAS_LIBRARYS}")

# disable eigen tensor
add_definitions(-DOPENCV_DISABLE_EIGEN_TENSOR_SUPPORT)

# add libraries
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    /usr/include/eigen3
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty
)

# set(CMAKE_CXX_FLAGS "${CAMKE_CXX_FLAGS} -std=c++14 -pthread")
if(BUILD_LEGACY_MAIN_NOMOVE)
    set(ROBOT_COMMON_SRCS
        ${CMAKE_CURRENT_SOURCE_DIR}/src/contact_sensing.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/force_sensor.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/realman_command.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/realman_kinematics.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/robot_control.cpp
    )

    add_executable(main_nomove
        ${CMAKE_CURRENT_SOURCE_DIR}/src/main_nomove.cpp
        ${ROBOT_COMMON_SRCS}
    )
endif()

add_executable(arm_read_state
    ${CMAKE_CURRENT_SOURCE_DIR}/src/arm_read_state.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/realman_command.cpp
)

add_executable(arm_read_state_raw
    ${CMAKE_CURRENT_SOURCE_DIR}/src/arm_read_state_raw.cpp
)

add_executable(arm_movej_delta
    ${CMAKE_CURRENT_SOURCE_DIR}/src/arm_movej_delta.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/realman_command.cpp
)

# set c++11 and threading
set_property(TARGET arm_read_state PROPERTY CXX_STANDARD 17)
set_property(TARGET arm_read_state_raw PROPERTY CXX_STANDARD 17)
set_property(TARGET arm_movej_delta PROPERTY CXX_STANDARD 17)
if(BUILD_LEGACY_MAIN_NOMOVE)
    set_property(TARGET main_nomove PROPERTY CXX_STANDARD 17)
endif()
set(CMAKE_BUILD_TYPE "Release" CMAKE_CXX_FLAGS_RELEASE:STRING=-O3 -DNDEBUG)

if(BUILD_LEGACY_MAIN_NOMOVE)
    target_link_libraries(main_nomove
        ${HIREDIS_LIB}
        ${OpenCV_LIBS}
        # ${DEPENDENCIES}
        # ${TORCH_LIBRARIES}
        # /usr/lib
        # /usr/local/lib
    )
endif()
