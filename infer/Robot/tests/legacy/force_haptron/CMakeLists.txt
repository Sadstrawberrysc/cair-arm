cmake_minimum_required(VERSION 3.10)

project(haptron_force LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)

add_executable(main
    ${PROJECT_SOURCE_DIR}/src/main.cpp
    ${PROJECT_SOURCE_DIR}/src/force_haptron.cpp
)

target_include_directories(main PRIVATE
    ${PROJECT_SOURCE_DIR}/include
    /usr/local/include
)

target_link_libraries(main PRIVATE Threads::Threads)
