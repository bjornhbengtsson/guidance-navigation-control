cmake_minimum_required(VERSION 3.20)

project(
    guidance_navigation_control
    VERSION 0.1.0
    LANGUAGES C
)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

enable_testing()

add_library(gnc INTERFACE)

target_include_directories(
    gnc
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
