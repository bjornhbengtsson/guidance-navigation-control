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

add_library(
    gnc
    STATIC
        src/math/vector3.c
        src/math/quaternion.c
        src/attitude/mahony.c
        src/attitude/mekf.c
)

target_include_directories(
    gnc
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

if(MSVC)
    target_compile_options(gnc PRIVATE /W4 /WX)
else()
    target_compile_options(
        gnc
        PRIVATE
            -Wall
            -Wextra
            -Wpedantic
            -Werror
    )
endif()

add_executable(
    test_math
    tests/unit/test_math.c
)

target_link_libraries(
    test_math
    PRIVATE
        gnc
        m
)

add_test(
    NAME portable_math
    COMMAND test_math
)

add_executable(
    test_mahony
    tests/unit/test_mahony.c
)

target_link_libraries(
    test_mahony
    PRIVATE
        gnc
        m
)

add_test(
    NAME portable_mahony
    COMMAND test_mahony
)

add_executable(
    test_mekf
    tests/unit/test_mekf.c
)

target_link_libraries(
    test_mekf
    PRIVATE
        gnc
        m
)

add_test(
    NAME portable_mekf_initialization
    COMMAND test_mekf
)
