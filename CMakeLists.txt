cmake_minimum_required(VERSION 3.20)
project(riscv_jit_emulator C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Both for C and C++
add_compile_options(
    -Wall -Wextra
    -Wno-unused-parameter
)

# Only for C
add_compile_options(
    $<$<COMPILE_LANGUAGE:C>:-Wswitch-enum>
    $<$<COMPILE_LANGUAGE:C>:-Werror=switch-enum>
)

# Only for Cpp
# Empty now...

# Sanitizer (Debug only, both for C & C++)
add_compile_options(
    $<$<CONFIG:Debug>:-fsanitize=address,undefined>
    $<$<CONFIG:Debug>:-fno-omit-frame-pointer>
    $<$<CONFIG:Debug>:-O0>
    $<$<CONFIG:Debug>:-g3>
)
add_link_options(
    $<$<CONFIG:Debug>:-fsanitize=address,undefined>
)

# Release
add_compile_options(
    $<$<CONFIG:Release>:-O2>
    $<$<CONFIG:Release>:-g>
)

include_directories(src)

add_executable(riscv_jit_emulator
        src/main.c)

# Test
enable_testing()