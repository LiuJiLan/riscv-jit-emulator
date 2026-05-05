cmake_minimum_required(VERSION 3.20)
project(riscv_jit_emulator C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
# CMAKE_C_EXTENSIONS default(ON) => -std=gnu11。
# JIT fast path 需要 __builtin_expect / __attribute__ / __builtin_unreachable
# 等 GCC 扩展; POSIX 函数(pread / open / fstat ...)在 gnu11 下自动暴露不必每文件
# 显式 _POSIX_C_SOURCE。Linux 专有特性(MADV_NOHUGEPAGE 等)仍需各文件自报 _GNU_SOURCE。

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# CMAKE_CXX_EXTENSIONS  default(ON) => -std=gnu++17, 与 C 侧对称。

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
        src/main.c
        src/loader.c
        src/loader.h
        src/core/cpu.c
        src/core/cpu.h
        src/core/dispatcher.c
        src/core/dispatcher.h
        src/core/interpreter.c
        src/core/interpreter.h
        src/core/decode.c
        src/core/decode.h
        src/core/mmu.c
        src/core/mmu.h
        src/core/tlb.c
        src/core/tlb.h
        src/platform/ram.c
        src/platform/ram.h
        src/config.h
        src/riscv.h
        src/core/trap.c
        src/core/trap.h
        src/core/csr.c
        src/core/csr.h
        src/isa/lsu.c
        src/isa/lsu.h)

# Test
enable_testing()