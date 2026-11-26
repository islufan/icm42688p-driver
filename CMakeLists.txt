cmake_minimum_required(VERSION 3.16)
project(icm42688 C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# 源码为 UTF-8，MSVC 默认按本地代码页解析会报 C4819
if(MSVC)
    add_compile_options(/utf-8)
endif()

add_library(icm42688 STATIC
    src/icm42688.c
)
target_include_directories(icm42688 PUBLIC src)

# 只在使用 STM32 HAL 时才需要（需要 main.h / HAL 头文件路径）
option(ICM_BUILD_HAL_PORT "Build STM32 HAL adapter" OFF)
if(ICM_BUILD_HAL_PORT)
    add_library(icm42688_hal STATIC port/icm42688_hal.c)
    target_link_libraries(icm42688_hal PUBLIC icm42688)
    target_include_directories(icm42688_hal PUBLIC port)
endif()

option(ICM_BUILD_TESTS "Build unit tests" OFF)
if(ICM_BUILD_TESTS)
    enable_testing()
    add_executable(test_icm42688 tests/test_icm42688.c)
    target_link_libraries(test_icm42688 PRIVATE icm42688)
    add_test(NAME icm42688_tests COMMAND test_icm42688)
endif()
