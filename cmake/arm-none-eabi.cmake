################################################################################
# Arm GNU Toolchain (arm-none-eabi) for the RP2040
#
# Set PICO_TOOLCHAIN_PATH to the toolchain root (the folder that contains
# "bin"), either as a CMake cache variable or as an environment variable:
#
#   cmake -DPICO_TOOLCHAIN_PATH=/path/to/arm-gnu-toolchain ...
#   export PICO_TOOLCHAIN_PATH=/path/to/arm-gnu-toolchain
#
# If it is not set, arm-none-eabi-gcc is looked up on PATH.
################################################################################

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# The compiler test would otherwise try to link a program for a target with no
# C runtime startup files, which always fails.  Compile a static library
# instead; the real firmware link below supplies the linker script.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

if(NOT DEFINED PICO_TOOLCHAIN_PATH AND DEFINED ENV{PICO_TOOLCHAIN_PATH})
    set(PICO_TOOLCHAIN_PATH "$ENV{PICO_TOOLCHAIN_PATH}" CACHE PATH
        "Root of the Arm GNU toolchain (the folder containing bin/)")
endif()

if(PICO_TOOLCHAIN_PATH)
    set(_pico_toolchain_hints "${PICO_TOOLCHAIN_PATH}/bin")
    # try_compile projects only inherit variables listed here.
    list(APPEND CMAKE_TRY_COMPILE_PLATFORM_VARIABLES PICO_TOOLCHAIN_PATH)
elseif(DEFINED ENV{PICO_TOOLCHAIN_PATH})
    file(TO_CMAKE_PATH "$ENV{PICO_TOOLCHAIN_PATH}" _pico_toolchain_root)
    set(_pico_toolchain_hints "${_pico_toolchain_root}/bin")
else()
    set(_pico_toolchain_hints "")
endif()

# The preset may set CMAKE_C_COMPILER directly.  That value is also visible in
# the nested try_compile projects, which do not inherit ordinary cache
# variables such as PICO_TOOLCHAIN_PATH.
if(NOT CMAKE_C_COMPILER)
    find_program(ARM_NONE_EABI_GCC
        NAMES arm-none-eabi-gcc
        HINTS ${_pico_toolchain_hints}
        REQUIRED)
    set(CMAKE_C_COMPILER "${ARM_NONE_EABI_GCC}")
endif()

if(NOT CMAKE_ASM_COMPILER)
    set(CMAKE_ASM_COMPILER "${CMAKE_C_COMPILER}")
endif()

if(NOT CMAKE_SIZE)
    find_program(ARM_NONE_EABI_SIZE
        NAMES arm-none-eabi-size
        HINTS ${_pico_toolchain_hints})
    if(ARM_NONE_EABI_SIZE)
        set(CMAKE_SIZE "${ARM_NONE_EABI_SIZE}")
    endif()
endif()

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
