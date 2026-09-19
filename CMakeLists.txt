cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(pico_w_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(PICO_BOARD pico_w)

pico_sdk_init()

add_executable(pico_w_project
    src/main.c
)

target_link_libraries(pico_w_project
    pico_stdlib
    pico_cyw43_arch_none
)

pico_enable_stdio_uart(pico_w_project 0)
pico_enable_stdio_usb(pico_w_project 1)

pico_add_extra_outputs(pico_w_project)
