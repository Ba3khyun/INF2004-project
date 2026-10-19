################################################################################
# micro T-Kernel 3.0 for the Raspberry Pi Pico W -- CMake build
#
# This builds exactly the same firmware as build_make/pico_rp2040.mk:
# the same sources, defines, include paths, linker script and .uf2 output.
# The makefile remains the reference build; keep the two in step when adding
# a source file or a new build option.
#
# Quick start (from this folder):
#   cmake --preset pico-w
#   cmake --build --preset pico-w
#
# The options below are the CMake equivalent of the make variables:
#   make SMP=0 TEST=1   ->   cmake -DSMP=0 -DTEST=1 ...
#
#   SMP          0 | 1        dual-core kernel (default 1)
#   CONSOLE      usb_cdc|uart USB CDC console, or UART0 only (default usb_cdc)
#   WIFI         cyw43|none   Pico W radio driver (default cyw43)
#   NET          none|lwip    lwIP on top of the radio (default none)
#   MQTT         0 | 1        MQTT client, needs NET=lwip (default 0)
#   TEST         0 | 1        build the subsystem self-tests (default 0)
#   CRASH_REPORT 0 | 1        mirror the console to UART0 on GP0/GP1
#
# CONSOLE=usb_cdc and WIFI=cyw43 need the Pico C SDK.  Point PICO_SDK_PATH at
# it, either as an environment variable or as a CMake cache variable:
#   export PICO_SDK_PATH=$HOME/pico/pico-sdk
#   cmake --preset pico-w -DPICO_SDK_PATH=/path/to/pico-sdk
# The SDK's tinyusb, cyw43-driver and (for NET=lwip) lwip submodules must be
# initialised:  git submodule update --init lib/tinyusb lib/cyw43-driver
#
# PICO_TOOLCHAIN_PATH is the root of the Arm GNU toolchain (the folder that
# contains bin/).  If it is not set, arm-none-eabi-gcc is taken from PATH.
################################################################################

cmake_minimum_required(VERSION 3.20)

# Use the bundled toolchain file when none was given on the command line.
if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_LIST_DIR}/cmake/arm-none-eabi.cmake"
        CACHE FILEPATH "Arm GNU toolchain for the RP2040" FORCE)
endif()

################################################################################
# Build options
################################################################################

set(SMP "1" CACHE STRING "SMP=1 dual core, SMP=0 single core")
set(CONSOLE "usb_cdc" CACHE STRING "Console: usb_cdc or uart")
set(WIFI "cyw43" CACHE STRING "Pico W radio: cyw43 or none")
set(NET "none" CACHE STRING "Network stack: none or lwip")
set(MQTT "0" CACHE STRING "MQTT client: 0 or 1 (needs NET=lwip)")
set(TEST "0" CACHE STRING "Build the subsystem self-tests: 0 or 1")
set(CRASH_REPORT "0" CACHE STRING "Mirror the console to UART0 on GP0/GP1: 0 or 1")

set_property(CACHE SMP PROPERTY STRINGS 0 1)
set_property(CACHE CONSOLE PROPERTY STRINGS usb_cdc uart)
set_property(CACHE WIFI PROPERTY STRINGS cyw43 none)
set_property(CACHE NET PROPERTY STRINGS none lwip)
set_property(CACHE MQTT PROPERTY STRINGS 0 1)
set_property(CACHE TEST PROPERTY STRINGS 0 1)
set_property(CACHE CRASH_REPORT PROPERTY STRINGS 0 1)

if(NOT "${SMP}" MATCHES "^(0|1)$")
    message(FATAL_ERROR "Unknown SMP '${SMP}'; use 0 or 1")
endif()
if(NOT "${CONSOLE}" MATCHES "^(usb_cdc|uart)$")
    message(FATAL_ERROR "Unknown CONSOLE '${CONSOLE}'; use usb_cdc or uart")
endif()
if(NOT "${WIFI}" MATCHES "^(cyw43|none)$")
    message(FATAL_ERROR "Unknown WIFI '${WIFI}'; use cyw43 or none")
endif()
if(NOT "${NET}" MATCHES "^(none|lwip)$")
    message(FATAL_ERROR "Unknown NET '${NET}'; use none or lwip")
endif()
if(NOT "${MQTT}" MATCHES "^(0|1)$")
    message(FATAL_ERROR "Unknown MQTT '${MQTT}'; use 0 or 1")
endif()
if(NOT "${TEST}" MATCHES "^(0|1)$")
    message(FATAL_ERROR "Unknown TEST '${TEST}'; use 0 or 1")
endif()
if(NOT "${CRASH_REPORT}" MATCHES "^(0|1)$")
    message(FATAL_ERROR "Unknown CRASH_REPORT '${CRASH_REPORT}'; use 0 or 1")
endif()

if("${NET}" STREQUAL "lwip" AND NOT "${WIFI}" STREQUAL "cyw43")
    message(FATAL_ERROR "NET=lwip requires WIFI=cyw43")
endif()
if("${MQTT}" STREQUAL "1" AND NOT "${NET}" STREQUAL "lwip")
    message(FATAL_ERROR "MQTT=1 requires NET=lwip")
endif()

# CONSOLE=uart always uses UART0, so it forces the crash-report mirror on.
if("${CONSOLE}" STREQUAL "uart")
    set(CRASH_REPORT "1")
endif()

################################################################################
# Paths: Pico SDK and Arm toolchain
#
# Paths written for WSL (/mnt/c/...) are translated to C:/... when CMake
# itself is the Windows build, so the same command works from either side.
################################################################################

function(_normalise_host_path out_var path)
    if(CMAKE_HOST_WIN32 AND path MATCHES "^/mnt/([A-Za-z])/(.*)$")
        string(TOUPPER "${CMAKE_MATCH_1}" _drive)
        set(path "${_drive}:/${CMAKE_MATCH_2}")
    endif()
    file(TO_CMAKE_PATH "${path}" path)
    set(${out_var} "${path}" PARENT_SCOPE)
endfunction()

# Newest match of <root>/* that looks like a Pico SDK / toolchain.
function(_find_newest_dir out_var)
    file(GLOB _candidates LIST_DIRECTORIES true ${ARGN})
    set(_found "")
    foreach(_candidate IN LISTS _candidates)
        if(IS_DIRECTORY "${_candidate}"
           AND (EXISTS "${_candidate}/pico_sdk_init.cmake"
                OR EXISTS "${_candidate}/bin/arm-none-eabi-gcc"
                OR EXISTS "${_candidate}/bin/arm-none-eabi-gcc.exe"))
            list(APPEND _found "${_candidate}")
        endif()
    endforeach()
    if(_found)
        list(SORT _found COMPARE NATURAL ORDER DESCENDING)
        list(GET _found 0 _best)
        set(${out_var} "${_best}" PARENT_SCOPE)
    endif()
endfunction()

set(_sdk_required FALSE)
if("${CONSOLE}" STREQUAL "usb_cdc" OR "${WIFI}" STREQUAL "cyw43")
    set(_sdk_required TRUE)
endif()

if(PICO_SDK_PATH)
    _normalise_host_path(PICO_SDK_PATH "${PICO_SDK_PATH}")
elseif(DEFINED ENV{PICO_SDK_PATH})
    _normalise_host_path(PICO_SDK_PATH "$ENV{PICO_SDK_PATH}")
elseif(_sdk_required)
    # Common Raspberry Pi Pico VS Code / installer locations.
    _find_newest_dir(_sdk_candidate
        "$ENV{HOME}/.pico-sdk/sdk/*"
        "$ENV{USERPROFILE}/.pico-sdk/sdk/*")
    if(_sdk_candidate)
        _normalise_host_path(PICO_SDK_PATH "${_sdk_candidate}")
        message(STATUS "PICO_SDK_PATH not set; using ${PICO_SDK_PATH}")
    endif()
endif()
if(PICO_SDK_PATH)
    set(PICO_SDK_PATH "${PICO_SDK_PATH}" CACHE PATH "Pico C SDK" FORCE)
endif()

if(_sdk_required)
    if(NOT PICO_SDK_PATH)
        message(FATAL_ERROR
            "PICO_SDK_PATH is not set.  Point it at the Pico C SDK, e.g.\n"
            "  cmake --preset pico-w -DPICO_SDK_PATH=/path/to/pico-sdk\n"
            "or build without the SDK: -DCONSOLE=uart -DWIFI=none")
    endif()
    if(NOT EXISTS "${PICO_SDK_PATH}/pico_sdk_init.cmake")
        message(FATAL_ERROR
            "PICO_SDK_PATH='${PICO_SDK_PATH}' does not look like a Pico C SDK folder")
    endif()
endif()

# The toolchain file finds arm-none-eabi-gcc.  Nothing else is needed here.

project(mtk3pico C ASM)

# Lets editors and clangd find the exact flags for every file.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

################################################################################
# Output name
#
# The image name records the profile, exactly as the makefile does, so a
# test or single-core build cannot be mistaken for the normal one.
################################################################################

set(_wifi_suffix "")
if("${WIFI}" STREQUAL "cyw43")
    set(_wifi_suffix "_wifi")
endif()
set(_net_suffix "")
if("${NET}" STREQUAL "lwip")
    set(_net_suffix "_lwip")
endif()
set(_mqtt_suffix "")
if("${MQTT}" STREQUAL "1")
    set(_mqtt_suffix "_mqtt")
endif()
set(_crash_suffix "")
if("${CONSOLE}" STREQUAL "usb_cdc" AND "${CRASH_REPORT}" STREQUAL "1")
    set(_crash_suffix "_crash")
endif()
set(_test_suffix "")
if("${TEST}" STREQUAL "1")
    set(_test_suffix "_test")
endif()

set(EXE_NAME "mtk3pico_smp${SMP}_${CONSOLE}${_wifi_suffix}${_net_suffix}${_mqtt_suffix}${_crash_suffix}${_test_suffix}")
message(STATUS "Build profile: ${EXE_NAME}")

################################################################################
# Sources -- kept in the same groups as the subdir.mk files
################################################################################

# Every .c in app_program/ and one level of subfolders (one per subsystem).
file(GLOB APP_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/app_program/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/app_program/*/*.c")

set(KERNEL_SOURCES
    kernel/inittask/inittask.c
    kernel/sysinit/sysinit.c
    kernel/tstdlib/bitop.c
    kernel/tstdlib/string.c
    kernel/usermain/usermain.c
    kernel/tkernel/cpuctl.c
    kernel/tkernel/device.c
    kernel/tkernel/deviceio.c
    kernel/tkernel/eventflag.c
    kernel/tkernel/int.c
    kernel/tkernel/klock.c
    kernel/tkernel/mailbox.c
    kernel/tkernel/memory.c
    kernel/tkernel/mempfix.c
    kernel/tkernel/mempool.c
    kernel/tkernel/messagebuf.c
    kernel/tkernel/misc_calls.c
    kernel/tkernel/mutex.c
    kernel/tkernel/objname.c
    kernel/tkernel/power.c
    kernel/tkernel/ready_queue_smp.c
    kernel/tkernel/rendezvous.c
    kernel/tkernel/semaphore.c
    kernel/tkernel/smp_dispatch.c
    kernel/tkernel/smp_lock.c
    kernel/tkernel/task.c
    kernel/tkernel/task_manage.c
    kernel/tkernel/task_sync.c
    kernel/tkernel/time_calls.c
    kernel/tkernel/timer.c
    kernel/tkernel/tkinit.c
    kernel/tkernel/wait.c
    kernel/sysdepend/pico_rp2040/cpu_clock.c
    kernel/sysdepend/pico_rp2040/devinit.c
    kernel/sysdepend/pico_rp2040/hw_setting.c
    kernel/sysdepend/pico_rp2040/power_save.c
    kernel/sysdepend/cpu/rp2040/boot_2nd.c
    kernel/sysdepend/cpu/rp2040/vector_tbl.c
    kernel/sysdepend/cpu/rp2040/smp_rp2040.c
    kernel/sysdepend/cpu/rp2040/smp_atomic.c
    kernel/sysdepend/cpu/core/armv6m/cpu_cntl.c
    kernel/sysdepend/cpu/core/armv6m/dispatch.S
    kernel/sysdepend/cpu/core/armv6m/exc_hdr.c
    kernel/sysdepend/cpu/core/armv6m/interrupt.c
    kernel/sysdepend/cpu/core/armv6m/reset_hdl.c
)

set(LIB_SOURCES
    lib/libtm/libtm.c
    lib/libtm/libtm_printf.c
    lib/libtm/sysdepend/pico_rp2040/tm_com.c
    lib/libtm/sysdepend/no_device/tm_com.c
    lib/libtk/fastlock.c
    lib/libtk/fastmlock.c
    lib/libtk/kmalloc.c
    lib/libtk/sysdepend/cpu/rp2040/int_rp2040.c
    lib/libtk/sysdepend/cpu/rp2040/ptimer_rp2040.c
    lib/libtk/sysdepend/cpu/core/armv6m/int_armv6m.c
    lib/libtk/sysdepend/cpu/core/armv6m/wusec_armv6m.c
    lib/libbsp/sysdepend/cpu/rp2040/libbsp_pwm.c
)

set(DEVICE_SOURCES
    device/common/drvif/msdrvif.c
    device/adc/adc.c
    device/adc/sysdepend/rp2040/adc_rp2040.c
    device/i2c/i2c.c
    device/i2c/sysdepend/rp2040/i2c_rp2040.c
    device/ser/ser.c
    device/ser/sysdepend/rp2040/ser_rp2040.c
)

set(SOURCE_DIRS
    include
    config
    kernel/knlinc
)

set(SOURCES "")
set(SDK_SOURCES "")

if("${CONSOLE}" STREQUAL "usb_cdc")
    list(APPEND SOURCE_DIRS
        lib/libtm/sysdepend/pico_rp2040/usb
        "${PICO_SDK_PATH}/lib/tinyusb/src"
        "${PICO_SDK_PATH}/src/common/pico_base_headers/include"
        "${PICO_SDK_PATH}/src/common/pico_binary_info/include"
        "${PICO_SDK_PATH}/src/rp2040/pico_platform/include"
        "${PICO_SDK_PATH}/src/rp2_common/pico_platform_compiler/include"
        "${PICO_SDK_PATH}/src/rp2_common/pico_platform_sections/include"
        "${PICO_SDK_PATH}/src/rp2_common/pico_platform_panic/include"
        "${PICO_SDK_PATH}/src/rp2_common/pico_platform_common/include"
        "${PICO_SDK_PATH}/src/rp2_common/hardware_base/include"
        "${PICO_SDK_PATH}/src/rp2_common/hardware_irq/include"
        "${PICO_SDK_PATH}/src/rp2_common/hardware_resets/include"
        "${PICO_SDK_PATH}/src/rp2_common/hardware_sync/include"
        "${PICO_SDK_PATH}/src/rp2_common/hardware_sync_spin_lock/include"
        "${PICO_SDK_PATH}/src/rp2_common/hardware_timer/include"
        "${PICO_SDK_PATH}/src/rp2040/hardware_regs/include"
        "${PICO_SDK_PATH}/src/rp2040/hardware_structs/include"
    )
    list(APPEND SOURCES
        lib/libtm/sysdepend/pico_rp2040/usb/usb_console.c
        lib/libtm/sysdepend/pico_rp2040/usb/usb_descriptors.c
        lib/libtm/sysdepend/pico_rp2040/usb/usb_sdk_compat.c
        lib/libtm/sysdepend/pico_rp2040/usb/usb_tinyusb_glue.c)
    list(APPEND SDK_SOURCES
        "${PICO_SDK_PATH}/lib/tinyusb/src/tusb.c"
        "${PICO_SDK_PATH}/lib/tinyusb/src/common/tusb_fifo.c"
        "${PICO_SDK_PATH}/lib/tinyusb/src/device/usbd.c"
        "${PICO_SDK_PATH}/lib/tinyusb/src/device/usbd_control.c"
        "${PICO_SDK_PATH}/lib/tinyusb/src/class/cdc/cdc_device.c"
        "${PICO_SDK_PATH}/lib/tinyusb/src/portable/raspberrypi/rp2040/rp2040_usb.c"
        "${PICO_SDK_PATH}/lib/tinyusb/src/portable/raspberrypi/rp2040/dcd_rp2040.c")
endif()

if("${WIFI}" STREQUAL "cyw43")
    list(APPEND SOURCE_DIRS
        lib/libwifi/sysdepend/pico_rp2040
        lib/libtm/sysdepend/pico_rp2040/usb
        "${PICO_SDK_PATH}/lib/cyw43-driver/src"
        "${PICO_SDK_PATH}/lib/cyw43-driver/firmware"
        "${PICO_SDK_PATH}/src/rp2_common/pico_cyw43_driver/include"
        "${PICO_SDK_PATH}/src/boards/include"
        "${PICO_SDK_PATH}/src/common/pico_base_headers/include"
        "${PICO_SDK_PATH}/src/common/hardware_claim/include"
        "${PICO_SDK_PATH}/src/common/pico_binary_info/include"
        "${PICO_SDK_PATH}/src/common/pico_util/include"
        "${PICO_SDK_PATH}/src/rp2040/pico_platform/include"
        "${PICO_SDK_PATH}/src/rp2040/hardware_regs/include"
        "${PICO_SDK_PATH}/src/rp2040/hardware_structs/include"
        "${PICO_SDK_PATH}/src/rp2_common/pico_platform_common/include"
        "${PICO_SDK_PATH}/src/rp2_common/pico_platform_compiler/include"
        "${PICO_SDK_PATH}/src/rp2_common/pico_platform_sections/include"
        "${PICO_SDK_PATH}/src/rp2_common/pico_platform_panic/include"
        "${PICO_SDK_PATH}/src/rp2_common/hardware_base/include"
        "${PICO_SDK_PATH}/src/rp2_common/hardware_pio/include"
        "${PICO_SDK_PATH}/src/rp2_common/hardware_gpio/include"
        "${PICO_SDK_PATH}/src/rp2_common/hardware_dma/include"
        "${PICO_SDK_PATH}/src/rp2_common/hardware_irq/include"
        "${PICO_SDK_PATH}/src/rp2_common/hardware_sync/include"
        "${PICO_SDK_PATH}/src/rp2_common/hardware_sync_spin_lock/include"
        "${PICO_SDK_PATH}/src/rp2_common/hardware_clocks/include"
        "${PICO_SDK_PATH}/src/rp2_common/hardware_resets/include"
        "${PICO_SDK_PATH}/src/rp2_common/hardware_timer/include"
    )
    list(APPEND SOURCES
        lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c
        lib/libwifi/sysdepend/pico_rp2040/cyw43_utk_kernel.c
        lib/libwifi/sysdepend/pico_rp2040/cyw43_utk_compat.c)
    list(APPEND SDK_SOURCES
        "${PICO_SDK_PATH}/lib/cyw43-driver/src/cyw43_ll.c"
        "${PICO_SDK_PATH}/lib/cyw43-driver/src/cyw43_ctrl.c"
        "${PICO_SDK_PATH}/lib/cyw43-driver/src/cyw43_stats.c"
        "${PICO_SDK_PATH}/src/rp2_common/pico_cyw43_driver/cyw43_bus_pio_spi.c"
        "${PICO_SDK_PATH}/src/rp2_common/hardware_pio/pio.c"
        "${PICO_SDK_PATH}/src/rp2_common/hardware_dma/dma.c"
        "${PICO_SDK_PATH}/src/rp2_common/hardware_gpio/gpio.c")
endif()

if("${NET}" STREQUAL "lwip")
    list(APPEND SOURCE_DIRS
        "${PICO_SDK_PATH}/lib/lwip/src/include"
        lib/liblwip/sysdepend/pico_rp2040/include
        lib/liblwip/sysdepend/pico_rp2040)
    list(APPEND SOURCES
        lib/liblwip/sysdepend/pico_rp2040/sys_arch.c)
    list(APPEND SDK_SOURCES
        "${PICO_SDK_PATH}/lib/cyw43-driver/src/cyw43_lwip.c"
        "${PICO_SDK_PATH}/lib/lwip/src/core/init.c"
        "${PICO_SDK_PATH}/lib/lwip/src/core/def.c"
        "${PICO_SDK_PATH}/lib/lwip/src/core/dns.c"
        "${PICO_SDK_PATH}/lib/lwip/src/core/inet_chksum.c"
        "${PICO_SDK_PATH}/lib/lwip/src/core/ip.c"
        "${PICO_SDK_PATH}/lib/lwip/src/core/mem.c"
        "${PICO_SDK_PATH}/lib/lwip/src/core/memp.c"
        "${PICO_SDK_PATH}/lib/lwip/src/core/netif.c"
        "${PICO_SDK_PATH}/lib/lwip/src/core/pbuf.c"
        "${PICO_SDK_PATH}/lib/lwip/src/core/raw.c"
        "${PICO_SDK_PATH}/lib/lwip/src/core/stats.c"
        "${PICO_SDK_PATH}/lib/lwip/src/core/sys.c"
        "${PICO_SDK_PATH}/lib/lwip/src/core/altcp.c"
        "${PICO_SDK_PATH}/lib/lwip/src/core/altcp_alloc.c"
        "${PICO_SDK_PATH}/lib/lwip/src/core/altcp_tcp.c"
        "${PICO_SDK_PATH}/lib/lwip/src/core/tcp.c"
        "${PICO_SDK_PATH}/lib/lwip/src/core/tcp_in.c"
        "${PICO_SDK_PATH}/lib/lwip/src/core/tcp_out.c"
        "${PICO_SDK_PATH}/lib/lwip/src/core/timeouts.c"
        "${PICO_SDK_PATH}/lib/lwip/src/core/udp.c"
        "${PICO_SDK_PATH}/lib/lwip/src/core/ipv4/acd.c"
        "${PICO_SDK_PATH}/lib/lwip/src/core/ipv4/autoip.c"
        "${PICO_SDK_PATH}/lib/lwip/src/core/ipv4/dhcp.c"
        "${PICO_SDK_PATH}/lib/lwip/src/core/ipv4/etharp.c"
        "${PICO_SDK_PATH}/lib/lwip/src/core/ipv4/icmp.c"
        "${PICO_SDK_PATH}/lib/lwip/src/core/ipv4/igmp.c"
        "${PICO_SDK_PATH}/lib/lwip/src/core/ipv4/ip4_frag.c"
        "${PICO_SDK_PATH}/lib/lwip/src/core/ipv4/ip4.c"
        "${PICO_SDK_PATH}/lib/lwip/src/core/ipv4/ip4_addr.c"
        "${PICO_SDK_PATH}/lib/lwip/src/netif/ethernet.c")
endif()

if("${MQTT}" STREQUAL "1")
    list(APPEND SOURCES
        lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c)
    list(APPEND SDK_SOURCES
        "${PICO_SDK_PATH}/lib/lwip/src/apps/mqtt/mqtt.c")
endif()

add_executable(${EXE_NAME}
    ${APP_SOURCES}
    ${KERNEL_SOURCES}
    ${LIB_SOURCES}
    ${DEVICE_SOURCES}
    ${SOURCES}
    ${SDK_SOURCES})

set_target_properties(${EXE_NAME} PROPERTIES
    OUTPUT_NAME "${EXE_NAME}"
    SUFFIX ".elf"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

set(_c_include_dirs "")
foreach(_dir IN LISTS SOURCE_DIRS)
    if(IS_ABSOLUTE "${_dir}")
        list(APPEND _c_include_dirs "${_dir}")
    else()
        list(APPEND _c_include_dirs "${CMAKE_CURRENT_SOURCE_DIR}/${_dir}")
    endif()
endforeach()

target_include_directories(${EXE_NAME} PRIVATE
    $<$<COMPILE_LANGUAGE:C>:${_c_include_dirs}>)

# The makefile passes only -I../include -I../config -I../kernel/knlinc to the
# assembler (INCPATH), not the SDK or console include paths.
target_include_directories(${EXE_NAME} PRIVATE
    $<$<COMPILE_LANGUAGE:ASM>:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<$<COMPILE_LANGUAGE:ASM>:${CMAKE_CURRENT_SOURCE_DIR}/config>
    $<$<COMPILE_LANGUAGE:ASM>:${CMAKE_CURRENT_SOURCE_DIR}/kernel/knlinc>)

################################################################################
# Compiler and linker flags -- the same as build_make/pico_rp2040.mk
################################################################################

target_compile_options(${EXE_NAME} PRIVATE
    $<$<COMPILE_LANGUAGE:C>:-mcpu=cortex-m0plus;-mthumb;-ffreestanding;-std=gnu11;-O2;-g3;-mfloat-abi=soft>
    $<$<COMPILE_LANGUAGE:ASM>:-mcpu=cortex-m0plus;-mthumb;-ffreestanding;-x;assembler-with-cpp;-O2;-g3>)

target_compile_definitions(${EXE_NAME} PRIVATE
    $<$<COMPILE_LANGUAGE:C>:_PICO_RP2040_;APP_TEST=${TEST};TM_CONSOLE_UART=${CRASH_REPORT}>
    $<$<COMPILE_LANGUAGE:ASM>:_PICO_RP2040_>)

if("${CRASH_REPORT}" STREQUAL "0")
    target_compile_definitions(${EXE_NAME} PRIVATE
        $<$<COMPILE_LANGUAGE:C>:BOARD_UART0_FREE=1>)
endif()

if("${SMP}" STREQUAL "1")
    target_compile_definitions(${EXE_NAME} PRIVATE
        $<$<COMPILE_LANGUAGE:C>:CNF_SMP=1>
        $<$<COMPILE_LANGUAGE:ASM>:CNF_SMP=1>)
endif()

if("${CONSOLE}" STREQUAL "usb_cdc")
    target_compile_definitions(${EXE_NAME} PRIVATE
        $<$<COMPILE_LANGUAGE:C>:TM_CONSOLE_USB_CDC=1;NDEBUG>)
endif()

if("${WIFI}" STREQUAL "cyw43")
    target_compile_definitions(${EXE_NAME} PRIVATE
        $<$<COMPILE_LANGUAGE:C>:TM_WIFI_CYW43=1;CYW43_LWIP=$<IF:$<STREQUAL:${NET},lwip>,1,0>;CYW43_ENABLE_BLUETOOTH=0;CYW43_USE_OTP_MAC=1;PICO_CYW43_LOGGING_ENABLED=0;PICO_RP2040=1;PICO_32BIT=1;PICO_ON_DEVICE=1;PICO_BUILD=1;PICO_NO_HARDWARE=0;RASPBERRYPI_PICO_W=1;CYW43_DEFAULT_PIN_WL_REG_ON=23u;CYW43_DEFAULT_PIN_WL_DATA_OUT=24u;CYW43_DEFAULT_PIN_WL_DATA_IN=24u;CYW43_DEFAULT_PIN_WL_HOST_WAKE=24u;CYW43_DEFAULT_PIN_WL_CLOCK=29u;CYW43_DEFAULT_PIN_WL_CS=25u;NDEBUG>)
    target_compile_options(${EXE_NAME} PRIVATE
        $<$<COMPILE_LANGUAGE:C>:-ffunction-sections;-fdata-sections>)
endif()

if("${NET}" STREQUAL "lwip")
    target_compile_definitions(${EXE_NAME} PRIVATE
        $<$<COMPILE_LANGUAGE:C>:TM_NET_LWIP=1>)
endif()

if("${MQTT}" STREQUAL "1")
    target_compile_definitions(${EXE_NAME} PRIVATE
        $<$<COMPILE_LANGUAGE:C>:TM_MQTT=1>)
endif()

set(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/etc/linker/pico_rp2040/tkernel_map.ld")

target_link_options(${EXE_NAME} PRIVATE
    -mcpu=cortex-m0plus
    -mthumb
    -ffreestanding
    -nostartfiles
    -O2
    -g3
    -mfloat-abi=soft
    -T "${LINKER_SCRIPT}"
    "-Wl,-Map,${CMAKE_BINARY_DIR}/${EXE_NAME}.map"
    $<$<STREQUAL:${WIFI},cyw43>:-Wl,--gc-sections>)

################################################################################
# elf2uf2 host tool and the .uf2 image
#
# The converter is a small host program built from tools/elf2uf2 with the
# host C++ compiler, exactly as the makefile does.
################################################################################

find_program(HOST_CXX NAMES c++ g++ clang++ REQUIRED
    DOC "Host C++ compiler used to build tools/elf2uf2")

set(HOST_EXE_SUFFIX "")
if(CMAKE_HOST_WIN32)
    set(HOST_EXE_SUFFIX ".exe")
endif()
set(ELF2UF2 "${CMAKE_BINARY_DIR}/elf2uf2${HOST_EXE_SUFFIX}")

add_custom_command(
    OUTPUT "${ELF2UF2}"
    COMMAND "${HOST_CXX}" -O2 -std=c++14
            -I "${CMAKE_CURRENT_SOURCE_DIR}/tools/elf2uf2"
            -o "${ELF2UF2}"
            "${CMAKE_CURRENT_SOURCE_DIR}/tools/elf2uf2/main.cpp"
    DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/tools/elf2uf2/main.cpp"
            "${CMAKE_CURRENT_SOURCE_DIR}/tools/elf2uf2/elf.h"
            "${CMAKE_CURRENT_SOURCE_DIR}/tools/elf2uf2/boot/uf2.h"
    COMMENT "Building host tool: elf2uf2")

add_custom_target(elf2uf2 DEPENDS "${ELF2UF2}")
add_dependencies(${EXE_NAME} elf2uf2)

add_custom_command(TARGET ${EXE_NAME} POST_BUILD
    COMMAND "${CMAKE_SIZE}" "$<TARGET_FILE:${EXE_NAME}>"
    COMMAND "${ELF2UF2}" "$<TARGET_FILE:${EXE_NAME}>"
            "${CMAKE_BINARY_DIR}/${EXE_NAME}.uf2"
    COMMENT "Creating ${EXE_NAME}.uf2"
    VERBATIM)

# Keep the same convenience target name as the makefile: 'clean'.
add_custom_target(clean-all
    COMMAND "${CMAKE_COMMAND}" -E rm -rf "${CMAKE_BINARY_DIR}"
    COMMENT "Removing the build directory")
