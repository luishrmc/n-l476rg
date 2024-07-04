cmake_minimum_required(VERSION 3.22)

set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

set(PROJECT_NAME freeRTOS)
set(EXE_NAME ${PROJECT_NAME}.elf)

#[[ ccache is a compiler cache designed to speed up the compilation process by 
    caching the results of previous compilations. It's not a feature of CMake itself 
    but a separate tool that can be used in conjunction with various build systems
#]]
find_program(CCACHE_FOUND ccache)

if(CCACHE_FOUND)
    message(STATUS "Found CCACHE!")
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")
find_package(toolchain MODULE REQUIRED)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ------------------------------------ C CONFIG ------------------------------------ #
set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_C_FLAGS_DEBUG "-g3")
set(CMAKE_C_FLAGS_RELEASE "-O1")

# ------------------------------------ CPP CONFIG ------------------------------------ #
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS_DEBUG "-g3")
set(CMAKE_CXX_FLAGS_RELEASE "-O1")

# ------------------------------------ PROJECT ------------------------------------ #
# ! STAY HERE !
project(${PROJECT_NAME}
    VERSION 0.0.1
    LANGUAGES C CXX ASM
)

# ------------------------------------ FREE RTOS ------------------------------------ #
option(ENABLE_FREERTOS "Enable FreeRTOS" ON)

if(ENABLE_FREERTOS)
    # Adding the FreeRTOS-Kernel subdirectory
    set(FREERTOS_KERNEL_PATH "${CMAKE_CURRENT_BINARY_DIR}/_deps/freertos-kernel-src")

    # Add the freertos_config for FreeRTOS-Kernel
    add_library(freertos_config INTERFACE)

    target_include_directories(freertos_config
        INTERFACE
        "${CMAKE_CURRENT_SOURCE_DIR}/config"
        "${CMAKE_CURRENT_SOURCE_DIR}/lib/SEGGER/SystemView_Src_V354/Sample/FreeRTOSV11"
        "${CMAKE_CURRENT_SOURCE_DIR}/lib/SEGGER/SystemView_Src_V354/SEGGER"
        "${CMAKE_CURRENT_SOURCE_DIR}/lib/SEGGER/SystemView_Src_V354/Config"
    )

    # Select the heap port.  values between 1-4 will pick a heap.
    set(FREERTOS_HEAP "4" CACHE STRING "" FORCE)

    # Select the native compile PORT
    set(FREERTOS_PORT "GCC_ARM_CM4F" CACHE STRING "" FORCE)
endif()

# ------------------------------------ FREE RTOS ------------------------------------ #
option(ENABLE_ZEPHYR "Enable Zephyr" OFF)

if(ENABLE_ZEPHYR)
    # //TODO
endif()

# ------------------------------------ OPTIONS ------------------------------------ #
option(ENABLE_WARNINGS "Enable to add warnings to a target." ON)

option(ENABLE_WARNINGS_AS_ERRORS "Enable to treat warnings as errors." OFF)

# ------------------------------------ CMAKE MODULES ------------------------------------ #
include(ConfigSafeGuards)
include(AddGitSubmodule)
include(Docs)

if(ENABLE_WARNINGS)
    include(Warnings)
endif()

# ------------------------------------ EXTERNAL LIBRARIES ------------------------------------ #
message(STATUS "Using CPM")
include(CPM)

# ------------------------------------ PROJECT DIRECTORIES  ----------------------------------- #
add_git_submodule(lib/${MCU}/hal-driver)
add_subdirectory(config)
add_subdirectory(app)
add_subdirectory(lib)

# ------------------------------------ FREE RTOS INCLUDE/COMPILE ------------------------------------ #
if(ENABLE_FREERTOS)
    CPMAddPackage("gh:FreeRTOS/FreeRTOS-Kernel#V11.1.0")
endif()

if(ENABLE_ZEPHYR)
    # //TODO
    # CPMAddPackage("gh:zephyrproject-rtos/zephyr#v3.6.0")
endif()
