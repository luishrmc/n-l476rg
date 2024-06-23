cmake_minimum_required(VERSION 3.22)

set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

set(PROJECT_NAME STM_FreeRTOS)
set(OPENOCD_CFG_NAME ${PROJECT_NAME}.cfg)

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

if(${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Linux")
    set(EXE_SUFFIX "")
elseif(${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Windows")
    set(EXE_SUFFIX ".exe")
endif()

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED TRUE)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_C_FLAGS_DEBUG "-Og -g3")
set(CMAKE_C_FLAGS_RELEASE "-O1")

# To enable colored terminal
option(FORCE_COLORED_OUTPUT "Always produce ANSI-colored output (GNU/Clang only.)" ON)
if (${FORCE_COLORED_OUTPUT})
    if ("${CMAKE_C_COMPILER_ID}" STREQUAL "GNU")
        add_compile_options(-fdiagnostics-color=always)
    elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
        add_compile_options(-fcolor-diagnostics)
    endif()
endif()

# Reduce binary size. Might be used to code to fit in the MCU
option(GARBAGE_COLLECT_SECTIONS "Use -f{function,data}-sections and -Wl,--gc-sections." ON)
if (${GARBAGE_COLLECT_SECTIONS})
    add_compile_options(-ffunction-sections -fdata-sections)
    add_link_options(-Wl,--gc-sections)
endif()

## TOOLCHAIN CONFIGURATION FILE

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}")
find_package(toolchain MODULE REQUIRED)

# CMAKE project name, languages and version
project(${PROJECT_NAME}
    LANGUAGES C CXX ASM
    VERSION 0.0.1
)

set(EXE_NAME ${PROJECT_NAME}.elf)

add_subdirectory(Drivers)
add_subdirectory(Libraries/FreeRTOS)
add_subdirectory(Libraries/SEGGER)

add_executable(${EXE_NAME})

target_sources( ${EXE_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/freertos.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/stm32l4xx_hal_msp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/stm32l4xx_hal_timebase_tim.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/stm32l4xx_it.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/syscalls.c
    # ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/sysmem.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/system_stm32l4xx.c

    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Startup/startup_stm32l476rgtx.s
)

target_include_directories(${EXE_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32L4xx/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32L4xx_HAL_Driver/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy
)

target_compile_options(${EXE_NAME} PRIVATE
    -Wall
    -fstack-usage
)

target_link_options(${EXE_NAME} PRIVATE
    -T${CMAKE_CURRENT_SOURCE_DIR}/STM32L476RGTX.ld
    -Wl,--gc-sections

    -static # The -static linker flag is used to instruct the linker to link the program statically against libraries, rather than dynamically. 
            # When you compile a program, it typically depends on various libraries, which can be linked either statically or dynamically.

    #-shared # The -shared linker flag is used to create a shared object file (.so) on Unix-like systems (such as Linux). 
            # A shared object file is a dynamically linked library that contains functions and variables which can be used by other programs.
            # When you compile a program that uses shared libraries, the linker needs to know that it should create a shared object file instead of an executable.

    -specs=nano.specs
    -specs=nosys.specs

    -Wl,--print-memory-usage 
    -Wl,-Map=${PROJECT_NAME}.map
    -Wl,--wrap=malloc
    -Wl,--wrap=free
    
    -lc 
    -lm 
)

target_link_libraries(${EXE_NAME} PRIVATE 
    Drivers
    FreeRTOS
    SEGGER
)

target_compile_definitions(${EXE_NAME} PRIVATE
    STM32L476xx 
    USE_HAL_DRIVER
)

add_custom_command(
    TARGET ${EXE_NAME}
    POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} ARGS -O binary ${EXE_NAME} ${PROJECT_NAME}.bin
)
