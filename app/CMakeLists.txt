add_executable(${EXE_NAME})

set(CTRL "src/controllers")
set(SERV "src/services")
set(DRV "src/drivers")
set(CORE "lib/${MCU}/core/src")

target_sources(${EXE_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/${CTRL}/mainController.c

    ${CMAKE_CURRENT_SOURCE_DIR}/${DRV}/uartDriver.c

    ${PROJECT_SOURCE_DIR}/${CORE}/freertos.c
    ${PROJECT_SOURCE_DIR}/${CORE}/startup_stm32l476rgtx.s
    ${PROJECT_SOURCE_DIR}/${CORE}/stm32l4xx_hal_timebase_tim.c
    ${PROJECT_SOURCE_DIR}/${CORE}/stm32l4xx_it.c
    ${PROJECT_SOURCE_DIR}/${CORE}/syscalls.c
    ${PROJECT_SOURCE_DIR}/${CORE}/sysmem.c
    ${PROJECT_SOURCE_DIR}/${CORE}/system_stm32l4xx.c
)

target_include_directories(${EXE_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/inc/controllers
    ${CMAKE_CURRENT_SOURCE_DIR}/inc/services
    ${CMAKE_CURRENT_SOURCE_DIR}/inc/drivers
    ${CMAKE_BINARY_DIR}/config_files/include
)

target_link_options(${EXE_NAME} PRIVATE
    -T${PROJECT_SOURCE_DIR}/${CORE}/${LINKER_SCRIPT}
    ${RUNTIME_LIBRARY}
    ${RUNTIME_LIBRARY_SYSCALLS}
    -Wl,--print-memory-usage
    -Wl,-Map=${PROJECT_NAME}.map
    -Wl,--gc-sections
    -static
    -Wl,--start-group
    -lc
    -lm
    -Wl,--end-group
)

target_link_libraries(${EXE_NAME} PRIVATE
    ${MCU}_HAL
    UTILS
    freertos_kernel
    freertos_config
)

target_compile_definitions(${EXE_NAME} PRIVATE
    ${MCU}
    USE_HAL_DRIVER
)

if(${ENABLE_WARNINGS})
    target_set_warnings(
        TARGET
        ${EXE_NAME}
        ENABLE
        ${ENABLE_WARNINGS}
        AS_ERRORS
        ${ENABLE_WARNINGS_AS_ERRORS})
endif()

add_custom_command(
    TARGET ${EXE_NAME}
    POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} ARGS -O binary ${EXE_NAME} ${PROJECT_NAME}.bin
)
