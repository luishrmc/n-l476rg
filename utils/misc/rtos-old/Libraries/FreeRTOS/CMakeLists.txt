set(FREERTOS_LIBRARY "FreeRTOS")

add_library( ${FREERTOS_LIBRARY} STATIC

)

target_sources( ${FREERTOS_LIBRARY} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/croutine.c
    ${CMAKE_CURRENT_SOURCE_DIR}/event_groups.c
    ${CMAKE_CURRENT_SOURCE_DIR}/list.c
    ${CMAKE_CURRENT_SOURCE_DIR}/queue.c
    ${CMAKE_CURRENT_SOURCE_DIR}/stream_buffer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/tasks.c
    ${CMAKE_CURRENT_SOURCE_DIR}/timers.c

    ${CMAKE_CURRENT_SOURCE_DIR}/portable/MemMang/heap_4.c

    ${CMAKE_CURRENT_SOURCE_DIR}/portable/GCC/ARM_CM4F/port.c
)

target_include_directories( ${FREERTOS_LIBRARY} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/portable/GCC/ARM_CM4F

    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Inc

    ${CMAKE_CURRENT_SOURCE_DIR}/../SEGGER/OS
    ${CMAKE_CURRENT_SOURCE_DIR}/../SEGGER/SEGGER
    ${CMAKE_CURRENT_SOURCE_DIR}/../SEGGER/Config

)

target_compile_options( ${FREERTOS_LIBRARY} PRIVATE
    -Wall
    -fstack-usage
)

target_compile_definitions( ${FREERTOS_LIBRARY} PRIVATE
    STM32L476xx 
    USE_HAL_DRIVER
)
