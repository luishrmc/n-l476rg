set(PROTOTYPES "PROTOTYPES")

add_library(${PROTOTYPES} STATIC
)

target_sources(${PROTOTYPES} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/prototype.c
)

target_include_directories(${PROTOTYPES} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
    ${PROJECT_SOURCE_DIR}/app/inc/controllers
    ${PROJECT_SOURCE_DIR}/app/inc/drivers
    ${PROJECT_SOURCE_DIR}/app/inc/services
    ${PROJECT_SOURCE_DIR}/lib/${MCU}/hal-driver/Inc
    ${PROJECT_SOURCE_DIR}/lib/${MCU}/core/inc
)

target_compile_definitions(${PROTOTYPES} PUBLIC
    ${MCU}
    USE_HAL_DRIVER
)

target_link_libraries(${PROTOTYPES} PUBLIC m c)

# Ensure ranlib is run
add_custom_command(
    TARGET ${PROTOTYPES}
    POST_BUILD
    COMMAND "ranlib" ARGS ${CMAKE_CURRENT_BINARY_DIR}/libPROTOTYPES.a
)
