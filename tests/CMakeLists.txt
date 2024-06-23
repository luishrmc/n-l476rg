# ------------------------------------ CONFIG ------------------------------------ #
# Override toolchain for native compilation
set(CMAKE_SYSTEM_NAME "Generic")
set(CMAKE_SYSTEM_PROCESSOR "x86_64")

# Set native compilers
set(CMAKE_C_COMPILER "/usr/bin/gcc")
set(CMAKE_CXX_COMPILER "/usr/bin/g++")

# Set specific CFLAGS for test directory
set(CMAKE_C_FLAGS "-g3")

# Set specific CPP for test directory
set(CMAKE_CXX_FLAGS "-g3")

# Add Unity library
CPMAddPackage("gh:ThrowTheSwitch/Unity#v2.6.0")

add_custom_target(unity_build ALL
    COMMAND "make" unity 
    COMMAND "ranlib" ${CMAKE_BINARY_DIR}/_deps/unity-build/libunity.a
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

# ------------------------------------ UNITY TEST PROTOTYPES ------------------------------------ #
add_subdirectory(prototype)

# ------------------------------------ UNITY TEST FOO ------------------------------------ #
set(TEST "TEST_FOO")
add_executable(${TEST} test_foo.c)
target_include_directories(${TEST} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/prototype/inc)
target_link_libraries(${TEST} PUBLIC unity PROTOTYPES)
add_test(NAME "TagProcDist" COMMAND ${TEST})
