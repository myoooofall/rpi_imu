cmake_minimum_required(VERSION 3.14)

# chip option
option(CM4_VERSION "compile for CM4" ON)
option(ROCKS_VERSION "compile for ROCKS" OFF)
if(ROCKS_VERSION)
    set(CMAKE_C_COMPILER aarch64-none-linux-gnu-gcc)
    set(CMAKE_CXX_COMPILER aarch64-none-linux-gnu-g++)
    set(CMAKE_EXE_LINKER_FLAGS "-static")
endif()


project(S01 LANGUAGES CXX)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})


set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")

if(CM4_VERSION)
    set(LIBS wiringPi)
elseif(ROCKS_VERSION)
    include_directories(
        $ENV{HOME}/aarch64-toolchains
        $ENV{HOME}/aarch64-toolchains/include
        $ENV{HOME}/aarch64-toolchains/fmt-aarch64
        $ENV{HOME}/aarch64-toolchains/mraa-aarch64/include
    )
    set(LIBS
        $ENV{HOME}/aarch64-toolchains/mraa-aarch64/lib/libmraa.a
    )
endif()

find_package(Threads REQUIRED)
find_package(fmt REQUIRED)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/share
    ${CMAKE_CURRENT_SOURCE_DIR}/control
    ${CMAKE_CURRENT_SOURCE_DIR}/unit_test
    ${CMAKE_CURRENT_SOURCE_DIR}/share/zos/project/core/include
    ${CMAKE_CURRENT_SOURCE_DIR}/share/utils
    ${CMAKE_CURRENT_SOURCE_DIR}/robot
    ${CMAKE_CURRENT_SOURCE_DIR}
)
set(LIBS
    ${LIBS}
    Threads::Threads
    fmt::fmt
)
file(GLOB SHARED_SOURCE_FILES
    ${SHARED_SOURCE_FILES}
    ${CMAKE_CURRENT_SOURCE_DIR}/share/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/control/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/unit_test/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/share/utils/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/robot/*.cpp
)
# chip option
if(CM4_VERSION)
    file(GLOB SHARED_SOURCE_FILES
        ${SHARED_SOURCE_FILES}
        ${CMAKE_CURRENT_SOURCE_DIR}/device_CM4.cpp
    )
elseif(ROCKS_VERSION)
    file(GLOB SHARED_SOURCE_FILES
        ${SHARED_SOURCE_FILES}
        ${CMAKE_CURRENT_SOURCE_DIR}/device_ROCKS.cpp
    )
endif()


file(GLOB TEST_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/unit_test/*.cpp
)

foreach(test_file ${TEST_FILES})
    file(RELATIVE_PATH test_file_name ${CMAKE_CURRENT_SOURCE_DIR} ${test_file})
    string(REPLACE ".cpp" "" test_name ${test_file_name})
    string(REPLACE "test/" "" test_name ${test_name})
    message("test file: " ${test_file} " -- " ${test_name})

    add_executable (${test_name} ${test_file_name} ${SHARED_SOURCE_FILES})
    target_link_libraries (${test_name} ${LIBS})
    if(CM4_VERSION)
        target_compile_definitions(${test_name} PRIVATE CM4_VERSION)
    elseif(ROCKS_VERSION)
        target_compile_definitions(${test_name} PRIVATE ROCKPIS_VERSION)
    endif()
endforeach()
