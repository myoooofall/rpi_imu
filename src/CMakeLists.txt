cmake_minimum_required(VERSION 3.14)
# chip option
option(CM4_VERSION "compile for CM4" OFF)
option(ROCKS_VERSION "compile for ROCKS" ON)
if(ROCKS_VERSION)
    set(CMAKE_C_COMPILER aarch64-none-linux-gnu-gcc)
    set(CMAKE_CXX_COMPILER aarch64-none-linux-gnu-g++)
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

find_package(fmt REQUIRED)
find_package(Threads REQUIRED)

include_directories(
    ${fmt_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/share/zos/project/include
    ${CMAKE_CURRENT_SOURCE_DIR}/share
    ${CMAKE_CURRENT_SOURCE_DIR}/share/utils
)
    
set(LIBS
    ${LIBS}
    Threads::Threads
    fmt::fmt
)

file(GLOB SHARED_SOURCE_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/share/utils/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/share/control/*.cpp
)

file(GLOB TEST_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/wifiz.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/robotz.cpp
)

# chip option
if(CM4_VERSION)
    file(GLOB TEST_FILES
        ${TEST_FILES}
        ${CMAKE_CURRENT_SOURCE_DIR}/device_CM4.cpp
    )
    add_executable (S01_CM4 ${TEST_FILES} ${SHARED_SOURCE_FILES})
    target_link_libraries (S01_CM4 ${LIBS})
    target_compile_definitions(S01_CM4 PRIVATE CM4_VERSION)
elseif(ROCKS_VERSION)
    file(GLOB TEST_FILES
        ${TEST_FILES}
        ${CMAKE_CURRENT_SOURCE_DIR}/device_ROCKS.cpp
    )
    add_executable (S01_ROCKS ${TEST_FILES} ${SHARED_SOURCE_FILES})
    target_link_libraries (S01_ROCKS ${LIBS})
    target_compile_definitions(S01_ROCKS PRIVATE ROCKPIS_VERSION)
endif()