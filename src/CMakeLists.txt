cmake_minimum_required(VERSION 3.4.1)
project(S01 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
find_package(Threads REQUIRED)
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/share
    ${CMAKE_CURRENT_SOURCE_DIR}/control
    ${CMAKE_CURRENT_SOURCE_DIR}/unit_test
)
set(LIBS
    ${LIBS}
    Threads::Threads
)
file(GLOB SHARED_SOURCE_FILES
    ${SHARED_SOURCE_FILES}
    ${CMAKE_CURRENT_SOURCE_DIR}/share/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/control/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/unit_test/*.cpp
)
add_executable(test ${SHARED_SOURCE_FILES})
target_link_libraries (test ${LIBS})
