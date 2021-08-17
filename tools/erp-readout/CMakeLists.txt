cmake_minimum_required(VERSION 3.0)
project(erp-readout)

set(CMAKE_C_FLAGS "${CMAKE_CXX_FLAGS} -O2 -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -Wall")

file(GLOB_RECURSE SOURCES ${CMAKE_SOURCE_DIR}/src/*.c ${CMAKE_SOURCE_DIR}/../../firmware/src/shared/erp/*.c)

add_executable(erp-readout ${SOURCES})

target_include_directories(erp-readout PRIVATE ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/../../firmware/src/shared/erp)
message(STATUS "CMAKE_SOURCE_DIR: ${CMAKE_SOURCE_DIR}")

target_link_libraries(erp-readout PRIVATE asound m)
