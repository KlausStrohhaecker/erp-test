cmake_minimum_required(VERSION 3.0)

message(STATUS "CMAKE_CURRENT_SOURCE_DIR: ${CMAKE_CURRENT_SOURCE_DIR}")

project(erp-readout)

set(CMAKE_C_FLAGS "${CMAKE_CXX_FLAGS} -O2 -Wall -Wno-bad-function-cast")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -Wall -Wno-bad-function-cast")

file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/src/shared/erp/*.c)

add_executable(erp-readout ${SOURCES})

target_include_directories(erp-readout PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/src/shared/erp)

target_link_libraries(erp-readout PRIVATE asound m)
