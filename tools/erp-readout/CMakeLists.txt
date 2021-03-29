cmake_minimum_required(VERSION 3.0)
project(erp-readout)

set(CMAKE_C_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")


include_directories(src/erp)

file(GLOB_RECURSE SOURCES src/erp/*.c)

add_executable(erp-readout src/main.c ${SOURCES})

target_link_libraries(erp-readout PRIVATE asound)
