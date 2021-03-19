cmake_minimum_required(VERSION 3.0)
project(erp-readout)

set(CMAKE_C_FLAGS "${CMAKE_CXX_FLAGS} -O2 -Wall -Wextra")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -Wall -Wextra")

include_directories(.)
add_executable(erp-readout main.c erp_decode.c)
target_link_libraries(erp-readout PRIVATE asound)
