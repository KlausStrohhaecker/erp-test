# input variables: FIRMWARE_DIRNAME

cmake_minimum_required(VERSION 3.2)

project(mk-sysex)

set(CMAKE_C_FLAGS "${CMAKE_CXX_FLAGS} -O2 -Wall -Wextra -Werror")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -Wall -Wextra -Werror")

include_directories(src ../../${FIRMWARE_DIRNAME}/src/shared)

add_executable(mk-sysex src/mk-sysex.c CMakeLists.txt)

