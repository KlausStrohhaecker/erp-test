cmake_minimum_required(VERSION 3.2)

project(erp-test)


set(FIRMWARE_DIRNAME firmware)
set(TOOLS_DIRNAME    tools)

add_subdirectory(${FIRMWARE_DIRNAME})
add_subdirectory(${TOOLS_DIRNAME})
