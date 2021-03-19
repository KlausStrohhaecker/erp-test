# input variables: FIRMWARE_DIRNAME

cmake_minimum_required(VERSION 3.2)

project (erp-test-hardware NONE)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR ARM)

if(MINGW OR CYGWIN OR WIN32)
    set(UTIL_SEARCH_CMD where)
elseif(UNIX OR APPLE)
    set(UTIL_SEARCH_CMD which)
endif()

set(TOOLCHAIN_PREFIX arm-none-eabi-)

execute_process(
  COMMAND ${UTIL_SEARCH_CMD} ${TOOLCHAIN_PREFIX}gcc
  OUTPUT_VARIABLE BINUTILS_PATH
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

get_filename_component(ARM_TOOLCHAIN_DIR ${BINUTILS_PATH} DIRECTORY)

# Without those CMake is not able to pass test compilation check ("-rdynamic" bug)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")
set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "")

set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_ASM_COMPILER ${CMAKE_C_COMPILER})
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}g++)

set(CMAKE_OBJCOPY ${ARM_TOOLCHAIN_DIR}/${TOOLCHAIN_PREFIX}objcopy CACHE INTERNAL "objcopy tool")
set(CMAKE_SIZE_UTIL ${ARM_TOOLCHAIN_DIR}/${TOOLCHAIN_PREFIX}size CACHE INTERNAL "size tool")

set(CMAKE_FIND_ROOT_PATH ${BINUTILS_PATH})
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

include_directories ("./src/shared")

#variables passed to the lower scripts
set(MAIN_APP_NAME               application)
set(FLASH_APP_NAME              in-app-flasher)
set(LINKER_SCRIPT_SUFFIX        .ld)
set(IMAGE_SUFFIX                .image)
set(IMAGE_TARGET_SUFFIX         _image)
set(IMAGE_OBJCOPY_SUFFIX        _image)
set(IMAGE_OBJECT_SUFFIX         _image.o)
set(IMAGE_OBJECT_TARGET_SUFFIX  _image_object)
set(SYSEX_SUFFIX                .syx)
set(SYSEX_TARGET_SUFFIX         _sysex)
set(LINKER_SCRIPTS_PATH          ${CMAKE_SOURCE_DIR}/${FIRMWARE_DIRNAME}/ld)

add_subdirectory (src/${MAIN_APP_NAME})
add_subdirectory (src/${FLASH_APP_NAME})
