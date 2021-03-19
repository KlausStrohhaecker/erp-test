# input variables: MAIN_APP_NAME, FLASH_APP_NAME, LINKER_SCRIPT_SUFFIX, SYSEX_SUFFIX, SYSEX_TARGET_SUFFIX, IMAGE_OBJECT_SUFFIX, IMAGE_OBJECT_TARGET_SUFFIX, LINKER_SCRIPTS_PATH

option(EVAL_BOARD "Evaluation Board specifics (GPIOs for LEDs etc)" OFF) #OFF by default
if(EVAL_BOARD)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D EVAL_BOARD")
endif(EVAL_BOARD)
unset(EVAL_BOARD) # <---- this is the important!!

set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mcpu=cortex-m4 -mthumb")
set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Os")
set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -fno-builtin -nodefaultlibs -nostdlib")
set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fno-common -fmessage-length=0 -ffunction-sections -fdata-sections ")
set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wl,-print-memory-usage")

add_compile_options("-DCORE_M4")

file(GLOB_RECURSE SOURCES ./src/*.c ../shared/*.c)

add_executable(${FLASH_APP_NAME} ${SOURCES} ../${MAIN_APP_NAME}/${MAIN_APP_NAME}${IMAGE_OBJECT_SUFFIX})

set_source_files_properties(../${MAIN_APP_NAME}/${MAIN_APP_NAME}${IMAGE_OBJECT_SUFFIX} PROPERTIES GENERATED TRUE)

target_include_directories (${FLASH_APP_NAME} PRIVATE "./src")

target_link_options(${FLASH_APP_NAME} PUBLIC -Wl,-Map,output.map -Wl,--gc-sections)
target_link_directories(${FLASH_APP_NAME} PUBLIC ${LINKER_SCRIPTS_PATH})

set_target_properties(${FLASH_APP_NAME} PROPERTIES LINKER_LANGUAGE C)
set_target_properties(${FLASH_APP_NAME} PROPERTIES LINK_FLAGS "-T${FLASH_APP_NAME}${LINKER_SCRIPT_SUFFIX}")
set_target_properties(${FLASH_APP_NAME} PROPERTIES COMPILE_FLAGS "${CMAKE_C_FLAGS}")

add_custom_command(OUTPUT ${FLASH_APP_NAME}${SYSEX_SUFFIX}
    DEPENDS ${FLASH_APP_NAME}
    DEPENDS ${MAIN_APP_NAME}${IMAGE_OBJECT_TARGET_SUFFIX}
    DEPENDS mk-sysex
    COMMAND arm-none-eabi-objcopy --verbose --strip-all -O binary --remove-section=.ARM.attributes --remove-section=".bss*" --remove-section=".noinit*" ${FLASH_APP_NAME} ${FLASH_APP_NAME}${IMAGE_SUFFIX}
    COMMAND rm -f *${SYSEX_SUFFIX}
    # generate a SysEx file with the standard name containing the extracted version info from the image
    COMMAND mk-sysex ${FLASH_APP_NAME}${IMAGE_SUFFIX}
    # copy the SysEx file to the original name as that is the dependency use
    COMMAND cp *${SYSEX_SUFFIX} ${FLASH_APP_NAME}${SYSEX_SUFFIX}
    COMMAND echo created file duplicates: ; *${SYSEX_SUFFIX}
)

add_custom_target(${FLASH_APP_NAME}${SYSEX_TARGET_SUFFIX} ALL
    COMMENT "Creating MIDI SysEx files from image of '${FLASH_APP_NAME}': DONE"
    DEPENDS ${FLASH_APP_NAME}${SYSEX_SUFFIX}
)
