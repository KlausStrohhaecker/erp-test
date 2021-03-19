# input variables: MAIN_APP_NAME, LINKER_SCRIPT_SUFFIX, IMAGE_SUFFIX, IMAGE_OBJECT_SUFFIX, IMAGE_OBJECT_TARGET_SUFFIX, LINKER_SCRIPTS_PATH

option(EVAL_BOARD "Evaluation Board specifics (GPIOs for LEDs etc)" OFF) #OFF by default
if(EVAL_BOARD)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D EVAL_BOARD")
endif(EVAL_BOARD)
unset(EVAL_BOARD) # <---- this is the important!!

option(SEPARATE_USB_DEVICE_IDS "Different USB IDs and strings for HS and FS port, for debug/test" OFF) #OFF by default
if(SEPARATE_USB_DEVICE_IDS)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D SEPARATE_USB_DEVICE_IDS")
endif(SEPARATE_USB_DEVICE_IDS)
unset(SEPARATE_USB_DEVICE_IDS) # <---- this is the important!!

set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mcpu=cortex-m4 -mthumb")
set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wno-pragmas -fno-builtin -nodefaultlibs -nostdlib")
set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fno-common -fmessage-length=0 -ffunction-sections -fdata-sections ")
set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wl,-print-memory-usage")

add_compile_options("-DCORE_M4")

file(GLOB_RECURSE SOURCES ./src/*.c ../shared/*.c)

add_executable(${MAIN_APP_NAME} ${SOURCES})

target_include_directories (${MAIN_APP_NAME} PRIVATE "./src")

target_link_options(${MAIN_APP_NAME} PUBLIC -Wl,-Map,output.map -Wl,--gc-sections)
target_link_directories(${MAIN_APP_NAME} PUBLIC ${LINKER_SCRIPTS_PATH})

set_target_properties(${MAIN_APP_NAME} PROPERTIES LINKER_LANGUAGE C)
set_target_properties(${MAIN_APP_NAME} PROPERTIES LINK_FLAGS "-T${MAIN_APP_NAME}${LINKER_SCRIPT_SUFFIX}")
set_target_properties(${MAIN_APP_NAME} PROPERTIES COMPILE_FLAGS "${CMAKE_C_FLAGS}")

add_custom_command(OUTPUT ${MAIN_APP_NAME}${IMAGE_OBJECT_SUFFIX}
    DEPENDS ${MAIN_APP_NAME}
    COMMAND arm-none-eabi-objcopy --verbose --strip-all -O binary --remove-section=.ARM.attributes --remove-section=".bss*" --remove-section=".noinit*" ${MAIN_APP_NAME} ${MAIN_APP_NAME}${IMAGE_SUFFIX}
    COMMAND arm-none-eabi-objcopy --verbose -I binary -O elf32-littlearm -B arm --redefine-sym _binary_${MAIN_APP_NAME}${IMAGE_OBJCOPY_SUFFIX}_start=image_start --redefine-sym _binary_${MAIN_APP_NAME}${IMAGE_OBJCOPY_SUFFIX}_size=image_size ${MAIN_APP_NAME}${IMAGE_SUFFIX} ${MAIN_APP_NAME}${IMAGE_OBJECT_SUFFIX}
)

add_custom_target(${MAIN_APP_NAME}${IMAGE_OBJECT_TARGET_SUFFIX} ALL
    COMMENT "Creating linkable object from image of '${MAIN_APP_NAME}': DONE"
    DEPENDS ${MAIN_APP_NAME}${IMAGE_OBJECT_SUFFIX}
)
