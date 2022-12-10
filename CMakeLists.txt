# Pull in SDK (must be before project)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(nabu_usb_keyboard C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

add_executable(nabu_usb_keyboard
	nabu_usb_keyboard.c
	)

target_link_libraries(blink pico_stdlib)

# create map/bin/hex file etc.
pico_add_extra_outputs(blink)

add_compile_options(-Wall
	-Wno-format
	-Wno-unused-function
	-Wno-maybe-uninitialized
	)
