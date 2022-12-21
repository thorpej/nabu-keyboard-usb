cmake_minimum_required(VERSION 3.12)

# Pull in the Pico SDK.  This correctly pulls in TinyUSB for us.
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(nabu_usb_keyboard)
pico_sdk_init()

add_executable(nabu_usb_keyboard
	nabu_usb_keyboard.c
	usb_descriptors.c
	)

target_link_libraries(nabu_usb_keyboard
	pico_stdlib
	pico_sync
	pico_multicore
	tinyusb_board
	tinyusb_device
	)

# create map/bin/hex file etc.
pico_add_extra_outputs(nabu_usb_keyboard)

add_compile_options(-Wall
	-Wno-unused-function
	)

target_include_directories(nabu_usb_keyboard PUBLIC
	${CMAKE_CURRENT_LIST_DIR}
	)
