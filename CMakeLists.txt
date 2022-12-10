cmake_minimum_required(VERSION 3.12)

# Pull in the Pico SDK.  This correctly pulls in TinyUSB for us.
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# gets PROJECT name for the example (e.g. <BOARD>-<DIR_NAME>)
#family_get_project_name(PROJECT ${CMAKE_CURRENT_LIST_DIR})

project(nabu_usb_keyboard)
pico_sdk_init()

# Checks this example is valid for the family and initializes the project
# This calls pico_sdk_init() for us.
#family_initialize_project(nabu_usb_keyboard ${CMAKE_CURRENT_LIST_DIR})

add_executable(nabu_usb_keyboard
	nabu_usb_keyboard.c
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
	-Wno-format
	-Wno-unused-function
	-Wno-maybe-uninitialized
	)

target_include_directories(nabu_usb_keyboard PUBLIC
	${CMAKE_CURRENT_LIST_DIR}
	)
