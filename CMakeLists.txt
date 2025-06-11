# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.10...4.0)
include(CMakePrintHelpers)

# enable compile commands for use by IDE autocompletion
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
include($ENV{PICO_EXTRAS_PATH}/external/pico_extras_import.cmake)

if (PICO_SDK_VERSION_STRING VERSION_LESS "2.1.1")
  message(FATAL_ERROR "Raspberry Pi Pico SDK version 2.1.1 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

project(pio_blink C CXX ASM)

pico_sdk_init()

add_library(wm8960 
wm8960/sparkfun_wm8960.cpp
wm8960/wm8960.cpp)

add_executable(blinkPio blink.cpp)

pico_generate_pio_header(blinkPio ${CMAKE_CURRENT_LIST_DIR}/pio_blink.pio)

#pull in common dependencies
target_link_libraries(blinkPio 
pico_stdlib
hardware_pio
hardware_gpio
hardware_clocks
hardware_i2c
wm8960
pico_audio_i2s)

target_link_libraries(wm8960 
pico_stdlib
hardware_pio
hardware_gpio
hardware_clocks
hardware_i2c)

pico_add_extra_outputs(blinkPio)