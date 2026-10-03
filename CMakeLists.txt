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

# Enable compile commands for IDE autocompletion
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Use your own TinyUSB path
set(PICO_TINYUSB_PATH "${PICO_SDK_PATH}/lib/tinyusb")

# Initialize Pico SDK (must be after TinyUSB flags but before project)
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)
if(NOT PICO_EXTRAS_PATH AND DEFINED ENV{PICO_EXTRAS_PATH})
  set(PICO_EXTRAS_PATH "$ENV{PICO_EXTRAS_PATH}")
endif()
if(NOT PICO_EXTRAS_PATH)
  message(FATAL_ERROR "Set PICO_EXTRAS_PATH to the pico-extras source directory")
endif()
include(${PICO_EXTRAS_PATH}/external/pico_extras_import.cmake)

if (PICO_SDK_VERSION_STRING VERSION_LESS "2.1.1")
  message(FATAL_ERROR "Raspberry Pi Pico SDK version 2.1.1 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

project(blinkPio C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

# include(FetchContent)

# FetchContent_Declare(
#   tinyusb
#   GIT_REPOSITORY https://github.com/hathach/tinyusb.git
#   GIT_TAG master
# )

# FetchContent_MakeAvailable(tinyusb)

# === Main executable ===
add_executable(blinkPio 
  src/blink.cpp
  src/usb_descriptors.c
  src/quirk_os_guessing.c
)

target_compile_definitions(blinkPio PRIVATE
            AUDIO_FREQ_MAX=48000

            # ours are zero based, so say so
            PICO_USBDEV_USE_ZERO_BASED_INTERFACES=1

            # need large descriptor
            PICO_USBDEV_MAX_DESCRIPTOR_SIZE=256


            PICO_USBDEV_ISOCHRONOUS_BUFFER_STRIDE_TYPE=1
            PICO_USBDEV_ENABLE_DEBUG_TRAgCE

            PICO_AUDIO_I2S_MONO_OUTPUT=0
            PICO_AUDIO_I2S_MONO_INPUT=0
    )

# === Link libraries ===
target_link_libraries(blinkPio 
  tinyusb_device
  tinyusb_board
  pico_stdlib
  hardware_pio
  hardware_gpio
  hardware_clocks
  hardware_i2c
  pico_audio_i2s
  usb_device
  pico_multicore
)

# === Include paths ===
target_include_directories(blinkPio PUBLIC
  headers
  ${CMAKE_BINARY_DIR}/headers
)

target_include_directories(${PROJECT_NAME}  PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
)

pico_enable_stdio_usb(blinkPio 1)
pico_enable_stdio_uart(blinkPio 1)

# === Generate UF2, HEX, etc. ===
pico_add_extra_outputs(blinkPio)
