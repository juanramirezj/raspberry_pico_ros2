cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(pico-neopixel C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(pico-neopixel 
   pico-neopixel.c 
)

pico_generate_pio_header(pico-neopixel ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio)

target_sources(pico-neopixel PRIVATE pico-neopixel.c)

pico_enable_stdio_usb(pico-neopixel 1)
pico_enable_stdio_uart(pico-neopixel 0)
pico_add_extra_outputs(pico-neopixel)
target_link_libraries(pico-neopixel PRIVATE pico_stdlib hardware_pio)
pico_add_extra_outputs(pico-neopixel)