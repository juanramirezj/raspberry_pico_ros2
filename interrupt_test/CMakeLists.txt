cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(interrupt_test C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

#link_directories( /home/administrator/micro_ros_ws/src/micro_ros_raspberrypi_pico_sdk/libmicroros)
add_executable(interrupt_test
    main.c
)

target_link_libraries(interrupt_test
   pico_stdlib
)


#target_include_directories(motor4_ros PUBLIC
#    /home/administrator/micro_ros_ws/src/micro_ros_raspberrypi_pico_sdk/libmicroros/include
#)

# SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
# SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

# Configure Pico
pico_enable_stdio_usb(interrupt_test 1)
pico_enable_stdio_uart(interrupt_test 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# Generate UF2
pico_add_extra_outputs( interrupt_test )
