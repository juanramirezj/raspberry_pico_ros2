# Set minimum required version of CMake
cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

# Set name of project (as PROJECT_NAME) and C/C++ standards
project(motor_driver C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(PICO_PATH ${PROJECT_SOURCE_DIR})

# Creates a pico-sdk subdirectory in our project for the libraries
pico_sdk_init()

include(auto_set_url.cmake)

add_subdirectory(motor_driver)
add_subdirectory(motor2_driver)
add_subdirectory(motor3_driver)
add_subdirectory(test_pwm)
add_subdirectory(test_pwm-led)
add_subdirectory(hello_world)
add_subdirectory(motor4_ros)

add_compile_options(-Wall 
    -Wno-format   #int != int32_t as far as the compiler is concerned because gcc has int32_t as long in
    -Wno-unused-function  # we have some for the docs that aren't calle
    -Wno-maybe-uninitialized
)


# Enable usb output, disable uart output
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)
