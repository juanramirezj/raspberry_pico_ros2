add_executable(motor4_ros
    main.cpp 
)

target_link_libraries(motor4_ros 
   pico_stdlib
   hardware_pwm)

      
pico_add_extra_outputs( motor4_ros)
auto_set_url(motor4_ros)

pico_enable_stdio_usb(motor4_ros 1)
pico_enable_stdio_uart(motor4_ros 0)
