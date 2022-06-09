add_executable(motor2_driver
    main.cpp 
)

target_link_libraries(motor2_driver 
   pico_stdlib
   hardware_pwm)

      
pico_add_extra_outputs( motor2_driver)
auto_set_url(motor2_driver)

pico_enable_stdio_usb(motor2_driver 1)
pico_enable_stdio_uart(motor2_driver 0)
