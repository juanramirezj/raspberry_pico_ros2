add_executable(motor3_driver
    main.cpp 
)

target_link_libraries(motor3_driver 
   pico_stdlib
   hardware_pwm)

      
pico_add_extra_outputs( motor3_driver)
auto_set_url(motor3_driver)

pico_enable_stdio_usb(motor3_driver 1)
pico_enable_stdio_uart(motor3_driver 0)
