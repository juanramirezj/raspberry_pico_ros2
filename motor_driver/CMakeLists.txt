add_executable(motor_driver
    main.cpp 
)

target_link_libraries(motor_driver 
   pico_stdlib
   hardware_pwm)
   
pico_add_extra_outputs( motor_driver)
auto_set_url(motor_driver)
