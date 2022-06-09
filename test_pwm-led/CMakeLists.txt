add_executable(test_pwm-led
    main.cpp 
)

target_link_libraries(test_pwm-led 
   pico_stdlib
   hardware_pwm)

pico_add_extra_outputs( test_pwm-led)
auto_set_url(test_pwm-led)