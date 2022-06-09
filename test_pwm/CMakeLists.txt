add_executable(test_pwm
    main.cpp 
)

target_link_libraries(test_pwm 
   pico_stdlib
   hardware_pwm)

      
pico_add_extra_outputs( test_pwm)
auto_set_url(test_pwm)