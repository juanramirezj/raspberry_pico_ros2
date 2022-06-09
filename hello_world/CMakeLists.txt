add_executable(hello_world
    hello_world.cpp
    )

target_link_libraries(hello_world pico_stdlib)

pico_add_extra_outputs(hello_world)

auto_set_url(hello_world)

pico_enable_stdio_usb(hello_world 1)
pico_enable_stdio_uart(hello_world 0)