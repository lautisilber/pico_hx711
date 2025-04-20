add_executable(read_sync_example)

target_sources(read_sync_example PRIVATE read_sync_example.c)

target_link_libraries(read_sync_example PRIVATE pico_stdlib PicoHX711)

# enable usb output, disable uart output
pico_enable_stdio_usb(read_sync_example 1)
pico_enable_stdio_uart(read_sync_example 0)
pico_add_extra_outputs(read_sync_example)