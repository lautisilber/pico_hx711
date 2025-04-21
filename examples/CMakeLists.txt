
foreach(PN read_sync_simple read_sync_stats calib_example manual_calib)
    add_executable(${PN})

    target_sources(${PN} PRIVATE "${PN}.c")

    target_link_libraries(${PN} PRIVATE pico_stdlib PicoHX711)

    # enable usb output, disable uart output
    pico_enable_stdio_usb(${PN} 1)
    pico_enable_stdio_uart(${PN} 0)
    pico_add_extra_outputs(${PN})

    endforeach()
    target_link_libraries(calib_example PRIVATE hardware_watchdog)
