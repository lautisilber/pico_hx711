add_library(PicoHX711 STATIC
    PicoHX711.c
)

add_subdirectory(lib)

target_include_directories(PicoHX711 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(PicoHX711 PRIVATE
    pico_stdlib
    pico_sync
    pico_time

    pico_hx711_welfords
)