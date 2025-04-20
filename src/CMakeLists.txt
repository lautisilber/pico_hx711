add_library(PicoHX711 STATIC
    PicoHX711.c
)

target_include_directories(PicoHX711 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(PicoHX711 PRIVATE
    pico_stdlib
    pico_sync
)