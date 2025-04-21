add_library(PicoHX711 STATIC
    PicoHX711.c
    PicoHX711Async.c
)

add_subdirectory(lib)

target_include_directories(PicoHX711 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(PicoHX711 PRIVATE
    pico_stdlib
    pico_sync
    pico_time
)

target_link_libraries(PicoHX711 PUBLIC
    pico_hx711_welfords
)