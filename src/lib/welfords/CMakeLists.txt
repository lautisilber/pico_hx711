set(LIB_NAME pico_hx711_welfords)

add_library(pico_hx711_welfords STATIC
    welfords.c
)

target_include_directories(pico_hx711_welfords PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(pico_hx711_welfords PRIVATE
    pico_stdlib
)