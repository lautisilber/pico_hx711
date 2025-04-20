cmake_minimum_required(VERSION 3.13)

set(PROJECT_NAME PicoHX711)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    # Standalone build
    include(pico_sdk_import.cmake)
    # include(pico-sdk/pico_sdk_init.cmake)
    project(${PROJECT_NAME} C CXX ASM)
    pico_sdk_init()
    set(PICO_PIO_PWM_BUILD_EXAMPLES ON)
else()
    # Used as a subproject / library
    set(PICO_PIO_PWM_BUILD_EXAMPLES OFF)
endif()

# Add the actual library
add_subdirectory(src)

# Optionally add examples
if(PICO_PIO_PWM_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

