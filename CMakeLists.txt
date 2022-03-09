# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "/home/dietpi/programming/pico/pico-sdk")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(gyrologger C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(gyrologger gyrologger.cpp )

pico_set_program_name(gyrologger "gyrologger")
pico_set_program_version(gyrologger "0.1")

# no_flash means the target is to run from RAM
pico_set_binary_type(gyrologger no_flash)

pico_enable_stdio_uart(gyrologger 0)
pico_enable_stdio_usb(gyrologger 1)

# Add the standard library to the build
target_link_libraries(gyrologger pico_stdlib)

# Add any user requested libraries
target_link_libraries(gyrologger
        hardware_spi
        pico_multicore
        )

pico_add_extra_outputs(gyrologger)

