# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "/home/dietpi/programming/pico/pico-sdk")

set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

set(HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/include/mpuBase.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/include/mpu6500.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/include/mpu6050.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/include/sdCardWorker.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/include/timer.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/include/gyroLogger.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/include/oneButton.hpp
        )

set(SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp 
        ${CMAKE_CURRENT_SOURCE_DIR}/src/mpuBase.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/mpu6500.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/mpu6050.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/hw_config.cpp 
        ${CMAKE_CURRENT_SOURCE_DIR}/src/sdCardWorker.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/timer.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/gyroLogger.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/oneButton.cpp
        )

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(gyrologger C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(gyrologger ${SOURCES} ${HEADERS})
target_include_directories(gyrologger PRIVATE ${INCLUDE_DIR})
target_sources(gyrologger PRIVATE ${SOURCES} PRIVATE ${HEADERS})

pico_set_program_name(gyrologger "gyrologger")
pico_set_program_version(gyrologger "0.1")

# no_flash means the target is to run from RAM
#pico_set_binary_type(gyrologger no_flash)

pico_enable_stdio_uart(gyrologger 0)
pico_enable_stdio_usb(gyrologger 1)

# Add the standard library to the build
target_link_libraries(gyrologger pico_stdlib)

add_subdirectory(no-OS-FatFS-SD-SPI-RPi-Pico/FatFs_SPI build)

# Add any user requested libraries
target_link_libraries(gyrologger
        hardware_spi
        hardware_i2c
        pico_multicore
        FatFs_SPI
        )

pico_add_extra_outputs(gyrologger)

