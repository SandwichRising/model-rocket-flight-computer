# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(Rocket_Computer C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(Rocket_Computer 
        Rocket_Computer.c
        hw_config.c
 )

# Tell CMake where to find other source code
add_subdirectory(lib/no-OS-FatFS-SD-SDIO-SPI-RPi-Pico/src build)

pico_set_program_name(Rocket_Computer "Rocket_Computer")
pico_set_program_version(Rocket_Computer "1.0")

# Generate PIO header
pico_generate_pio_header(Rocket_Computer ${CMAKE_CURRENT_LIST_DIR}/blink.pio)
pico_generate_pio_header(Rocket_Computer ${CMAKE_CURRENT_LIST_DIR}/servo.pio)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(Rocket_Computer 0)
pico_enable_stdio_usb(Rocket_Computer 1)

# Add the standard library to the build
target_link_libraries(Rocket_Computer
        pico_stdlib
        no-OS-FatFS-SD-SDIO-SPI-RPi-Pico
)

# Add the standard include files to the build
target_include_directories(Rocket_Computer PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(Rocket_Computer 
        hardware_i2c
        hardware_pio
        hardware_dma
        hardware_spi
        
        )

pico_add_extra_outputs(Rocket_Computer)

