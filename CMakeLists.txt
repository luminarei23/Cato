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
set(sdkVersion 2.0.0)
set(toolchainVersion 13_2_Rel1)
set(picotoolVersion 2.0.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(cat_lcd C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1
add_executable(cat_lcd src/cat_lcd.c)

# Generate the PIO header from the st7789_lcd.pio file
pico_generate_pio_header(cat_lcd ${CMAKE_CURRENT_LIST_DIR}/src/st7789_lcd.pio)

# Set program name and version
pico_set_program_name(cat_lcd "cat_lcd")
pico_set_program_version(cat_lcd "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(cat_lcd 0)
pico_enable_stdio_usb(cat_lcd 1)

# Add the standard library to the build
target_link_libraries(cat_lcd
        pico_stdlib
)

# Add the standard include files to the build
target_include_directories(cat_lcd PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/src
)

# Add any user requested libraries
target_link_libraries(cat_lcd 
        hardware_spi
        hardware_i2c
        pico_cyw43_arch_lwip_threadsafe_background
        hardware_pio
        hardware_interp
)

# Add extra outputs (e.g., .bin, .uf2)
pico_add_extra_outputs(cat_lcd)
