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
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

set(PICO_SDK_PATH ${CMAKE_CURRENT_LIST_DIR}/../pico-sdk)
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(pico_quiros C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# --- Generar gatt_db.h desde uart.gatt ---
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(UART_GATT_FILE "${CMAKE_CURRENT_LIST_DIR}/uart.gatt")

add_custom_command(
    OUTPUT ${CMAKE_BINARY_DIR}/generated/gatt_db.h
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/generated
    COMMAND ${Python3_EXECUTABLE} ${PICO_SDK_PATH}/lib/btstack/tool/compile_gatt.py
            "${UART_GATT_FILE}"
            ${CMAKE_BINARY_DIR}/generated/gatt_db.h
    DEPENDS "${UART_GATT_FILE}"
    VERBATIM
)
add_custom_target(gen_gatt DEPENDS ${CMAKE_BINARY_DIR}/generated/gatt_db.h)

# Agrega todos los archivos fuente del proyecto
add_executable(pico_quiros
    src/main.c
    src/ble_connection.c
    src/calibration.c
    src/frame_building.c
    src/get_sensors.c
)

add_dependencies(pico_quiros gen_gatt)

pico_set_program_name(pico_quiros "pico_quiros")
pico_set_program_version(pico_quiros "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(pico_quiros 0)
pico_enable_stdio_usb(pico_quiros 1)

# Optimization level
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O2")

# Add the standard library to the build
target_link_libraries(pico_quiros
        pico_stdlib
        hardware_i2c
        hardware_uart
        hardware_flash
        pico_multicore
        # --- BLE ---
        pico_btstack_ble
        pico_btstack_cyw43
        pico_cyw43_arch_none
)

# Incluye los directorios de cabeceras
target_include_directories(pico_quiros PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/include
        ${CMAKE_BINARY_DIR}/generated
)

pico_add_extra_outputs(pico_quiros)

