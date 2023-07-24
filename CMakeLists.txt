cmake_minimum_required(VERSION 3.12)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(PICO_SDK_PATH "../../pico-sdk")
set(PICO_BOARD pico_w)
include(pico_sdk_import.cmake)
project(carCtrl)

pico_sdk_init()

include_directories(${PROJECT_SOURCE_DIR})

# generate an executable file
add_executable(${PROJECT_NAME}
    src/main.cpp
)

pico_set_program_name(${PROJECT_NAME} "Pico Car Control")
pico_set_program_version(${PROJECT_NAME} "0.1")

# enable usb output, disable uart output
pico_enable_stdio_usb(${PROJECT_NAME} 0)
pico_enable_stdio_uart(${PROJECT_NAME} 1)

pico_add_extra_outputs(${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME} pico_stdlib hardware_spi hardware_i2c hardware_pwm pico_btstack_classic pico_btstack_cyw43 pico_cyw43_arch_none)
