cmake_minimum_required(VERSION 3.20)

project(stm32g_node_can C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_TOOLCHAIN_FILE)
    message(FATAL_ERROR "Use the provided CMake preset or pass -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake")
endif()

option(STM32G_NODE_USE_STUB_HAL "Build the CAN module against local FDCAN stubs instead of real STM32 HAL headers" ON)

set(STM32CUBE_FW_G4_DIR "$ENV{STM32CUBE_FW_G4_DIR}" CACHE PATH "Path to STM32Cube FW_G4 package root")

if(STM32G_NODE_USE_STUB_HAL)
    add_library(motor_node_can STATIC
        src/motor_node_can.c
    )

    target_include_directories(motor_node_can
        PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}/include
            ${CMAKE_CURRENT_SOURCE_DIR}/stubs
    )

    target_compile_definitions(motor_node_can
        PUBLIC
            STM32G431xx
            STM32G_NODE_USE_STUB_HAL
    )
else()
    if(NOT STM32CUBE_FW_G4_DIR)
        message(FATAL_ERROR "STM32CUBE_FW_G4_DIR is not set. Point it to the STM32CubeG4 firmware package root.")
    endif()

    set(STM32_HAL_INC_DIR "${STM32CUBE_FW_G4_DIR}/Drivers/STM32G4xx_HAL_Driver/Inc")
    set(STM32_HAL_LEGACY_INC_DIR "${STM32CUBE_FW_G4_DIR}/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy")
    set(STM32_CMSIS_DEVICE_INC_DIR "${STM32CUBE_FW_G4_DIR}/Drivers/CMSIS/Device/ST/STM32G4xx/Include")
    set(STM32_CMSIS_CORE_INC_DIR "${STM32CUBE_FW_G4_DIR}/Drivers/CMSIS/Include")

    foreach(required_dir
            "${STM32_HAL_INC_DIR}"
            "${STM32_CMSIS_DEVICE_INC_DIR}"
            "${STM32_CMSIS_CORE_INC_DIR}")
        if(NOT EXISTS "${required_dir}")
            message(FATAL_ERROR "Required STM32 include directory not found: ${required_dir}")
        endif()
    endforeach()

    add_library(motor_node_can STATIC
        src/motor_node_can.c
    )

    target_include_directories(motor_node_can
        PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}/include
            ${STM32_HAL_INC_DIR}
            ${STM32_HAL_LEGACY_INC_DIR}
            ${STM32_CMSIS_DEVICE_INC_DIR}
            ${STM32_CMSIS_CORE_INC_DIR}
    )

    target_compile_definitions(motor_node_can
        PUBLIC
            STM32G431xx
            USE_HAL_DRIVER
    )
endif()

target_compile_options(motor_node_can
    PRIVATE
        -mcpu=cortex-m4
        -mthumb
        -mfpu=fpv4-sp-d16
        -mfloat-abi=hard
        -ffunction-sections
        -fdata-sections
        -Wall
        -Wextra
        -Wshadow
)
