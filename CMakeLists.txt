cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME stm32Controller)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM, C, and C++
enable_language(C ASM CXX)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    app/app_main.c
    common/crc32.c
    common/framing_cobs.c
    common/mux_channels.c
    common/shared_protocol/robot_protocol.c
    utils/log.c
    control/MotionController.cpp
    control/StateEstimator.cpp
    control/ekf/BalancerEKF.cpp
    control/motion_modes.cpp
    control/motion_control.cpp
    control/control_timer.c
    drivers/imu/imu_bus.c
    drivers/imu/imu_bmi270.c
    drivers/imu/imu_bmm150.c
    drivers/imu/imu_icm42688.c
    drivers/imu/imu_exti.c
    drivers/imu/imu_bus_sched.c
    drivers/imu/imu_sched.c
    drivers/motors/motor_link.c
    drivers/motors/motor_link_framing.c
    drivers/param_storage.c
    drivers/system_reboot.c
    drivers/spi_bus.c
    drivers/debug_wdog.c
    drivers/led_status.c
    libs/BMI270_SensorAPI/bmi2.c
    libs/BMI270_SensorAPI/bmi270.c
    libs/BMM150_SensorAPI/bmm150.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    app
    common
    common/shared_protocol
    control
    control/ekf
    drivers
    drivers/imu
    drivers/motors
    utils
    libs/BMI270_SensorAPI
    libs/BMM150_SensorAPI
    libs/TinyEKF
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# Emit a firmware.map alias for build analyzers
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.map ${CMAKE_CURRENT_BINARY_DIR}/firmware.map
    COMMENT "Copying ${CMAKE_PROJECT_NAME}.map to firmware.map")

# Generate raw binary for bootloader consumption (offset already handled in linker script)
find_program(OBJCOPY_BIN arm-none-eabi-objcopy)
if(OBJCOPY_BIN)
    add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
        COMMAND ${OBJCOPY_BIN} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin
        COMMENT "Generating ${CMAKE_PROJECT_NAME}.bin")
else()
    message(WARNING "objcopy not found; binary image will not be generated")
endif()
