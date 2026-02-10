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

# Collect all .c and .cpp files from app/ directory (including subdirectories)
file(GLOB_RECURSE APP_C_SOURCES "app/*.c")
file(GLOB_RECURSE APP_CPP_SOURCES "app/*.cpp")
set(APP_SOURCES ${APP_C_SOURCES} ${APP_CPP_SOURCES})

# Ensure newly added control sources are always captured by CMake.
list(APPEND APP_SOURCES app/control/hip_behavior.c)
list(APPEND APP_SOURCES app/control/lqr_lut.c)

# Collect all .cpp files from control/ directory (including subdirectories)
file(GLOB_RECURSE COMMON_CPP_SOURCES "common/*.c")

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${APP_SOURCES}
    ${COMMON_CPP_SOURCES}
    libs/BMI270_SensorAPI/bmi2.c
    libs/BMI270_SensorAPI/bmi270.c
    libs/BMM150_SensorAPI/bmm150.c
)

# Collect all subdirectories of app/ that contain headers
file(GLOB_RECURSE APP_HEADERS "app/*.h")
set(APP_INCLUDE_DIRS "")
foreach(HEADER ${APP_HEADERS})
    get_filename_component(DIR ${HEADER} DIRECTORY)
    list(APPEND APP_INCLUDE_DIRS ${DIR})
endforeach()
list(REMOVE_DUPLICATES APP_INCLUDE_DIRS)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${APP_INCLUDE_DIRS}
    common
    common/shared_protocol
    libs/BMI270_SensorAPI
    libs/BMM150_SensorAPI
    libs/TinyEKF
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)
if(DEFINED APP_LOG_LEVEL)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE APP_LOG_LEVEL=${APP_LOG_LEVEL})
endif()

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
