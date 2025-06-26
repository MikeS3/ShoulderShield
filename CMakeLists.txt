cmake_minimum_required(VERSION 3.13)

if (DEFINED ENV{PICO_SDK_PATH})
	set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
else ()
	message( FATAL_ERROR "PICO_SDK_PATH not defined" )
endif()

# Include the SDK (either from src directory or root)
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Project name
project(Shoulder_Shield C CXX ASM)

# Set C and C++ standards
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

# Add include directories
# include_directories(include)
include_directories(src)

# Add the source directory
add_subdirectory(src)
