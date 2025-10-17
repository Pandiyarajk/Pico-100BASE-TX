cmake_minimum_required(VERSION 3.12)
include(pico_sdk_import.cmake)

project(pico100basetx C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_compile_options(-Wall)

include_directories(
	include
	)

add_subdirectory(libpico100basetx)
add_subdirectory(apps)
