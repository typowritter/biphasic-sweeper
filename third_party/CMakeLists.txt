cmake_minimum_required(VERSION 3.15)
set(CMAKE_TOOLCHAIN_FILE toolchain.cmake)

project(third_party C)
file(GLOB_RECURSE SOURCES lvgl/src/*.c)
add_library(lvgl STATIC ${SOURCES})
