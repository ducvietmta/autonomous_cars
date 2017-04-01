
cmake_minimum_required(VERSION 2.8)

project( api_uart )

include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )

set( source_files
rs232.c
rs232.h
api_uart.cpp
api_uart.h
)

set (libname uart )

add_library( ${libname} ${source_files})

set( exname test-uart )

add_executable( ${exname} test_uart.cpp)

target_link_libraries( ${exname} ${libname})

