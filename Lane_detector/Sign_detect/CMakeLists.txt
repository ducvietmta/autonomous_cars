
cmake_minimum_required(VERSION 2.8)

project( sign_detect )

include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )

set( source_files
api_stop_sign_detection.cpp
api_stop_sign_detection.h
)

set (libname sign_detect )

add_library( ${libname} ${source_files})

