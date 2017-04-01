
cmake_minimum_required(VERSION 2.8)

project( LoadImage )

find_package( OpenCV REQUIRED )
set( OPENNI2_INCLUDE /home/ubuntu/Downloads/2-Linux/OpenNI-Linux-Arm-2.3/Include)
set( OPENNI2_REDIST  /home/ubuntu/Downloads/2-Linux/OpenNI-Linux-Arm-2.3/Redist)
include_directories( ${OPENNI2_INCLUDE} 
                      ${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/GL 
                    )

link_directories ( ${OPENNI2_REDIST} )

set( ex1 test-openni-opencv)

set( source_files1
    main_origin.cpp
)

add_executable( ${ex1} ${source_files1} )

target_link_libraries( ${ex1} ${OpenCV_LIBS} OpenNI2)
