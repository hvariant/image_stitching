cmake_minimum_required(VERSION 2.8)

project( Theseus )

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( Theseus ./steps.cpp ./param.cpp ./theseus.cpp ./images.cpp ./features.cpp)
target_link_libraries( Theseus ${OpenCV_LIBS} )
