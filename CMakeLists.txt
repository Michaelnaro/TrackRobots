cmake_minimum_required(VERSION 2.8)
project( hello_world )
find_package( OpenCV REQUIRED )


file(GLOB OPEN_CV_FILES . "*.cpp")
add_definitions(-std=c++11)
add_executable(hello ${OPEN_CV_FILES} )
target_link_libraries( hello ${OpenCV_LIBS} )
