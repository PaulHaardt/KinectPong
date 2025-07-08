cmake_minimum_required(VERSION 4.0)
project(MyOpenCVApp)

find_package(OpenCV REQUIRED)

add_executable(my_app main.cpp)
target_link_libraries(my_app ${OpenCV_LIBS})
