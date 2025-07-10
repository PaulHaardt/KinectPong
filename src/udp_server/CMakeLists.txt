
cmake_minimum_required(VERSION 3.14)

# set the project name
project(EPITA_CODEFEST_WEEK)


set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

find_package(libfreenect REQUIRED)
find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc calib3d imgcodecs)
find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets)


# Pseudo target to OpenNI
#add_library(OpenNI INTERFACE)
#target_include_directories(OpenNI INTERFACE thirdparty/OpenNI/Include)
#target_link_libraries(OpenNI INTERFACE ${CMAKE_SOURCE_DIR}/thirdparty/OpenNI/Redist/libOpenNI2.so)


# add the executable
add_executable(glview src/glview.c)
target_link_libraries(glview PRIVATE ${FREENECT_LIBRARIES} GLUT::GLUT OpenGL::GL m)

add_executable(test-cv src/test-cv.cpp)

add_library(opencv_kinect src/capture-cv.hpp src/capture-cv.cpp src/utils.cpp src/calibration-utils.hpp src/calibration-utils.cpp)
target_link_libraries(opencv_kinect PRIVATE opencv_imgproc opencv_calib3d ${FREENECT_LIBRARIES} Qt6::Core)
target_link_libraries(opencv_kinect PUBLIC opencv_core opencv_imgcodecs)

target_link_libraries(test-cv PRIVATE opencv_highgui opencv_kinect)

add_executable(calibration src/calibrate-qt.cpp src/calibrate-qt-main.cpp)
target_link_libraries(calibration PRIVATE Qt6::Gui Qt6::Widgets opencv_kinect)

add_executable(simple_server src/simple_server.cpp)
add_executable(simple_client src/simple_client.cpp)
add_executable(utils src/utils.cpp)

target_link_libraries(simple_server PRIVATE 
    ${FREENECT_LIBRARIES}   # Direct freenect (like glview.c)
    opencv_imgproc          # For detection only
    opencv_core             # For cv::Mat
    Threads::Threads
)
target_link_libraries(simple_client PRIVATE Threads::Threads)

target_include_directories(simple_server PRIVATE src)

# Platform-specific networking libraries
if(WIN32)
    target_link_libraries(simple_server PRIVATE ws2_32)
    target_link_libraries(simple_client PRIVATE ws2_32)
endif()

# Platform-specific networking libraries
if(WIN32)
    target_link_libraries(simple_server PRIVATE ws2_32)
    target_link_libraries(simple_client PRIVATE ws2_32)
endif()
