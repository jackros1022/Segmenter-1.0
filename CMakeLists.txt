cmake_minimum_required ( VERSION 2.6 )
set(PCL_DIR "/home/jack/ros/kinect2/pcl-pcl-1.8.0/build")
PROJECT(v4r)

##########################################
## Do not waste the top level cmake !!! ##
##########################################
include(opencv.cmake)   # OPENCV_LIBRARIES 
include(v4rutils.cmake) # V4R utils

SET(LIBRARY_OUTPUT_PATH "${CMAKE_CURRENT_SOURCE_DIR}/lib")
SET(EXECUTABLE_OUTPUT_PATH "${CMAKE_CURRENT_SOURCE_DIR}/bin")

set(V4R_BUILD 1)

#add_definitions(-DDEBUG)
add_definitions(-DLINUX)
add_definitions("-Wall -Wl,--no-undefined -fno-strict-aliasing -pthread")

include_directories(
    /home/jack/learning_projects/pcl/Segmenter-1.0/surface/include
    /home/jack/learning_projects/pcl/Segmenter-1.0
)
link_directories(
     /home/jack/learning_projects/pcl/Segmenter-1.0/lib       
)
add_subdirectory(v4r)
add_subdirectory(apps)
add_subdirectory(v4rexternal)

#install target
configure_file(v4r.pc.in "${PROJECT_BINARY_DIR}/v4r.pc" @ONLY)
INSTALL(FILES ${PROJECT_BINARY_DIR}/v4r.pc DESTINATION lib/pkgconfig)

# uninstall target
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)
add_custom_target(uninstall COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
