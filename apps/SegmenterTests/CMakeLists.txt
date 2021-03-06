cmake_minimum_required (VERSION 2.6)
SET(APPS_SEGMENTER_TESTS 1 CACHE BOOL "Test apps for segmenter framework" )
IF(APPS_SEGMENTER_TESTS)

IF(V4R_GRAPH_CUT)
IF(V4R_SVM)
IF(V4R_SURFACE_RELATIONS)
IF(V4R_SURFACE_MODELING)
IF(V4R_SURFACE_CLUSTERING)
IF(V4R_SURFACE_UTILS)
IF(V4R_KINECT_INTERFACE)
IF(V4R_TOMGINE)
IF(V4R_PCL_ADD_ONS)

include_directories(.)
include_directories(${OPENCV_INCLUDE_DIRS})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

SET(BINARY_NAME presegmenter)
add_executable(${BINARY_NAME} PreSegmenter.cpp)
target_link_libraries (${BINARY_NAME} ${PCL_LIBRARIES} ${OPENCV_LIBRARIES})
target_link_libraries (${BINARY_NAME} v4rKinectInterface v4rSurfaceClustering v4rSurfaceUtils)
target_link_libraries (${BINARY_NAME} v4rTomGinePCL)
v4r_add_binary(${BINARY_NAME})

SET(BINARY_NAME modelAbstractor)
add_executable(${BINARY_NAME} ModelAbstractor.cpp)
target_link_libraries (${BINARY_NAME} ${PCL_LIBRARIES} ${OPENCV_LIBRARIES})
target_link_libraries (${BINARY_NAME} v4rKinectInterface v4rSurfaceClustering v4rSurfaceModeling v4rSurfaceUtils)
v4r_add_binary(${BINARY_NAME})

SET(BINARY_NAME modelRefiner)
add_executable(${BINARY_NAME} ModelRefinement.cpp)
target_link_libraries (${BINARY_NAME} ${PCL_LIBRARIES} ${OPENCV_LIBRARIES})
target_link_libraries (${BINARY_NAME} v4rKinectInterface v4rSurfaceClustering v4rSurfaceModeling v4rObjectModeling v4rSurfaceUtils)
v4r_add_binary(${BINARY_NAME})

SET(BINARY_NAME segmenter)
add_executable(${BINARY_NAME} Segmenter.cpp)
target_link_libraries (${BINARY_NAME} ${PCL_LIBRARIES} ${OPENCV_LIBRARIES})
target_link_libraries (${BINARY_NAME} v4rKinectInterface v4rSurfaceModeling v4rSurfaceClustering v4rObjectModeling v4rSurfaceRelations v4rsvm v4rGraphCut v4rSurfaceUtils)
v4r_add_binary(${BINARY_NAME})


ELSE(V4R_PCL_ADD_ONS)
  MESSAGE(STATUS "********************************************\n   Apps: Segmenter warning: missing V4R_PCL_ADD_ONS\n   ********************************************\n")
ENDIF(V4R_PCL_ADD_ONS)

ELSE(V4R_TOMGINE)
  MESSAGE(STATUS "********************************************\n   Apps: Segmenter warning: missing V4R_TOMGINE\n   ********************************************\n")
ENDIF(V4R_TOMGINE)

ELSE(V4R_KINECT_INTERFACE)
  MESSAGE(STATUS "********************************************\n   Apps: Segmenter warning: missing V4R_KINECT_INTERFACE\n   *********************************************\n")
ENDIF(V4R_KINECT_INTERFACE)

ELSE(V4R_SURFACE_UTILS)
  MESSAGE(STATUS "********************************************\n   Apps: Segmenter warning: missing V4R_SURFACE_UTILS\n   ********************************************\n")
ENDIF(V4R_SURFACE_UTILS)

ELSE(V4R_SURFACE_CLUSTERING)
  MESSAGE(STATUS "********************************************\n   Apps: Segmenter warning: missing V4R_SURFACE_CLUSTERING\n   ********************************************\n")
ENDIF(V4R_SURFACE_CLUSTERING)

ELSE(V4R_SURFACE_MODELING)
  MESSAGE(STATUS "********************************************\n   Apps: Segmenter warning: missing V4R_SURFACE_MODELING\n   ********************************************\n")
ENDIF(V4R_SURFACE_MODELING)

ELSE(V4R_SURFACE_RELATIONS)
  MESSAGE(STATUS "********************************************\n   Apps: Segmenter warning: missing V4R_SURFACE_RELATIONS\n   ********************************************\n")
ENDIF(V4R_SURFACE_RELATIONS)

ELSE(V4R_SVM)
  MESSAGE(STATUS "********************************************\n   Apps: Segmenter warning: missing V4R_SVM\n   ********************************************\n")
ENDIF(V4R_SVM)

ELSE(V4R_GRAPH_CUT)
  MESSAGE(STATUS "********************************************\n   Apps: Segmenter warning: missing V4R_GRAPH_CUT\n   ********************************************\n")
ENDIF(V4R_GRAPH_CUT)

ENDIF(APPS_SEGMENTER_TESTS)
