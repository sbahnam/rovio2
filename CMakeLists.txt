# cmake_minimum_required (VERSION 3.22.1)
cmake_minimum_required (VERSION 3.18.4) # debian on the pi only has binaries for this version, not 3.22.1. Fingers crossed

project(rovio
  VERSION 1.0
  LANGUAGES CXX)

set(CMAKE_VERBOSE_MAKEFILE ON)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17" ) # not sure if that's good. but was needed for proper filesystem handling in rovio_node.cpp
#set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
add_compile_options(-O3 -flto -funroll-loops)
add_link_options(-O3 -flto)
set(ROVIO_NMAXFEATURE 25 CACHE STRING "Number of features for ROVIO")
set(ROVIO_NCAM 1 CACHE STRING "Number of enabled cameras")
set(ROVIO_NLEVELS 4 CACHE STRING "Number of image leavels for the features")
set(ROVIO_PATCHSIZE 6 CACHE STRING "Size of patch (edge length in pixel)")
set(ROVIO_NPOSE 0 CACHE STRING "Additional estimated poses for external pose measurements")
add_definitions(-DROVIO_NMAXFEATURE=${ROVIO_NMAXFEATURE})
add_definitions(-DROVIO_NCAM=${ROVIO_NCAM})
add_definitions(-DROVIO_NLEVELS=${ROVIO_NLEVELS})
add_definitions(-DROVIO_PATCHSIZE=${ROVIO_PATCHSIZE})
add_definitions(-DROVIO_NPOSE=${ROVIO_NPOSE})

# add_subdirectory(lightweight_filtering)

# ##################### Find, include, and compile library #####################
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
if(0)
	find_package(OpenMP REQUIRED)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc features2d)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories("ext/kindr/include")

find_package(PkgConfig)

find_package(yaml_cpp_catkin QUIET)
if(${yaml_cpp_catkin_FOUND})
  message(STATUS "Found yaml_cpp_catkin, using instead of system library.")
  set(YamlCpp_LIBRARIES ${yaml_cpp_catkin_LIBRARIES})
  set(YamlCpp_INCLUDE_DIRS ${yaml_cpp_catkin_INCLUDE_DIRS})
else()
  message(STATUS "No yaml_cpp_catkin, using yaml-cpp system library instead.")
  pkg_check_modules(YamlCpp REQUIRED yaml-cpp>=0.5)
endif()




# add_library(${PROJECT_NAME} src/Camera.cpp src/FeatureCoordinates.cpp src/FeatureDistance.cpp)

add_library(${PROJECT_NAME} src/Camera.cpp src/FeatureCoordinates.cpp src/FeatureDistance.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE ${PROJECT_SOURCE_DIR}/include
												${PROJECT_SOURCE_DIR}/lightweight_filtering/include)


add_executable(rovio_node src/rovio_node.cpp)

target_include_directories(rovio_node PRIVATE ${PROJECT_SOURCE_DIR}/include
											${PROJECT_SOURCE_DIR}/lightweight_filtering/include)
				

target_link_libraries(rovio_node ${OpenCV_LIBS}
								${Eigen3_LIBS}
								${PROJECT_NAME}
								${YamlCpp_LIBRARIES})

add_custom_command(
        TARGET rovio_node POST_BUILD
        COMMAND cp -R ${CMAKE_SOURCE_DIR}/cfg ${CMAKE_CURRENT_BINARY_DIR}/cfg)

add_custom_command(
        TARGET rovio_node POST_BUILD
        COMMAND cp -R ${CMAKE_SOURCE_DIR}/example ${CMAKE_CURRENT_BINARY_DIR}/example)
