cmake_minimum_required(VERSION 3.24 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(cloud_viewer)

set(cloud_viewer_src
    src/main.cpp
    src/pointcloud_processing.cpp
    src/sequence_viewer.cpp)

set(Qt5_DIR /usr/local/Cellar/qt@5/5.15.5_1/lib/cmake/Qt5)
# set(Boost_DIR /usr/local/Cellar/boost/1.79.0_1/lib/cmake)
# set(Boost_INCLUDE_DIRS /usr/local/Cellar/boost/1.79.0_1/include)

if(EXISTS /usr/local/lib/cmake/Boost-1.79.0/BoostConfig.cmake)
    set(Boost_NO_BOOST_CMAKE ON)
    set(Boost_USE_STATIC_LIBS=OFF) 
    set(Boost_USE_MULTITHREADED=OFF) 
    set(Boost_USE_STATIC_RUNTIME=OFF)
endif()
find_package(PCL 1.2 REQUIRED)
# find_package(PCL)

if(Boost_FOUND)   
MESSAGE("Success!") 
find_package(Boost 1.79.0 COMPONENTS system program_options thread REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
elseif(NOT Boost_FOUND)
message(FATAL_ERROR "Unable to find correct Boost version. Did you set BOOST_ROOT?") 
endif()

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(include)
add_executable (cloud_viewer ${cloud_viewer_src})
target_link_libraries (cloud_viewer ${PCL_LIBRARIES} ${Boost_LIBRARIES})
