# writed by zhang ning 2018/3/22
cmake_minimum_required( VERSION 2.8 )

project(trajectory_compare)

set( CMAKE_BUILD_TYPE "Debug" )

set( CMAKE_CXX_FLAGS "-std=c++11 -O3" ) 



find_package( Sophus REQUIRED)
find_package( Pangolin REQUIRED)


include_directories( "/usr/include/eigen3" )
include_directories( ${Sophus_INCLUDE_DIRS} )
include_directories( ${Pangolin_INCLUDE_DIRS} )

add_executable( trajectory_compare trajectory_compare.cpp)

target_link_libraries( trajectory_compare ${Sophus_LIBRARIES} ${Pangolin_LIBRARIES} )




