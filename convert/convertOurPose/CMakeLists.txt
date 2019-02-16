cmake_minimum_required(VERSION 2.8)
project(convert)
set( CMAKE_CXX_FLAGS "-std=c++11" )


find_package( OpenCV 3 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )


# eigen 
include_directories( "/usr/include/eigen3/" )

# pcl 
find_package( PCL 1.7 )
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

find_package(OpenMP)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")


add_executable(convert_pose convert_pose.cpp)
target_link_libraries(convert_pose ${OpenCV_LIBS} ${PCL_LIBRARIES} )

add_executable(getgt getgt.cpp)
target_link_libraries(getgt ${OpenCV_LIBS})

