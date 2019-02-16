cmake_minimum_required( VERSION 2.8 )
project(convert1)

#set( CMAKE_BUILD_TYPE Debug )
set( CMAKE_CXX_FLAGS "-std=c++11" )

# opencv 
find_package( OpenCV 3 REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

# eigen 
include_directories( "/usr/include/eigen3/" )

#boost
find_package(Boost REQUIRED COMPONENTS filesystem system)
include_directories(${Boost_INCLUDE_DIRS})

# pcl 
#find_package( PCL 1.8 REQUIRED COMPONENT common io filters HINTS /home/zuojiaxing/local)
#include_directories( ${PCL_INCLUDE_DIRS} )
#add_definitions( ${PCL_DEFINITIONS} )

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

add_executable( convert1 convert1.cpp )
target_link_libraries( convert1 ${OpenCV_LIBS} ${Boost_LIBRARIES})
#target_link_libraries( convert1 ${OpenCV_LIBS} ${PCL_LIBRARIES} )


add_executable( convert2 convert2.cpp )
target_link_libraries( convert2 ${OpenCV_LIBS} ${Boost_LIBRARIES})
#target_link_libraries( convert1 ${OpenCV_LIBS} ${PCL_LIBRARIES} )

# Set optimized building:
if(MSVC_IDE)
  add_definitions(-D_CRT_SECURE_NO_WARNINGS)
else(MSVC_IDE)
  if( CMAKE_BUILD_TYPE MATCHES "Debug" )
      #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -fpic -Wextra -pedantic -std=c++11 -pthread -g")
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse3 -std=c++11 -pthread -g")
  else( CMAKE_BUILD_TYPE MATCHES "Release" )
      #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -fpic -Wextra -pedantic -std=c++11 -pthread -O3 -march=native")
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse3 -std=c++11 -pthread -O3 -march=native")
  endif( CMAKE_BUILD_TYPE MATCHES "Debug" )
endif(MSVC_IDE)


message(====================================================)
message(${CMAKE_CXX_FLAGS})
message(====================================================)
