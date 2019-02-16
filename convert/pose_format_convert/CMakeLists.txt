cmake_minimum_required(VERSION 2.8)

project(convert)

find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

set(CMAKE_BUILD_TYPE "Release") 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")


include_directories(
	${EIGEN3_INCLUDE_DIRS}
	${CERES_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
	convert.cpp
)
