cmake_minimum_required(VERSION 2.8.3)
project(cuda_example)
FIND_PACKAGE(CUDA REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
)

SET(CUDA_NVCC_FLAGS "-arch=sm_30" CACHE STRING "nvcc flags" FORCE)
SET(CUDA_VERBOSE_BUILD ON CACHE BOOL "nvcc verbose" FORCE)
SET(LIB_TYPE STATIC)

catkin_package(
   CATKIN_DEPENDS std_msgs sensor_msgs roscpp
)

include_directories(
	${CATKIN_INCLUDE_DIRS}
)

# For hello world program
CUDA_ADD_LIBRARY(Testlib ${LIB_TYPE} src/hello_world/helloworld.cu)

ADD_EXECUTABLE(cuda_example_node src/hello_world/main.cpp)
ADD_DEPENDENCIES(cuda_example_node Testlib)
TARGET_LINK_LIBRARIES(cuda_example_node
	${catkin_LIBRARIES}
	Testlib
)

#For simple addition program
CUDA_ADD_LIBRARY(SimpleAddlib ${LIB_TYPE} src/simple_addition/simple_addition.cu)

ADD_EXECUTABLE(simple_addition_node src/simple_addition/main.cpp)
ADD_DEPENDENCIES(simple_addition_node SimpleAddlib)
TARGET_LINK_LIBRARIES(simple_addition_node
	${catkin_LIBRARIES}
	SimpleAddlib
)

#For vector_addition program
CUDA_ADD_LIBRARY(VectorAddlib ${LIB_TYPE} src/vector_addition/vector_addition.cu)

ADD_EXECUTABLE(vector_addition_node src/vector_addition/main.cpp)
ADD_DEPENDENCIES(vector_addition_node VectorAddlib)
TARGET_LINK_LIBRARIES(vector_addition_node
	${catkin_LIBRARIES}
	VectorAddlib
)

#For performance evaluation program 
CUDA_ADD_LIBRARY(Comparelib ${LIB_TYPE} src/compare/compare.cu)

ADD_EXECUTABLE(compare_node src/compare/main.cpp)
ADD_DEPENDENCIES(compare_node Comparelib)
TARGET_LINK_LIBRARIES(compare_node
	${catkin_LIBRARIES}
	Comparelib
)

# CUDA Architecture examples ----------------------------------------------------------

#For kernel example 
CUDA_ADD_LIBRARY(Kernelslib ${LIB_TYPE} src/kernels/kernels.cu)

ADD_EXECUTABLE(kernels_node src/kernels/main.cpp)
ADD_DEPENDENCIES(kernels_node Kernelslib)
TARGET_LINK_LIBRARIES(kernels_node
	${catkin_LIBRARIES}
	Kernelslib
)



