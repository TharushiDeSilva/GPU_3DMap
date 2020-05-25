cmake_minimum_required(VERSION 2.8.3)
project(map_builder)
FIND_PACKAGE(CUDA REQUIRED 
#PCL REQUIRED COMPONENTS common io
)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  roscpp
  sensor_msgs
  geometry_msgs
  octomap_msgs
  octomap_ros
  pcl_ros
)

SET(CUDA_NVCC_FLAGS "-arch=sm_30" CACHE STRING "nvcc flags" FORCE)
SET(CUDA_VERBOSE_BUILD ON CACHE BOOL "nvcc verbose" FORCE)
SET(LIB_TYPE STATIC)

catkin_package(
    CATKIN_DEPENDS octomap_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  #${PCL_INCLUDE_DIRS}
)
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

#---------------------------------------------------------------------------------------
# For ROS  CUDA template

CUDA_ADD_LIBRARY(OneLib ${LIB_TYPE} src/roscuda_pointers/template.cu)

ADD_EXECUTABLE(cuda_pointer_node src/roscuda_pointers/main.cpp)
ADD_DEPENDENCIES(cuda_pointer_node OneLib)
TARGET_LINK_LIBRARIES(cuda_pointer_node
  ${catkin_LIBRARIES}
  OneLib

)

#-----------------------------------------------------------------------------------------

#For development node

#CUDA_ADD_LIBRARY(TwoLib ${LIB_TYPE} src/development_node/devel.cu)

#ADD_EXECUTABLE(devel_node src/development_node/devel.cpp)
#ADD_DEPENDENCIES(devel_node TwoLib)
#TARGET_LINK_LIBRARIES(devel_node
#  ${catkin_LIBRARIES}
#  TwoLib

#  )

#-----------------------------------------------------------------------------------------

#For Parallel Map Building Node 

CUDA_ADD_LIBRARY(ThreeLib ${LIB_TYPE} src/parallel_map_build/parallel.cu)

ADD_EXECUTABLE(parallel_node src/parallel_map_build/parallel.cpp)
ADD_DEPENDENCIES(parallel_node ThreeLib)
TARGET_LINK_LIBRARIES(parallel_node
  ${catkin_LIBRARIES}
  ThreeLib

)


#-----------------------------------------------------------------------------------------

#For Serial map building node 

CUDA_ADD_LIBRARY(FourLib ${LIB_TYPE} src/serial_map_build/serial.cu)

ADD_EXECUTABLE(serial_node src/serial_map_build/serial.cpp)
ADD_DEPENDENCIES(serial_node FourLib)
TARGET_LINK_LIBRARIES(serial_node
  ${catkin_LIBRARIES}
  FourLib

)


#-----------------------------------------------------------------------------------------

#For PARALLEL FREESPACE model building node 

CUDA_ADD_LIBRARY(FiveLib ${LIB_TYPE} src/parallel_freespace_map/freespace.cu)

ADD_EXECUTABLE(freespace_node src/parallel_freespace_map/freespace.cpp)
ADD_DEPENDENCIES(freespace_node FiveLib)
TARGET_LINK_LIBRARIES(freespace_node
  ${catkin_LIBRARIES}
  FiveLib

)


#For Serial FREESPACE model building node 

CUDA_ADD_LIBRARY(SixLib ${LIB_TYPE} src/serial_freespace_map/freespace.cu)

ADD_EXECUTABLE(freespace_serial_node src/serial_freespace_map/freespace.cpp)
ADD_DEPENDENCIES(freespace_serial_node SixLib)
TARGET_LINK_LIBRARIES(freespace_serial_node
  ${catkin_LIBRARIES}
  SixLib

)

#For CUDA + Octree Obstacles only Mode 

CUDA_ADD_LIBRARY(SevenLib ${LIB_TYPE} src/cuda_with_octree_parallel/devel.cu)

ADD_EXECUTABLE(octree_parallel_node src/cuda_with_octree_parallel/devel.cpp)
ADD_DEPENDENCIES(octree_parallel_node SevenLib)
TARGET_LINK_LIBRARIES(octree_parallel_node
  ${catkin_LIBRARIES}
  SevenLib

)

#For CUDA + Octree Dynamic Object Model  

CUDA_ADD_LIBRARY(EightLib ${LIB_TYPE} src/parallel_dynamic_model/dynamic_parallel.cu)

ADD_EXECUTABLE(dynamic_parallel_node src/parallel_dynamic_model/dynamic_parallel.cpp)
ADD_DEPENDENCIES(dynamic_parallel_node EightLib)
TARGET_LINK_LIBRARIES(dynamic_parallel_node
  ${catkin_LIBRARIES}
  EightLib
)

# for models with multi-resolutional octrees  -------------------------------------------------------------------------
#----------------------------------------------------------------------------------------------------------------------
# multi-resolution octree, obstacle only model, serial version change the .cu file to switch. 

CUDA_ADD_LIBRARY(NineLib ${LIB_TYPE} src/multires_static/parallel.cu)

ADD_EXECUTABLE(multires_static_node src/multires_static/map_builder.cpp)
ADD_DEPENDENCIES(multires_static_node NineLib)
TARGET_LINK_LIBRARIES(multires_static_node
  ${catkin_LIBRARIES}
  NineLib
)


# multi-resolution octree, full environment model. to switch between parallel, and serial versions, change .cu file 

CUDA_ADD_LIBRARY(TenLib ${LIB_TYPE} src/multires_dynamic/parallel.cu)

ADD_EXECUTABLE(multires_dynamic_node src/multires_dynamic/map_builder.cpp)
ADD_DEPENDENCIES(multires_dynamic_node TenLib)
TARGET_LINK_LIBRARIES(multires_dynamic_node
  ${catkin_LIBRARIES}
  TenLib
)
