cmake_minimum_required(VERSION 2.8.3)
project(map_builder)
FIND_PACKAGE(CUDA REQUIRED 
#PCL REQUIRED COMPONENTS common io
)

#add_compile_options(-std=c++11)

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


# multi-resolution octree, full environment model with full raycasting. to switch between parallel, and serial versions, change .cu file 

CUDA_ADD_LIBRARY(ElevenLib ${LIB_TYPE} src/multires_raycast/parallel.cu)

ADD_EXECUTABLE(multires_raycast_node src/multires_raycast/map_builder.cpp)
ADD_DEPENDENCIES(multires_raycast_node ElevenLib)
TARGET_LINK_LIBRARIES(multires_raycast_node
  ${catkin_LIBRARIES}
  ElevenLib
)


# -----------------------------------------------------------------------------------------------------------------
# -------------------------- Morton code based implementation -----------------------------------------------------

# morton code octree, static environmnt only. to switch between parallel, and serial versions, change .cu file 

CUDA_ADD_LIBRARY(TweleveLib ${LIB_TYPE} src/mc_static/parallel.cu)

ADD_EXECUTABLE(mc_static_node src/mc_static/map_builder.cpp)
ADD_DEPENDENCIES(mc_static_node TweleveLib)
TARGET_LINK_LIBRARIES(mc_static_node
  ${catkin_LIBRARIES}
  TweleveLib
)

# morton code octree, full environment. to switch between parallel, and serial versions, change .cu file 

CUDA_ADD_LIBRARY(ThirteenLib ${LIB_TYPE} src/mc_raycast/parallel.cu)

ADD_EXECUTABLE(mc_raycast_node src/mc_raycast/map_builder.cpp)
ADD_DEPENDENCIES(mc_raycast_node ThirteenLib)
TARGET_LINK_LIBRARIES(mc_raycast_node
  ${catkin_LIBRARIES}
  ThirteenLib
)

# direct memory access from octree --------------------------------------------------------

# static environment 
CUDA_ADD_LIBRARY(FourteenLib ${LIB_TYPE} src/final_static/parallel.cu)

ADD_EXECUTABLE(final_static_node src/final_static/map_builder.cpp)
ADD_DEPENDENCIES(final_static_node FourteenLib)
TARGET_LINK_LIBRARIES(final_static_node
  ${catkin_LIBRARIES}
  FourteenLib
)

# dynamic environment 
CUDA_ADD_LIBRARY(FifteenLib ${LIB_TYPE} src/final_dynamic/parallel.cu)

ADD_EXECUTABLE(final_dynamic_node src/final_dynamic/map_builder.cpp)
ADD_DEPENDENCIES(final_dynamic_node FifteenLib)
TARGET_LINK_LIBRARIES(final_dynamic_node
  ${catkin_LIBRARIES}
  FifteenLib
)


# the evaluator node ---------------------------------------------------------------
# dynamic environment 
CUDA_ADD_LIBRARY(SixteenLib ${LIB_TYPE} src/testing/evaluator/dynamic_serial.cu)

ADD_EXECUTABLE(eval_node src/testing/evaluator/map_builder.cpp)
ADD_DEPENDENCIES(eval_node SixteenLib)
TARGET_LINK_LIBRARIES(eval_node
  ${catkin_LIBRARIES}
  SixteenLib
)

# compare the tree algorithms  
CUDA_ADD_LIBRARY(SeventeenLib ${LIB_TYPE} src/testing/algo_comparison/comparison.cu)

ADD_EXECUTABLE(comparison_node src/testing/algo_comparison/comparison.cpp)
ADD_DEPENDENCIES(comparison_node SeventeenLib)
TARGET_LINK_LIBRARIES(comparison_node
  ${catkin_LIBRARIES}
  SeventeenLib
)
