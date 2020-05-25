cmake_minimum_required(VERSION 2.8.3)
project(class_example)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  CATKIN_DEPENDS 
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

ADD_EXECUTABLE(test_node src/simple_test.cpp)

TARGET_LINK_LIBRARIES(test_node
  ${catkin_LIBRARIES}
  
)
