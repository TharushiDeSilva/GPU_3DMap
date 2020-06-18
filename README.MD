Using SIMD architecture to increase performance of 3D Occupancy Grid Map building process such as OctoMap

developed for ROS kinetic (since hardware testing done on Turtlebots)

How to Use: 

INSTALL AND BUILD:  
1. Clone the repository from GitHub
2. Go to project root and: 
    sudo rosdep install -y --from-paths src --ignore-src --rosdistro kinetic 
3. catkin_make_isolated

RUN:
1. Run turtlebot from Gazebo  
    roslaunch testworld_gazebo testworld.launch 
    roslaunch testworld_gazebo drive_turtlebot.launch 

    or 

    use any robot platform (hw/gazebo) equipped with RGBD sensor which publishes to the ROS topic /camera/depth_registered/points, and /odom  
    (find the equivalent topic and remap)

2. Execute the map building with traditional Octrees 
    1. Maps for Humans: 
        rosrun map_builder multires_static_node
    2. Maps for Robots: 
        rosrun map_builder multires_raycast_node
    (You can evaluate the process with or without CUDA. Refer to CMakeLists.txt for how to)

3. Execute the map building with Morton Code based Octrees
    1. Maps for Humans: 
        rosrun map_builder final_static_node
    2. Maps for Robots: 
        rosrun map_builder final_dynamic_node
    (You can evaluate the process with or without CUDA. Refer to CMakeLists.txt for how to)



