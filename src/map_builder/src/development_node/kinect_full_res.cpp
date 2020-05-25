// main class for capturing kinect full point cloud faced to a wall 

#include "iostream"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_ros/transforms.h>
#include <boost/thread/mutex.hpp>
#include <boost/algorithm/string.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf/transform_datatypes.h>
//#include "LinearMath/Matrix3x3.h"
#include <geometry_msgs/Vector3.h>
#include "geometry_msgs/Quaternion.h"

using namespace std; 
using namespace pcl; 

static int counter =1; 

//int testmain(pcl::PointCloud<pcl::PointXYZRGB> point_cloud_std); 
int cudamain(sensor_msgs::PointCloud2 point_cloud_std, int array_size); 

void cloud_callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input){
	counter+=1;
	sensor_msgs::PointCloud2 in_cloud_2 = *input; 
	sensor_msgs::PointCloud cloud_converted; 
   	sensor_msgs::convertPointCloud2ToPointCloud(in_cloud_2, cloud_converted); 
   	int array_size = cloud_converted.points.size();
	cudamain(in_cloud_2, array_size); 
    //write octomap to file
	 
}

int main(int argc, char **argv){ 
    ros::init(argc, argv, "color_pointcloud_sub");

    ros::NodeHandle nh;  
    //message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub(nh, "/camera/depth_registered/points", 10); 
    ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", 1000, cloud_callback); 
    
    ros::Rate loop_rate(0.1);    // frequency 
    while(ros::ok()){  	 
        // mtx_spin.lock(); 
        ros::spinOnce();       
    }
    //ros::spin(); 
    return 0; 
}
