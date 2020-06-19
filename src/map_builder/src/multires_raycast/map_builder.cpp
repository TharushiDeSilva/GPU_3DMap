//K. T. D. S. De Silva ---- University of Moratuwa 
// main module 
// raycast based on obstacle points model of unknown environment 
// traditional octree representation with multi resolutions 
//-------------------------------------------------------------------------------------------------- 

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
int cudamain(sensor_msgs::PointCloud2 point_cloud_std, nav_msgs::Odometry odom_message_std, int array_size); 

void synced_callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input_cloud, const boost::shared_ptr<const nav_msgs::Odometry>& input_odom){
    counter +=1; 
    sensor_msgs::PointCloud2 in_cloud_2 = *input_cloud; 
	sensor_msgs::PointCloud cloud_converted; 
   	sensor_msgs::convertPointCloud2ToPointCloud(in_cloud_2, cloud_converted); 
   	int array_size = cloud_converted.points.size();
	nav_msgs::Odometry in_odom = *input_odom;     
    //std::cout<<"processing called"<<std::endl; 
    cudamain(in_cloud_2, in_odom, array_size); 
    
}

const double map_resolution = 0.05; //octomap resolution = 5cm 
const float base_height = -0.30f; 			// height of the navigation plane from the camera
const float base_nav_height = base_height - map_resolution;

int main(int argc, char **argv){
    int seq_num = 0; 
    ros::init(argc, argv, "color_pointcloud_sub");

    ros::NodeHandle nh;  
    message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub(nh, "/camera/depth_registered/points", 10); 
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "/odom", 10); 
    message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, nav_msgs::Odometry> sync(pointcloud_sub, odom_sub, 10); 
    sync.registerCallback(boost::bind(&synced_callback, _1, _2)); 
    
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2> ("points2", 1, true);
    //ros::Publisher pub_grid = nh.advertise<sensor_msgs::PointCloud2> ("Obstacles", 1, true);
    
    // boost::mutex mtx_spin;  
    ros::Rate loop_rate(0.1);    // frequency 
    while(ros::ok()){  	 
        // mtx_spin.lock(); 
        ros::spinOnce();      
        seq_num +=1; 
        // read the text file and publish octomap 
        
        ifstream ifile;
        ifile.open ("octree_nodes.txt", ios::in);
        float x_map, y_map, z_map;  
        int r_map, g_map, b_map; 
        
        pcl::PointCloud<pcl::PointXYZRGB> cloud_;  
         
        while(ifile >>x_map>>y_map>>z_map>>r_map>>g_map>>b_map){
            // insert the points into a point cloud
            pcl::PointXYZRGB *pt;
            pt = new pcl::PointXYZRGB(uint8_t(r_map), uint8_t(g_map),uint8_t(b_map));
            //pt = new pcl::PointXYZRGB(uint8_t(0), uint8_t(255),uint8_t(0));
            pt->x = x_map; pt->y = y_map; pt->z = z_map; 
            cloud_.points.push_back(*pt);
            delete pt; 
        }
        ifile.close(); 
        
       /*
       ifstream ifile;
        ifile.open ("nav_nodes.txt", ios::in);
        float x_map, y_map, z_map;  
        int occ; 
        
        pcl::PointCloud<pcl::PointXYZRGB> cloud_;  
         
        while(ifile >>x_map>>y_map>>z_map>>occ){
            // insert the points into a point cloud
            pcl::PointXYZRGB *pt;
            if(occ==0){
                pt = new pcl::PointXYZRGB(uint8_t(155), uint8_t(155),uint8_t(155));
            }else if(occ >0){
                pt = new pcl::PointXYZRGB(uint8_t(0), uint8_t(0),uint8_t(0));
            }else{
                pt = new pcl::PointXYZRGB(uint8_t(255), uint8_t(255),uint8_t(255));
            }
            
            pt->x = x_map; pt->y = y_map; pt->z = z_map; 
            cloud_.points.push_back(*pt);
            delete pt; 
        }
        ifile.close(); 
        */
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(cloud_, msg);  
          

        msg.header.seq = seq_num;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "map"; 
        pub.publish (msg);  
        
    }
    //ros::spin(); 
    return 0; 
}
