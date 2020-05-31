//This node is used to read data from a color octomap_msgs/Octomap, 
//transform according to a given transformation and save in .ot format. 

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
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Vector3.h>
#include "geometry_msgs/Quaternion.h"
#include <boost/unordered_map.hpp>

using namespace std; 
using namespace pcl; 

static int counter =1; 
extern boost::unordered::unordered_map<uint64_t, uint32_t> octree;
static int half_axis_length = 32768;
static float resolution = 0.05f; 

//int testmain(pcl::PointCloud<pcl::PointXYZRGB> point_cloud_std); 
int cudamain(sensor_msgs::PointCloud2 point_cloud_std, nav_msgs::Odometry odom_message_std);  

void synced_callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input_cloud, const boost::shared_ptr<const nav_msgs::Odometry>& input_odom){
    counter +=1; 
    sensor_msgs::PointCloud2 in_cloud_2 = *input_cloud; 
	sensor_msgs::PointCloud cloud_converted; 
   	sensor_msgs::convertPointCloud2ToPointCloud(in_cloud_2, cloud_converted); 
   	int array_size = cloud_converted.points.size();
	nav_msgs::Odometry in_odom = *input_odom;     
    //std::cout<<"processing called"<<std::endl; 
    cudamain(in_cloud_2, in_odom); 
}

const double map_resolution = 0.05; //octomap resolution = 5cm 
const float base_height = -0.30f; 			// height of the navigation plane from the camera
const float base_nav_height = base_height - map_resolution;

int main(int argc, char **argv){
    int seq_num = 0; 
    ros::init(argc, argv, "color_pointcloud_sub");

    ros::NodeHandle nh;  
    message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub(nh, "/camera/depth_registered/points", 10); // queue size 
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "/odom", 10);   // queue size 
    std::cout<<"initialized subscribers to point cloud and odometry"<<std::endl; 

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> my_sync_policy; 
    
    message_filters::Synchronizer<my_sync_policy> sync(my_sync_policy(20), pointcloud_sub, odom_sub);
    sync.registerCallback(boost::bind(&synced_callback, _1, _2));
    
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2> ("points2", 1, true);
    //ros::Publisher pub_grid = nh.advertise<sensor_msgs::PointCloud2> ("Obstacles", 1, true);
    
    // boost::mutex mtx_spin;  
    ros::Rate loop_rate(0.1);    // frequency 
    while(ros::ok()){  	 
        // mtx_spin.lock(); 
        ros::spinOnce();      
        seq_num +=1; 
        pcl::PointCloud<pcl::PointXYZRGB> cloud_;

        boost::unordered::unordered_map<uint64_t, uint32_t>::iterator it;
        for (it=octree.begin(); it!=octree.end(); it++){
            uint64_t code = it->first; 
            uint32_t rgbo = it->second; 
            
            uint64_t x_int =  code & 0b0000000000000000000000000000000000000000000000000000000000000001; 
            // x int - 0b 0000000000000000 0000000000000000 0000000000000000 0000000000000001 
            x_int = x_int | (code>>2 & 0b0000000000000000000000000000000000000000000000000000000000000010);
            // x int - 0b 0000000000000000 0000000000000000 0000000000000000 0000000000000011 
            x_int = x_int | (code>>4 & 0b0000000000000000000000000000000000000000000000000000000000000100);
            // x int - 0b 0000000000000000 0000000000000000 0000000000000000 0000000000000111 
            x_int = x_int | (code>>6 & 0b0000000000000000000000000000000000000000000000000000000000001000);
            // x int - 0b 0000000000000000 0000000000000000 0000000000000000 0000000000001111 
            x_int = x_int | (code>>8 & 0b0000000000000000000000000000000000000000000000000000000000010000);
            // x int - 0b 0000000000000000 0000000000000000 0000000000000000 0000000000011111 
            x_int = x_int | (code>>10 & 0b0000000000000000000000000000000000000000000000000000000000100000);
            // x int - 0b 0000000000000000 0000000000000000 0000000000000000 0000000000111111 
            x_int = x_int | (code>>12 & 0b0000000000000000000000000000000000000000000000000000000001000000);
            // x int - 0b 0000000000000000 0000000000000000 0000000000000000 0000000001111111 
            x_int = x_int | (code>>14 & 0b0000000000000000000000000000000000000000000000000000000010000000);
            // x int - 0b 0000000000000000 0000000000000000 0000000000000000 0000000011111111 
            x_int = x_int | (code>>16 & 0b0000000000000000000000000000000000000000000000000000000100000000);
            // x int - 0b 0000000000000000 0000000000000000 0000000000000000 0000000111111111 
            x_int = x_int | (code>>18 & 0b0000000000000000000000000000000000000000000000000000001000000000);
            // x int - 0b 0000000000000000 0000000000000000 0000000000000000 0000001111111111 
            x_int = x_int | (code>>20 & 0b0000000000000000000000000000000000000000000000000000010000000000);
            // x int - 0b 0000000000000000 0000000000000000 0000000000000000 0000011111111111 
            x_int = x_int | (code>>22 & 0b0000000000000000000000000000000000000000000000000000100000000000);
            // x int - 0b 0000000000000000 0000000000000000 0000000000000000 0000111111111111 
            x_int = x_int | (code>>24 & 0b0000000000000000000000000000000000000000000000000001000000000000);
            // x int - 0b 0000000000000000 0000000000000000 0000000000000000 0001111111111111 
            x_int = x_int | (code>>26 & 0b0000000000000000000000000000000000000000000000000010000000000000);
            // x int - 0b 0000000000000000 0000000000000000 0000000000000000 0011111111111111 
            x_int = x_int | (code>>28 & 0b0000000000000000000000000000000000000000000000000100000000000000);
            // x int - 0b 0000000000000000 0000000000000000 0000000000000000 0111111111111111 
            x_int = x_int | (code>>30 & 0b0000000000000000000000000000000000000000000000001000000000000000);
            // x int - 0b 0000000000000000 0000000000000000 0000000000000000 1111111111111111 

            uint64_t y_int =  (code>>1) & 0b0000000000000000000000000000000000000000000000000000000000000001; 
            // y int - 0b 0000000000000000 0000000000000000 0000000000000000 0000000000000001 
            y_int = y_int | (code>>3 & 0b0000000000000000000000000000000000000000000000000000000000000010);
            // y int - 0b 0000000000000000 0000000000000000 0000000000000000 0000000000000011 
            y_int = y_int | (code>>5 & 0b0000000000000000000000000000000000000000000000000000000000000100);
            // y int - 0b 0000000000000000 0000000000000000 0000000000000000 0000000000000111 
            y_int = y_int | (code>>7 & 0b0000000000000000000000000000000000000000000000000000000000001000);
            // y int - 0b 0000000000000000 0000000000000000 0000000000000000 0000000000001111 
            y_int = y_int | (code>>9 & 0b0000000000000000000000000000000000000000000000000000000000010000);
            // y int - 0b 0000000000000000 0000000000000000 0000000000000000 0000000000011111 
            y_int = y_int | (code>>11 & 0b0000000000000000000000000000000000000000000000000000000000100000);
            // y int - 0b 0000000000000000 0000000000000000 0000000000000000 0000000000111111 
            y_int = y_int | (code>>13 & 0b0000000000000000000000000000000000000000000000000000000001000000);
            // y int - 0b 0000000000000000 0000000000000000 0000000000000000 0000000001111111 
            y_int = y_int | (code>>15 & 0b0000000000000000000000000000000000000000000000000000000010000000);
            // y int - 0b 0000000000000000 0000000000000000 0000000000000000 0000000011111111 
            y_int = y_int | (code>>17 & 0b0000000000000000000000000000000000000000000000000000000100000000);
            // y int - 0b 0000000000000000 0000000000000000 0000000000000000 0000000111111111 
            y_int = y_int | (code>>19 & 0b0000000000000000000000000000000000000000000000000000001000000000);
            // y int - 0b 0000000000000000 0000000000000000 0000000000000000 0000001111111111 
            y_int = y_int | (code>>21 & 0b0000000000000000000000000000000000000000000000000000010000000000);
            // y int - 0b 0000000000000000 0000000000000000 0000000000000000 0000011111111111 
            y_int = y_int | (code>>23 & 0b0000000000000000000000000000000000000000000000000000100000000000);
            // y int - 0b 0000000000000000 0000000000000000 0000000000000000 0000111111111111 
            y_int = y_int | (code>>25 & 0b0000000000000000000000000000000000000000000000000001000000000000);
            // y int - 0b 0000000000000000 0000000000000000 0000000000000000 0001111111111111 
            y_int = y_int | (code>>27 & 0b0000000000000000000000000000000000000000000000000010000000000000);
            // y int - 0b 0000000000000000 0000000000000000 0000000000000000 0011111111111111 
            y_int = y_int | (code>>29 & 0b0000000000000000000000000000000000000000000000000100000000000000);
            // y int - 0b 0000000000000000 0000000000000000 0000000000000000 0111111111111111 
            y_int = y_int | (code>>31 & 0b0000000000000000000000000000000000000000000000001000000000000000);
            // y int - 0b 0000000000000000 0000000000000000 0000000000000000 1111111111111111 
            
            uint64_t z_int =  (code>>2) & 0b0000000000000000000000000000000000000000000000000000000000000001; 
            // z int - 0b 0000000000000000 0000000000000000 0000000000000000 0000000000000001 
            z_int = z_int | (code>>4 & 0b0000000000000000000000000000000000000000000000000000000000000010);
            // z int - 0b 0000000000000000 0000000000000000 0000000000000000 0000000000000011 
            z_int = z_int | (code>>6 & 0b0000000000000000000000000000000000000000000000000000000000000100);
            // z int - 0b 0000000000000000 0000000000000000 0000000000000000 0000000000000111 
            z_int = z_int | (code>>8 & 0b0000000000000000000000000000000000000000000000000000000000001000);
            // z int - 0b 0000000000000000 0000000000000000 0000000000000000 0000000000001111 
            z_int = z_int | (code>>10 & 0b0000000000000000000000000000000000000000000000000000000000010000);
            // z int - 0b 0000000000000000 0000000000000000 0000000000000000 0000000000011111 
            z_int = z_int | (code>>12 & 0b0000000000000000000000000000000000000000000000000000000000100000);
            // z int - 0b 0000000000000000 0000000000000000 0000000000000000 0000000000111111 
            z_int = z_int | (code>>14 & 0b0000000000000000000000000000000000000000000000000000000001000000);
            // z int - 0b 0000000000000000 0000000000000000 0000000000000000 0000000001111111 
            z_int = z_int | (code>>16 & 0b0000000000000000000000000000000000000000000000000000000010000000);
            // z int - 0b 0000000000000000 0000000000000000 0000000000000000 0000000011111111 
            z_int = z_int | (code>>18 & 0b0000000000000000000000000000000000000000000000000000000100000000);
            // z int - 0b 0000000000000000 0000000000000000 0000000000000000 0000000111111111 
            z_int = z_int | (code>>20 & 0b0000000000000000000000000000000000000000000000000000001000000000);
            // z int - 0b 0000000000000000 0000000000000000 0000000000000000 0000001111111111 
            z_int = z_int | (code>>22 & 0b0000000000000000000000000000000000000000000000000000010000000000);
            // z int - 0b 0000000000000000 0000000000000000 0000000000000000 0000011111111111 
            z_int = z_int | (code>>24 & 0b0000000000000000000000000000000000000000000000000000100000000000);
            // z int - 0b 0000000000000000 0000000000000000 0000000000000000 0000111111111111 
            z_int = z_int | (code>>26 & 0b0000000000000000000000000000000000000000000000000001000000000000);
            // z int - 0b 0000000000000000 0000000000000000 0000000000000000 0001111111111111 
            z_int = z_int | (code>>28 & 0b0000000000000000000000000000000000000000000000000010000000000000);
            // z int - 0b 0000000000000000 0000000000000000 0000000000000000 0011111111111111 
            z_int = z_int | (code>>30 & 0b0000000000000000000000000000000000000000000000000100000000000000);
            // z int - 0b 0000000000000000 0000000000000000 0000000000000000 0111111111111111 
            z_int = z_int | (code>>32 & 0b0000000000000000000000000000000000000000000000001000000000000000);
            // z int - 0b 0000000000000000 0000000000000000 0000000000000000 1111111111111111 
            
            
            uint16_t x_final_int = uint16_t(x_int); 
            uint16_t y_final_int = uint16_t(y_int); 
            uint16_t z_final_int = uint16_t(z_int); 
            // convert back to float 
            float x_float = (x_final_int + 1 - half_axis_length)*resolution; 
            float y_float = (y_final_int + 1 - half_axis_length)*resolution; 
            float z_float = (z_final_int + 1 - half_axis_length)*resolution;  

            uint8_t o_final = uint8_t(rgbo & 0b00000000000000000000000011111111); 
            
            //for occupied nodes 
            if(o_final > 0b00010000){
                uint8_t r_final = uint8_t(rgbo>>24 & 0b00000000000000000000000011111111);
                uint8_t g_final = uint8_t(rgbo>>16 & 0b00000000000000000000000011111111);
                uint8_t b_final = uint8_t(rgbo>>8 & 0b00000000000000000000000011111111); 
                
                pcl::PointXYZRGB *pt;
                pt = new pcl::PointXYZRGB(r_final, g_final, b_final);
                pt->x = x_float; pt->y = y_float; pt->z = z_float; 
                cloud_.points.push_back(*pt);
                delete pt; 
            }
        }

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
