// This is the real Hello World for CUDA!
//It takes the string 'Hello', prints it, then passes it to cuda with an array of offsets
// Then the offsets are added in parallel to produce the string world! 
#include <iostream>
#include <bits/stdc++.h> 
#include <stdint.h>
#include <fstream>
#include <cuda.h>
#include <cuda_runtime.h>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <time.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <string>
#include <cmath> 
#include "math.h"
#include <cstdlib>
#include <stdio.h> 
#include <iterator>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include "geometry_msgs/Quaternion.h"
#include <boost/unordered_map.hpp>

using namespace std; 

// ==============================================================================================================================
int axis_length = 65536; // 2^16; 
const int half_axis_length = 32768; 

const float resolution = 0.05f; 	// Resolution of 5 cm
const float half_resolution = resolution/2; // the half of the resolution. this value is used in calculations 
 
const float max_sensor_radius = 3.00f; 	// scan data further than this modulus value will not be considered. 
 
void round_off_positional_coords(int size, float* x, float* y, float* z, float* x_result, float* y_result, float* z_result, 
	float x_trans, float y_trans, float z_trans, double sin_a, double sin_b, double sin_g, double cos_a, double cos_b, double cos_g){
	
	//int index = threadIdx.x + blockIdx.x * blockDim.x;
	
	for(int i=0; i<size; i++){
        if( (fabs(x[i]) < max_sensor_radius) and (fabs(y[i]) < max_sensor_radius) and (fabs(z[i]) < max_sensor_radius)){
            
            // C: Transformation model 2 ---------- Roll, pitch, yaw combined--------- for inclined planes navigation --------------
            float x_temp = x[i]*cos_a*cos_b + y[i]*cos_a*sin_b*sin_g - y[i]*sin_a*cos_g + z[i]*cos_a*sin_b*cos_g + z[i]*sin_a*sin_g + x_trans;  
            float y_temp = x[i]*sin_a*cos_b + y[i]*sin_a*sin_b*sin_g + y[i]*cos_a*cos_g + z[i]*sin_a*sin_b*cos_g - z[i]*cos_a*sin_g + y_trans; 
            float z_temp = x[i]*sin_b*-1 + y[i]*cos_b*sin_g + z[i]*cos_b*cos_g + z_trans; 
            
            x_result[i] = (ceilf(x_temp / resolution))*0.05 - half_resolution;
            y_result[i] = (ceilf(y_temp / resolution))*0.05 - half_resolution; 
            z_result[i] = (ceilf(z_temp / resolution))*0.05 - half_resolution;

            // -----------------------------------------------------------------------------------------------------------------------------------------
            
        }else{
            x_result[i] = 0.00f; 
            y_result[i] = 0.00f; 
            z_result[i] = 0.00f; 

        } 
    }
	
}
inline uint64_t generate_morton_code(uint16_t x, uint16_t y, uint16_t z){
    uint64_t x_original = uint64_t(x);
    uint64_t y_original = uint64_t(y);
    uint64_t z_original = uint64_t(z);
    
    // x_oroginal = 0000000000000000 0000000000000000 0000000000000000 1111111111111111
    uint64_t x_temp = (x_original<<32) & (0b0000000000000000100000000000000000000000000000000000000000000000); 
    // x_temp = 0000000000000000 1000000000000000 0000000000000000 0000000000000000
    x_temp = ((x_original<<30) & (0b0000000000000000000100000000000000000000000000000000000000000000)) | x_temp; 
    // x_temp = 0000000000000000 1001000000000000 0000000000000000 0000000000000000
    x_temp = ((x_original<<28) & (0b0000000000000000000000100000000000000000000000000000000000000000)) | x_temp;
    // x_temp = 0000000000000000 1001001000000000 0000000000000000 0000000000000000
    x_temp = ((x_original<<26) & (0b0000000000000000000000000100000000000000000000000000000000000000)) | x_temp;
    // x_temp = 0000000000000000 1001001001000000 0000000000000000 0000000000000000
    x_temp = ((x_original<<24) & (0b0000000000000000000000000000100000000000000000000000000000000000)) | x_temp;
    // x_temp = 0000000000000000 1001001001001000 0000000000000000 0000000000000000
    x_temp = ((x_original<<22) & (0b0000000000000000000000000000000100000000000000000000000000000000)) | x_temp;
    // x_temp = 0000000000000000 1001001001001001 0000000000000000 0000000000000000
    x_temp = ((x_original<<20) & (0b0000000000000000000000000000000000100000000000000000000000000000)) | x_temp;
    // x_temp = 0000000000000000 1001001001001001 0010000000000000 0000000000000000
    x_temp = ((x_original<<18) & (0b0000000000000000000000000000000000000100000000000000000000000000)) | x_temp;
    // x_temp = 0000000000000000 1001001001001001 0010010000000000 0000000000000000
    x_temp = ((x_original<<16) & (0b0000000000000000000000000000000000000000100000000000000000000000)) | x_temp;
    // x_temp = 0000000000000000 1001001001001001 0010010010000000 0000000000000000
    x_temp = ((x_original<<14) & (0b0000000000000000000000000000000000000000000100000000000000000000)) | x_temp;
    // x_temp = 0000000000000000 1001001001001001 0010010010010000 0000000000000000
    x_temp = ((x_original<<12) & (0b0000000000000000000000000000000000000000000000100000000000000000)) | x_temp;
    // x_temp = 0000000000000000 1001001001001001 0010010010010010 0000000000000000
    x_temp = ((x_original<<10) & (0b0000000000000000000000000000000000000000000000000100000000000000)) | x_temp;
    // x_temp = 0000000000000000 1001001001001001 0010010010010010 0100000000000000
    x_temp = ((x_original<<8) &  (0b0000000000000000000000000000000000000000000000000000100000000000)) | x_temp;
    // x_temp = 0000000000000000 1001001001001001 0010010010010010 0100100000000000
    x_temp = ((x_original<<6) &  (0b0000000000000000000000000000000000000000000000000000000100000000)) | x_temp;
    // x_temp = 0000000000000000 1001001001001001 0010010010010010 0100100100000000
    x_temp = ((x_original<<4) &  (0b0000000000000000000000000000000000000000000000000000000000100000)) | x_temp;
    // x_temp = 0000000000000000 1001001001001001 0010010010010010 0100100100100000
    x_temp = ((x_original<<2) &  (0b0000000000000000000000000000000000000000000000000000000000000100)) | x_temp;
    // x_temp = 0000000000000000 1001001001001001 0010010010010010 0100100100100100
    x_temp = (x_temp>>2); 

    uint64_t m_code = x_temp; 

    // y_oroginal = 0000000000000000 0000000000000000 0000000000000000 1111111111111111
    uint64_t y_temp = (y_original<<32) & (0b0000000000000000100000000000000000000000000000000000000000000000); 
    // y_temp = 0000000000000000 1000000000000000 0000000000000000 0000000000000000
    y_temp = ((y_original<<30) & (0b0000000000000000000100000000000000000000000000000000000000000000)) | y_temp; 
    // y_temp = 0000000000000000 1001000000000000 0000000000000000 0000000000000000
    y_temp = ((y_original<<28) & (0b0000000000000000000000100000000000000000000000000000000000000000)) | y_temp;
    // y_temp = 0000000000000000 1001001000000000 0000000000000000 0000000000000000
    y_temp = ((y_original<<26) & (0b0000000000000000000000000100000000000000000000000000000000000000)) | y_temp;
    // y_temp = 0000000000000000 1001001001000000 0000000000000000 0000000000000000
    y_temp = ((y_original<<24) & (0b0000000000000000000000000000100000000000000000000000000000000000)) | y_temp;
    // y_temp = 0000000000000000 1001001001001000 0000000000000000 0000000000000000
    y_temp = ((y_original<<22) & (0b0000000000000000000000000000000100000000000000000000000000000000)) | y_temp;
    // y_temp = 0000000000000000 1001001001001001 0000000000000000 0000000000000000
    y_temp = ((y_original<<20) & (0b0000000000000000000000000000000000100000000000000000000000000000)) | y_temp;
    // y_temp = 0000000000000000 1001001001001001 0010000000000000 0000000000000000
    y_temp = ((y_original<<18) & (0b0000000000000000000000000000000000000100000000000000000000000000)) | y_temp;
    // y_temp = 0000000000000000 1001001001001001 0010010000000000 0000000000000000
    y_temp = ((y_original<<16) & (0b0000000000000000000000000000000000000000100000000000000000000000)) | y_temp;
    // y_temp = 0000000000000000 1001001001001001 0010010010000000 0000000000000000
    y_temp = ((y_original<<14) & (0b0000000000000000000000000000000000000000000100000000000000000000)) | y_temp;
    // y_temp = 0000000000000000 1001001001001001 0010010010010000 0000000000000000
    y_temp = ((y_original<<12) & (0b0000000000000000000000000000000000000000000000100000000000000000)) | y_temp;
    // y_temp = 0000000000000000 1001001001001001 0010010010010010 0000000000000000
    y_temp = ((y_original<<10) & (0b0000000000000000000000000000000000000000000000000100000000000000)) | y_temp;
    // y_temp = 0000000000000000 1001001001001001 0010010010010010 0100000000000000
    y_temp = ((y_original<<8) &  (0b0000000000000000000000000000000000000000000000000000100000000000)) | y_temp;
    // y_temp = 0000000000000000 1001001001001001 0010010010010010 0100100000000000
    y_temp = ((y_original<<6) &  (0b0000000000000000000000000000000000000000000000000000000100000000)) | y_temp;
    // y_temp = 0000000000000000 1001001001001001 0010010010010010 0100100100000000
    y_temp = ((y_original<<4) &  (0b0000000000000000000000000000000000000000000000000000000000100000)) | y_temp;
    // y_temp = 0000000000000000 1001001001001001 0010010010010010 0100100100100000
    y_temp = ((y_original<<2) &  (0b0000000000000000000000000000000000000000000000000000000000000100)) | y_temp;
    // y_temp = 0000000000000000 1001001001001001 0010010010010010 0100100100100100
    y_temp = (y_temp>>1); 

    m_code = (m_code | y_temp); 

    // z_oroginal = 0000000000000000 0000000000000000 0000000000000000 1111111111111111
    uint64_t z_temp = (z_original<<32) & (0b0000000000000000100000000000000000000000000000000000000000000000); 
    // z_temp = 0000000000000000 1000000000000000 0000000000000000 0000000000000000
    z_temp = ((z_original<<30) & (0b0000000000000000000100000000000000000000000000000000000000000000)) | z_temp; 
    // z_temp = 0000000000000000 1001000000000000 0000000000000000 0000000000000000
    z_temp = ((z_original<<28) & (0b0000000000000000000000100000000000000000000000000000000000000000)) | z_temp;
    // z_temp = 0000000000000000 1001001000000000 0000000000000000 0000000000000000
    z_temp = ((z_original<<26) & (0b0000000000000000000000000100000000000000000000000000000000000000)) | z_temp;
    // z_temp = 0000000000000000 1001001001000000 0000000000000000 0000000000000000
    z_temp = ((z_original<<24) & (0b0000000000000000000000000000100000000000000000000000000000000000)) | z_temp;
    // z_temp = 0000000000000000 1001001001001000 0000000000000000 0000000000000000
    z_temp = ((z_original<<22) & (0b0000000000000000000000000000000100000000000000000000000000000000)) | z_temp;
    // z_temp = 0000000000000000 1001001001001001 0000000000000000 0000000000000000
    z_temp = ((z_original<<20) & (0b0000000000000000000000000000000000100000000000000000000000000000)) | z_temp;
    // z_temp = 0000000000000000 1001001001001001 0010000000000000 0000000000000000
    z_temp = ((z_original<<18) & (0b0000000000000000000000000000000000000100000000000000000000000000)) | z_temp;
    // z_temp = 0000000000000000 1001001001001001 0010010000000000 0000000000000000
    z_temp = ((z_original<<16) & (0b0000000000000000000000000000000000000000100000000000000000000000)) | z_temp;
    // z_temp = 0000000000000000 1001001001001001 0010010010000000 0000000000000000
    z_temp = ((z_original<<14) & (0b0000000000000000000000000000000000000000000100000000000000000000)) | z_temp;
    // z_temp = 0000000000000000 1001001001001001 0010010010010000 0000000000000000
    z_temp = ((z_original<<12) & (0b0000000000000000000000000000000000000000000000100000000000000000)) | z_temp;
    // z_temp = 0000000000000000 1001001001001001 0010010010010010 0000000000000000
    z_temp = ((z_original<<10) & (0b0000000000000000000000000000000000000000000000000100000000000000)) | z_temp;
    // z_temp = 0000000000000000 1001001001001001 0010010010010010 0100000000000000
    z_temp = ((z_original<<8) &  (0b0000000000000000000000000000000000000000000000000000100000000000)) | z_temp;
    // z_temp = 0000000000000000 1001001001001001 0010010010010010 0100100000000000
    z_temp = ((z_original<<6) &  (0b0000000000000000000000000000000000000000000000000000000100000000)) | z_temp;
    // z_temp = 0000000000000000 1001001001001001 0010010010010010 0100100100000000
    z_temp = ((z_original<<4) &  (0b0000000000000000000000000000000000000000000000000000000000100000)) | z_temp;
    // z_temp = 0000000000000000 1001001001001001 0010010010010010 0100100100100000
    z_temp = ((z_original<<2) &  (0b0000000000000000000000000000000000000000000000000000000000000100)) | z_temp;
    // z_temp = 0000000000000000 1001001001001001 0010010010010010 0100100100100100
    
    m_code = (m_code | z_temp); 

    return m_code;
}

inline uint32_t create_node_value(uint8_t r, uint8_t g, uint8_t b, int8_t occ){
    //0-15 = free
    // 16  = unobserved
    // 17-32 - occupied 
    uint32_t r_temp = uint32_t(r); 
    uint32_t g_temp = uint32_t(g); 
    uint32_t b_temp = uint32_t(b); 
    uint32_t occ_temp = uint32_t(occ);
    uint32_t result = (r_temp<<24) | (g_temp<<16) | (b_temp<<8) | occ_temp; 
    return result; 
}

extern boost::unordered::unordered_map<uint64_t, uint32_t> octree; 

int cudamain(sensor_msgs::PointCloud2 point_cloud_std, nav_msgs::Odometry odom_message_std){ 

    double starttotal, endtotal; 
	starttotal = clock();
	//make_range_array(resolution, max_sensor_radius); 
	int array_size = 640*480; 	 // this is constant for camera - indicates camera resolution 
    
    double start1, end1; 
    start1 = clock();
    
	double roll, pitch, yaw; 
	tf::Quaternion quat;
    tf::quaternionMsgToTF(odom_message_std.pose.pose.orientation, quat);	
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
	float x_trans = (float) odom_message_std.pose.pose.position.x; 
	float y_trans = (float) odom_message_std.pose.pose.position.y; 
	float z_trans = (float) odom_message_std.pose.pose.position.z; 

	double sin_g = sin(roll); 
	double sin_b = sin(pitch); 
	double sin_a = sin(yaw); 
	double cos_g = cos(roll); 
	double cos_b = cos(pitch); 
	double cos_a = cos(yaw); 

	int counter = 0; 
	
	//declare the arrray sets before reading the point cloud values 
	
	float *x, *y, *z; // for allocating position values of the points 
	uint8_t *r, *g, *b; // for color values of the point cloud 
	uint16_t *x_rounded, *y_rounded, *z_rounded; 							// the intermediate results after rounding off the x, y, z, original values to the resolution 
	
    int size_int_arr = array_size * sizeof(uint16_t);
    x_rounded = (uint16_t *)malloc( size_int_arr );
   	y_rounded = (uint16_t *)malloc( size_int_arr );
	z_rounded = (uint16_t *)malloc( size_int_arr );

	int size_position = array_size * sizeof(float);
	int size_color = array_size * sizeof(uint8_t);
	
	x = (float *)malloc( size_position );
   	y = (float *)malloc( size_position );
	z = (float *)malloc( size_position );

	r = (uint8_t *)malloc( size_color );
    g = (uint8_t *)malloc( size_color );
	b = (uint8_t *)malloc( size_color );
	
	// positional data vector generation 
	for(sensor_msgs::PointCloud2ConstIterator<float> it(point_cloud_std, "x"); it!=it.end(); ++it){
		y[counter] = it[0] * -1; 
		z[counter] = it[1] * -1;
		x[counter] = it[2];
		counter+=1;  
		
	}
	counter = 0; 
    for(sensor_msgs::PointCloud2ConstIterator<uint8_t> it_color(point_cloud_std, "rgb"); it_color!=it_color.end(); ++it_color){
		b[counter] = unsigned(it_color[0]);	
		g[counter] = unsigned(it_color[1]);	
		r[counter] = unsigned(it_color[2]); 
		counter+=1; 
	}
	counter = 0; 
    // double start1, end1; 
    // start1 = clock();
    end1 = clock();
    double time1 = (double)(end1 - start1);

    double start2, end2; 
    start2 = clock();
    // end1 = clock();
    // double timetotal = (double)(end1 - start1);
    // generate integer array for floats
    for(int i=0; i<array_size; i++){
        if( (fabs(x[i]) < max_sensor_radius) and (fabs(y[i]) < max_sensor_radius) and (fabs(z[i]) < max_sensor_radius)){
            
            // C: Transformation model 2 ---------- Roll, pitch, yaw combined--------- for inclined planes navigation --------------
            float x_temp = x[i]*cos_a*cos_b + y[i]*cos_a*sin_b*sin_g - y[i]*sin_a*cos_g + z[i]*cos_a*sin_b*cos_g + z[i]*sin_a*sin_g + x_trans;  
            float y_temp = x[i]*sin_a*cos_b + y[i]*sin_a*sin_b*sin_g + y[i]*cos_a*cos_g + z[i]*sin_a*sin_b*cos_g - z[i]*cos_a*sin_g + y_trans; 
            float z_temp = x[i]*sin_b*-1 + y[i]*cos_b*sin_g + z[i]*cos_b*cos_g + z_trans; 
            
            x_rounded[i] = uint16_t(half_axis_length + ceilf(x_temp / resolution) -1 );
            y_rounded[i] = uint16_t(half_axis_length + ceilf(y_temp / resolution) -1 );
            z_rounded[i] = uint16_t(half_axis_length + ceilf(z_temp / resolution) -1 );
            // -----------------------------------------------------------------------------------------------------------------------------------------
            
        }else{
            x_rounded[i] = uint16_t(half_axis_length-1);
            y_rounded[i] = uint16_t(half_axis_length-1);
            z_rounded[i] = uint16_t(half_axis_length-1); 

        } 
    }

    // double start2, end2; 
    // start2 = clock();
    end2 = clock();
    double time2 = (double)(end2 - start2);

    double start3, end3; 
    start3 = clock();
    // end2 = clock();
    // double timetotal = (double)(end2 - start2);

    // insert into octree 
    for(int i=0; i<array_size; i++){
        // Morton code for null points: 0b0000000000000000100011011011011011011011011011011011011011011011;
        uint64_t morton_code = generate_morton_code(x_rounded[i], y_rounded[i], z_rounded[i]); 
        //search for the above code in octree
        if(morton_code != 0b0000000000000000100011011011011011011011011011011011011011011011){
            boost::unordered::unordered_map<uint64_t, uint32_t>::iterator itr;  // to find the key 
            itr = octree.find(morton_code);
            if (itr == octree.end()){
                uint32_t value = create_node_value(r[i], g[i], b[i], 0b00010001); 
                octree.insert(std::make_pair<uint64_t, uint32_t>(morton_code, value));
                //std::cout<<i<<"\t"<<std::bitset<32>(value)<<std::endl;
            }else{
                uint32_t value = (itr->second); 
                
                // get r/2 from value 
                uint32_t r_value = uint32_t(r[i]) & 0b00000000000000000000000011111111;
                uint32_t g_value = uint32_t(g[i]) & 0b00000000000000000000000011111111;
                uint32_t b_value = uint32_t(b[i]) & 0b00000000000000000000000011111111; 

                // these operations are for a occupied node
                uint32_t occ_value = value & 0b00000000000000000000000011111111; 
                if (occ_value == 0b00000000000000000000000000001111){   // if -1, make it +1 
                    occ_value = 0b00000000000000000000000000010001;  
                }else if (occ_value < 0b00000000000000000000000000100000){  
                    occ_value += 0b00000000000000000000000000000001;  
                }
            
                value = (r_value<<24) | (g_value<<16) | (b_value<<8) | occ_value; 
                itr->second = value; 
                //std::cout<<i<<"\t"<<unsigned(r[i])<<"\t"<<unsigned(g[i])<<"\t"<<unsigned(b[i])<<"\t"<<std::bitset<32>(value)<<std::endl;
            }
        }    
    }	
    
    // double start3, end3; 
    // start3 = clock();
    end3 = clock();
    double time3 = (double)(end3 - start3);
	
    free(x);
    free(y);
	free(z);
	free(r);
	free(g);
	free(b);
	free(x_rounded); 
	free(y_rounded); 
	free(z_rounded);
	
	
    endtotal = clock();
    double timetotal = (double)(endtotal - starttotal);
    std::cout<<time1<<"\t"<<time2<<"\t"<<time3<<"\t"<<timetotal<<endl;
    //std::cout<<octree.size()<<endl;
	return EXIT_SUCCESS; 	
}
