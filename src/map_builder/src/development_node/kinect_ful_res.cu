// second file for capturing the full scope of kinect points 307200. 
 
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
#include <map>
#include <iterator>
#include <boost/lexical_cast.hpp>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include "geometry_msgs/Quaternion.h"


using namespace std; 

// =========================================================================================================================

int cudamain(sensor_msgs::PointCloud2 point_cloud_std, int size){ 
 
	int array_size = size; 	 
    
	int counter = 0; 
	int effective_point_count = 0; 
	//declare the arrray sets before reading the point cloud values 
	
	float *x, *y, *z; // for allocating position values of the points 
	//float *x_rounded, *y_rounded, *z_rounded; 							// the intermediate results after rounding off the x, y, z, original values to the resolution 
	u_int8_t *r, *g, *b; // for color values of the point cloud 
	

	int size_position = array_size * sizeof(float);
	int size_color = array_size * sizeof(u_int8_t);
	
	x = (float *)malloc( size_position );
   	y = (float *)malloc( size_position );
	z = (float *)malloc( size_position );

	r = (u_int8_t *)malloc( size_color );
    g = (u_int8_t *)malloc( size_color );
	b = (u_int8_t *)malloc( size_color );
	
	// positional data vector generation 
	for(sensor_msgs::PointCloud2ConstIterator<float> it(point_cloud_std, "x"); it!=it.end(); ++it){
		y[counter] = it[0] * -1; 
		z[counter] = it[1] * -1;
		x[counter] = it[2];
		counter+=1;  
		
	}
	counter = 0; 
    for(sensor_msgs::PointCloud2ConstIterator<u_int8_t> it_color(point_cloud_std, "rgb"); it_color!=it_color.end(); ++it_color){
		b[counter] = unsigned(it_color[0]);	
		g[counter] = unsigned(it_color[1]);	
		r[counter] = unsigned(it_color[2]); 
		counter+=1; 
	}
	counter = 0; 
    
    
    std::string map_file_name = "wall_points.txt";
	ofstream offile;
	offile.open(map_file_name.c_str(), ios::app);
	if(offile.is_open()) { 
		
        for(int i=0; i<array_size; i++){ 
            offile<<x[i]<<"\t"<<y[i]<<"\t"<<z[i]<<"\t"<<unsigned(r[i])<<"\t"<<unsigned(g[i])<<"\t"<<unsigned(b[i])<<std::endl;  
        }
        offile<<"end of set ..."<<endl; 
        				
	}
	//std::cout<<"file written"<<endl; 
	offile.close();
    
    std::cout<<"written.."<<std::endl; 
	free(x);
    free(y);
	free(z);
	free(r);
	free(g);
    free(b);
    
	return EXIT_SUCCESS; 	
}
