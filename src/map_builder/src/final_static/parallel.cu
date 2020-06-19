//K. T. D. S. De Silva ---- University of Moratuwa 
// CUDA processing module 
// Static/obstacle only model of unknown environment 
//-------------------------------------------------------------------------------------------------- 

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
#include <boost/unordered_map.hpp>
#include <iterator>
#include <boost/lexical_cast.hpp>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include "geometry_msgs/Quaternion.h"


using namespace std; 

#define THREADS_PER_BLOCK 256		// the optimal value is number of cuda cores, if (#cuda cores < max th.pr.blck). 256 for TX2
static int NUM_OF_BLOCKS = 1; 
__device__ const int axis_length = 65536; // 2^16; 
__device__ const int half_axis_length = 32768; 
__device__ const float resolution = 0.04f; 	// Resolution of 5 cm
__device__ const float half_resolution = resolution/2; // the half of the resolution. this value is used in calculations 
 
const float max_sensor_radius = 3.00f; 	// scan data further than this modulus value will not be considered. 
 
// x, y, z, r, g, b input arrays 
// sin, cos, tan rpy are input variables 
// x_result, y_result, z_result, rgb_result are outputs 
__global__ void generate_codes(float* x, float* y, float* z, uint8_t* r, uint8_t* g, uint8_t* b, 
    uint64_t* morton_code, uint32_t* rgb_result, 
	float x_trans, float y_trans, float z_trans, double sin_a, double sin_b, double sin_g, double cos_a, double cos_b, double cos_g){
	
	int index = threadIdx.x + blockIdx.x * blockDim.x;
	
	if( (fabs(x[index]) < max_sensor_radius) and (fabs(y[index]) < max_sensor_radius) and (fabs(z[index]) < max_sensor_radius)){
		
		// C: Transformation model 2 ---------- Roll, pitch, yaw combined--------- for inclined planes navigation --------------
		float x_tm = x[index]*cos_a*cos_b + y[index]*cos_a*sin_b*sin_g - y[index]*sin_a*cos_g + z[index]*cos_a*sin_b*cos_g + z[index]*sin_a*sin_g + x_trans;  
		float y_tm = x[index]*sin_a*cos_b + y[index]*sin_a*sin_b*sin_g + y[index]*cos_a*cos_g + z[index]*sin_a*sin_b*cos_g - z[index]*cos_a*sin_g + y_trans; 
		float z_tm = x[index]*sin_b*-1 + y[index]*cos_b*sin_g + z[index]*cos_b*cos_g + z_trans; 
        
        // convert the float into an unsigned integer 
        uint64_t x_original = uint64_t(half_axis_length + ceilf(x_tm / resolution) -1 );
        uint64_t y_original = uint64_t(half_axis_length + ceilf(y_tm / resolution) -1 );
        uint64_t z_original = uint64_t(half_axis_length + ceilf(z_tm / resolution) -1 );
        
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
        
        morton_code[index] = m_code; 

        // calculate rgbo value. 
        uint32_t r_temp = r[index] & 0b00000000000000000000000011111111; 
        uint32_t g_temp = g[index] & 0b00000000000000000000000011111111; 
        uint32_t b_temp = b[index] & 0b00000000000000000000000011111111; 
        uint32_t rgb_code = (r_temp<<24) | (g_temp<<16) | (b_temp<<8) | 0b00000000000000000000000000010001; // occ for obstcale nodes 
		rgb_result[index] = rgb_code; 
	}else{
        //printf("else situation"); 
        morton_code[index] = 0b0000000000000000100011011011011011011011011011011011011011011011; // index for mid point of environment.  
        rgb_result[index] = 0b00000000000000000000000000000000; 
	} 
}

__global__ void decode_codes(uint64_t* mcode, uint32_t* value, 
            uint64_t* x_out, uint64_t* y_out, uint64_t* z_out, uint8_t* r_out, uint8_t* g_out, uint8_t* b_out, uint8_t* occ_out){
	
	int index = threadIdx.x + blockIdx.x * blockDim.x;
    
    uint64_t code = mcode[index]; 
    uint32_t rgbo = value[index]; 

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
                   
    

    uint8_t r_final = uint8_t(rgbo>>24 & 0b00000000000000000000000011111111);
    uint8_t g_final = uint8_t(rgbo>>16 & 0b00000000000000000000000011111111);
    uint8_t b_final = uint8_t(rgbo>>8 & 0b00000000000000000000000011111111); 
    uint8_t o_final = uint8_t(rgbo & 0b00000000000000000000000011111111); 

    x_out[index] = x_int; 
    y_out[index] = y_int; 
    z_out[index] = z_int; 
    r_out[index] = r_final;
    g_out[index] = g_final;
    b_out[index] = b_final;
    occ_out[index] = o_final; 
    
}
int n=1; 
extern boost::unordered::unordered_map<uint64_t, uint32_t> octree;

int cudamain(sensor_msgs::PointCloud2 point_cloud_std, nav_msgs::Odometry odom_message_std){ 
    
    double starttotal, endtotal; 
	starttotal = clock();
	
	//make_range_array(resolution, max_sensor_radius); 
	int array_size = 640*480;  	 
    
    // convert quaternion orientation into roll, pitch, yaw representation 
	//double roll, pitch, yaw;
	double roll, pitch, yaw; 
	tf::Quaternion quat;
    tf::quaternionMsgToTF(odom_message_std.pose.pose.orientation, quat);	
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
	float x_position = (float) odom_message_std.pose.pose.position.x; 
	float y_position = (float) odom_message_std.pose.pose.position.y; 
	float z_position = (float) odom_message_std.pose.pose.position.z; 

	double sin_gamma = sin(roll); 
	double sin_beta = sin(pitch); 
	double sin_alpha = sin(yaw); 
	double cos_gamma = cos(roll); 
	double cos_beta = cos(pitch); 
	double cos_alpha = cos(yaw); 

	int counter = 0; 
	int effective_point_count = 0; 
	//declare the arrray sets before reading the point cloud values 
	
	float *x, *y, *z; // for allocating position values of the points 
	uint64_t *mcode_arr; 	// the intermediate results after rounding off the x, y, z, original values to the resolution 
	uint8_t *r, *g, *b; // for color values of the point cloud 
	uint32_t *rgb; 

	int size_position = array_size * sizeof(float);
	int size_color = array_size * sizeof(uint8_t);
    int size_morton_array = array_size * sizeof(uint64_t);
    int size_rgb = array_size * sizeof(uint32_t);
    
	x = (float *)malloc( size_position );
   	y = (float *)malloc( size_position );
	z = (float *)malloc( size_position );
    
    
	mcode_arr = (uint64_t *)malloc( size_morton_array );
   	
	r = (uint8_t *)malloc( size_color );
    g = (uint8_t *)malloc( size_color );
	b = (uint8_t *)malloc( size_color );
    
    rgb = (uint32_t *)malloc( size_rgb );
    
    double start1, end1; 
	start1 = clock();
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

    end1 = clock();
	double time1 = (double)(end1 - start1);
    
	// Adjust the number of blocks to be a whole number. 
	NUM_OF_BLOCKS = (array_size + THREADS_PER_BLOCK-1)/THREADS_PER_BLOCK; 

	//The cuda device variables 
    float *d_x, *d_y, *d_z;
    uint8_t *d_r, *d_g, *d_b; 
	uint64_t *d_mcode_arr; 
    uint32_t *d_rgb; 
    
	cudaMalloc( (void **) &d_x, size_position);
	cudaMalloc( (void **) &d_y, size_position);
    cudaMalloc( (void **) &d_z, size_position);
    
    cudaMalloc( (void **) &d_r, size_color);
	cudaMalloc( (void **) &d_g, size_color);
	cudaMalloc( (void **) &d_b, size_color);
    
    // the kernel outputs
	cudaMalloc( (void **) &d_mcode_arr, size_morton_array);
	cudaMalloc( (void **) &d_rgb, size_rgb);

    cudaMemcpy( d_x, x, size_position, cudaMemcpyHostToDevice );
	cudaMemcpy( d_y, y, size_position, cudaMemcpyHostToDevice );
    cudaMemcpy( d_z, z, size_position, cudaMemcpyHostToDevice );
    
    cudaMemcpy( d_r, r, size_color, cudaMemcpyHostToDevice );
    cudaMemcpy( d_g, g, size_color, cudaMemcpyHostToDevice );
    cudaMemcpy( d_b, b, size_color, cudaMemcpyHostToDevice );
	
	// GPU process START---------------------------------------------------------------------------------------------------------------------------------------------------------------
	//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	double start2, end2; 
	start2 = clock();
    
    generate_codes<<<NUM_OF_BLOCKS, THREADS_PER_BLOCK>>>(d_x, d_y, d_z, d_r, d_g, d_b, 
                                    d_mcode_arr, d_rgb, 
									x_position, y_position, z_position, sin_alpha, sin_beta, sin_gamma, cos_alpha, cos_beta, cos_gamma);
	
    end2 = clock();
    double time2 = (double)(end2 - start2);
                                    
	cudaMemcpy( mcode_arr, d_mcode_arr, size_morton_array, cudaMemcpyDeviceToHost );
    cudaMemcpy( rgb, d_rgb, size_rgb, cudaMemcpyDeviceToHost );
	
	// GPU process END----------------------------------------------------------------------------------------------------------------------------------------------------------------
	//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    double start3, end3; 
    start3 = clock();
    
    // add into the octree 
    for(int i=0; i<array_size; i++){
        // Morton code for null points: 0b0000000000000000100011011011011011011011011011011011011011011011;
        uint64_t morton_code = mcode_arr[i]; 
        //search for the above code in octree
        if(morton_code != 0b0000000000000000100011011011011011011011011011011011011011011011){
            boost::unordered::unordered_map<uint64_t, uint32_t>::iterator itr;  // to find the key 
            itr = octree.find(morton_code);
            if (itr == octree.end()){
                uint32_t value = rgb[i]; 
                octree.insert(std::make_pair<uint64_t, uint32_t>(morton_code, value));
                
            }else{
                uint32_t rgbo_map = (itr->second); 
                uint32_t occ_old = rgbo_map & 0b00000000000000000000000000111111;
                rgbo_map = rgb[i] & 0b11111111111111111111111100000000;             // this is the final variable 
                uint32_t occ_new = rgb[i] & 0b00000000000000000000000000111111;

                if(occ_old == 0b00000000000000000000000000001000){
                    // previous value is unkown
                    rgbo_map = rgbo_map | 0b00000000000000000000000000010001; 
                }else if(occ_old < 0b00000000000000000000000000100000){
                    // value less than 32. max occupancy 
                    occ_old +=1; 
                    rgbo_map = rgbo_map | occ_old; 
                }else{
                    
                }
                itr->second = rgbo_map; 
            }
        }    
    }		
    end3 = clock();
    double time3 = (double)(end3 - start3);
	free(x);
    free(y);
	free(z);
	free(r);
	free(g);
	free(b);
	free(mcode_arr); 
	free(rgb);
    
	cudaFree( d_x );
	cudaFree( d_y );
    cudaFree( d_z );
    cudaFree( d_r );
    cudaFree( d_g );
    cudaFree( d_b );
	cudaFree(d_mcode_arr); 
	cudaFree(d_rgb); 
    
    endtotal = clock();
    double timetotal = (double)(endtotal - starttotal);
    
    std::cout<<n<<"\t"<<time1<<"\t"<<time2<<"\t"<<time3<<"\t"<<timetotal<<endl; n+=1; 

	return EXIT_SUCCESS; 	
}
