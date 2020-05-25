// This is the real Hello World for CUDA!
//It takes the string 'Hello', prints it, then passes it to cuda with an array of offsets
// Then the offsets are added in parallel to produce the string world! 
#include <iostream>
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

// =========================================================================================================================
// This class file is to transport into include directory after development 

class Key3D{
	
private: 
	float x; 
	float y; 
	float z; 

public: 
	//Null constructor
	__host__ __device__ Key3D(){} 

	//preferred constructor
	__host__ __device__ Key3D(float x_in, float y_in, float z_in){
		x = x_in; 
		y = y_in; 
		z = z_in;  
	}

	//retrieve values for testing
	__host__ __device__ const float& getX() const {
		return x; 
	}
	__host__ __device__ const float& getY() const {
		return y; 
	}
	__host__ __device__ const float& getZ() const {
		return z; 
	}

	//the sorting operator for a map
	bool operator< (const Key3D& keyObj) const{

        if(keyObj.x < this->x){
			return true;
		} 
		return false;         
    }

	//destructor
	__host__ __device__ ~Key3D(){} 

}; 

const float epsilon = 0.0125; // epsilon to compare two floats. this value depends on the resolution we consider. (resolution /4 or lower )

struct VoxelKeyComparator
{	
	bool operator()(const Key3D &left, const Key3D &right) const{
		
		if((abs(left.getX()-right.getX()) > epsilon) || (abs(left.getY()-right.getY()) > epsilon) || (abs(left.getZ()-right.getZ()) > epsilon)){
			return (left.getX() <= right.getX());
			//return true; 
		}
		return false; 
	}
};

// This class is originally developed to generate within host. updating count parameter with device might require extra effort. 
class VoxelBody{		
private: 
	int point_count; 
	uint8_t r; 
	uint8_t g; 
	uint8_t b; 

public: 
	//Null constructor

	__host__ __device__ VoxelBody(){
		point_count = 0; 
		r = 0; 
		g = 0; 
		b = 0; 
	} 

	//full constructor
	__host__ __device__ VoxelBody(uint8_t r_in, uint8_t g_in, uint8_t b_in){
		point_count = 1; 	
		r = r_in; 
		g = g_in; 
		b = b_in; 						
	}
	// desctructor
	__host__ __device__ ~VoxelBody(){} 

	__host__ __device__ void updateVoxel(uint8_t r_in, uint8_t g_in, uint8_t b_in){
		point_count += 1; 
		/*if(r == 0 and g == 0 and b == 0){		// a previously mis initialized points
			r = r_in; 
			g = g_in; 
			b = b_in;
		}else{*/								// average the value within the voxel 
			r = (r + r_in)/2; 
			g = (g + g_in)/2; 
			b = (b + b_in)/2;
		//}
	}
	__host__ __device__ void addNewPoint(){
		point_count +=1; 
	}

	__host__ __device__ const uint8_t& get_r() const { return r; }
	__host__ __device__ const uint8_t& get_g() const { return g; }
	__host__ __device__ const uint8_t& get_b() const { return b; }
	__host__ __device__ const int& get_point_count() const {return point_count; }
	
};

// ==============================================================================================================================

using namespace std; 

#define THREADS_PER_BLOCK 256		// the optimal value is number of cuda cores, if (#cuda cores < max th.pr.blck). 256 for TX2
static int NUM_OF_BLOCKS = 1; 

__device__ const float resolution = 0.05f; 	// Resolution of 5 cm
__device__ const float half_resolution = resolution/2; // the half of the resolution. this value is used in calculations 
 
const float max_sensor_radius = 3.00f; 	// scan data further than this modulus value will not be considered. 
 /*
__global__ void create_point_cloud(float* x, float* y, float* z, uint8_t* r, uint8_t* g, uint8_t* b, Color3DPoint* output_array){

	int index = threadIdx.x + blockIdx.x * blockDim.x;
	int count = 0; // how many effective points are there. at the end of processing vectors, we can use this variable to allocate memory for the effective points array 
	if( (fabs(x[index]) < max_sensor_radius) and (fabs(y[index]) < max_sensor_radius) and (fabs(z[index]) < max_sensor_radius)){
		count +=1; 
		float *x_temp = &x[index]; 
		float *y_temp = &y[index]; 
		float *z_temp = &z[index]; 
		
		printf("%f\t%f\t%f\n", &x_temp, &y_temp, &z_temp); 
		delete x_temp; delete y_temp; delete z_temp; 
		
		Color3DPoint temp_point = Color3DPoint( *x_temp, 	
			*y_temp, 
			*z_temp, 
			r[index], 
			g[index], 
			b[index]); 


		output_array[index] = temp_point; 
		temp_point = Color3DPoint();

		
	}else{
		output_array[index] = Color3DPoint(0, 0, 0, 0, 0, 0); 
	}
	
	
}
*/
__global__ void round_off_positional_coords(float* x, float* y, float* z, float* x_result, float* y_result, float* z_result, 
	float x_trans, float y_trans, float z_trans, double sin_a, double sin_b, double sin_g, double cos_a, double cos_b, double cos_g){
	
	int index = threadIdx.x + blockIdx.x * blockDim.x;
	
	 
	if( (fabs(x[index]) < max_sensor_radius) and (fabs(y[index]) < max_sensor_radius) and (fabs(z[index]) < max_sensor_radius)){
				
		//B: Transformation model 1 -----------yaw only----------- for flat surface navigation-----------------------------------
		/*float x_temp = x[index]*cos_a - y[index]*sin_a + x_trans;  
		float y_temp = x[index]*sin_a + y[index]*cos_a + y_trans; 
		float z_temp = z[index] + z_trans; 
		
		x_result[index] = (ceilf(x_temp / resolution))*0.05 - half_resolution;
		y_result[index] = (ceilf(y_temp / resolution))*0.05 - half_resolution; 
		z_result[index] = (ceilf(z_temp / resolution))*0.05 - half_resolution; */
		//------------------------------------------------------------------------------------------------
		
		// C: Transformation model 2 ---------- Roll, pitch, yaw combined--------- for inclined planes navigation --------------
		float x_temp = x[index]*cos_a*cos_b + y[index]*cos_a*sin_b*sin_g - y[index]*sin_a*cos_g + z[index]*cos_a*sin_b*cos_g + z[index]*sin_a*sin_g + x_trans;  
		float y_temp = x[index]*sin_a*cos_b + y[index]*sin_a*sin_b*sin_g + y[index]*cos_a*cos_g + z[index]*sin_a*sin_b*cos_g - z[index]*cos_a*sin_g + y_trans; 
		float z_temp = x[index]*sin_b*-1 + y[index]*cos_b*sin_g + z[index]*cos_b*cos_g + z_trans; 
		
		x_result[index] = (ceilf(x_temp / resolution))*0.05 - half_resolution;
		y_result[index] = (ceilf(y_temp / resolution))*0.05 - half_resolution; 
		z_result[index] = (ceilf(z_temp / resolution))*0.05 - half_resolution;

		// -----------------------------------------------------------------------------------------------------------------------------------------
		
	}else{
		x_result[index] = 0.00f; 
		y_result[index] = 0.00f; 
		z_result[index] = 0.00f; 

	} 
	
}


void serial_remove_null_points(int camera_array_size, float* x, float* y, float* z, uint8_t* r, uint8_t* g, uint8_t* b,
													float* x_cropped, float* y_cropped, float* z_cropped, uint8_t* r_cropped, uint8_t* g_cropped, uint8_t* b_cropped){
	// x, y, z are input arrays with rounded and range controlled poitn clouds
	// x_cropped etc. stores the new array with 0,0,0 points removed. 
	int index = 0; 
	for(int i=0; i<camera_array_size; i++){			// size of original set of point cloud
		if(x[i] != 0.00f){
			x_cropped[index] = x[i]; 
			y_cropped[index] = y[i]; 
			z_cropped[index] = z[i]; 
			r_cropped[index] = r[i]; 
			g_cropped[index] = g[i]; 
			b_cropped[index] = b[i]; 
			
			index +=1; 
		}
	}
}

static std::map<Key3D, VoxelBody, VoxelKeyComparator> voxel_map_downsampled; 

void serial_downsample_point_cloud(int array_size, float* x_cleaned, float* y_cleaned, float* z_cleaned, uint8_t* r_cleaned, uint8_t* g_cleaned, uint8_t* b_cleaned){

	Key3D temp_key; 
	VoxelBody temp_body; 
	
	for(int i=0; i<array_size; i++){
		temp_key = Key3D(x_cleaned[i], y_cleaned[i], z_cleaned[i]);
		temp_body = VoxelBody(r_cleaned[i], g_cleaned[i], b_cleaned[i]); 
		// Keep inerting for now. will genaralize rgb values and count later. 
		//voxel_map_downsampled.insert(std::make_pair<Key3D, VoxelBody>(temp_key, temp_body));
		std::map<Key3D, VoxelBody, VoxelKeyComparator>::iterator it; 
		it = voxel_map_downsampled.find(temp_key); 
		if(it !=voxel_map_downsampled.end()){
			it->second.updateVoxel(r_cleaned[i], g_cleaned[i], b_cleaned[i]); 
		}else{
			voxel_map_downsampled.insert(std::make_pair<Key3D, VoxelBody>(temp_key, temp_body));
		}
	}
	
}


int downsampled_count = 0; 
void readings_function(int array_size, float* x_cleaned, float* y_cleaned, float* z_cleaned, uint8_t* r_cleaned, uint8_t* g_cleaned, uint8_t* b_cleaned, 
							std::map<Key3D, VoxelBody, VoxelKeyComparator> map_reading){

	Key3D temp_key; 
	VoxelBody temp_body; 
	
	for(int i=0; i<array_size; i++){
		temp_key = Key3D(x_cleaned[i], y_cleaned[i], z_cleaned[i]);
		temp_body = VoxelBody(r_cleaned[i], g_cleaned[i], b_cleaned[i]); 
		// Keep inerting for now. will genaralize rgb values and count later. 
		map_reading.insert(std::make_pair<Key3D, VoxelBody>(temp_key, temp_body));
	}
	downsampled_count =0; 
	
	std::map<Key3D, VoxelBody, VoxelKeyComparator>::iterator itr; 
	for(itr=map_reading.begin(); itr!=map_reading.end(); itr++){
		downsampled_count +=1;   
	}

}


int cudamain(sensor_msgs::PointCloud2 point_cloud_std, nav_msgs::Odometry odom_message_std, int size){ 
	//make_range_array(resolution, max_sensor_radius); 
	int array_size = size; 	 

	// convert quaternion orientation into roll, pith, yaw representation 
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

	
	//std::cout<<"alpha: "<<yaw<<"  sin alpha: "<<sin_alpha<<std::endl;
	//std::cout<<"\n\n"; 

	int counter = 0; 
	int effective_point_count = 0; 
	//declare the arrray sets before reading the point cloud values 
	
	float *x, *y, *z; // for allocating position values of the points 
	float *x_rounded, *y_rounded, *z_rounded; 							// the intermediate results after rounding off the x, y, z, original values to the resolution 
	u_int8_t *r, *g, *b; // for color values of the point cloud 
	

	int size_position = array_size * sizeof(float);
	int size_color = array_size * sizeof(u_int8_t);
	
	
	x = (float *)malloc( size_position );
   	y = (float *)malloc( size_position );
	z = (float *)malloc( size_position );

	x_rounded = (float *)malloc( size_position );
   	y_rounded = (float *)malloc( size_position );
	z_rounded = (float *)malloc( size_position );

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
	
	// Adjust the number of blocks to be a whole number. 
	NUM_OF_BLOCKS = (array_size + THREADS_PER_BLOCK-1)/THREADS_PER_BLOCK; 


	//The cuda device variables 
	float *d_x, *d_y, *d_z;
	float *d_x_rounded, *d_y_rounded, *d_z_rounded; 
	
	cudaMalloc( (void **) &d_x, size_position);
	cudaMalloc( (void **) &d_y, size_position);
	cudaMalloc( (void **) &d_z, size_position);
	
	cudaMalloc( (void **) &d_x_rounded, size_position);
	cudaMalloc( (void **) &d_y_rounded, size_position);
	cudaMalloc( (void **) &d_z_rounded, size_position);

    cudaMemcpy( d_x, x, size_position, cudaMemcpyHostToDevice );
	cudaMemcpy( d_y, y, size_position, cudaMemcpyHostToDevice );
	cudaMemcpy( d_z, z, size_position, cudaMemcpyHostToDevice );
	
	// GPU process START---------------------------------------------------------------------------------------------------------------------------------------------------------------
	//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	double startc, endc; 
	startc = clock();
	round_off_positional_coords<<<NUM_OF_BLOCKS, THREADS_PER_BLOCK>>>(d_x, d_y, d_z, d_x_rounded, d_y_rounded, d_z_rounded,
									x_position, y_position, z_position, sin_alpha, sin_beta, sin_gamma, cos_alpha, cos_beta, cos_gamma);
	endc = clock();
	double time_parallel = (double)(endc-startc); 
	std::cout<<"Time to generate voxel array: "<<time_parallel<<std::endl; 

	cudaMemcpy( x_rounded, d_x_rounded, size_position, cudaMemcpyDeviceToHost );
	cudaMemcpy( y_rounded, d_y_rounded, size_position, cudaMemcpyDeviceToHost );
	cudaMemcpy( z_rounded, d_z_rounded, size_position, cudaMemcpyDeviceToHost );

	
	// GPU process END----------------------------------------------------------------------------------------------------------------------------------------------------------------
	//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	
	
	for(int i=0; i<array_size; i++){		// counted the number of informative points 
		if(x_rounded[i] != 0.00f){
			effective_point_count += 1; 
		}
	}

	float *x_cleaned, *y_cleaned, *z_cleaned; 
	uint8_t *r_cleaned, *g_cleaned, *b_cleaned; 

	int size_cleaned = effective_point_count * sizeof(float);
	int size_colour_cleaned = effective_point_count * sizeof(uint8_t); 
	
	x_cleaned = (float *)malloc( size_cleaned );
	y_cleaned = (float *)malloc( size_cleaned );
	z_cleaned = (float *)malloc( size_cleaned );
	r_cleaned = (uint8_t *)malloc(size_colour_cleaned); 
	g_cleaned = (uint8_t *)malloc(size_colour_cleaned); 
	b_cleaned = (uint8_t *)malloc(size_colour_cleaned); 

	serial_remove_null_points(array_size, x_rounded, y_rounded, z_rounded, r, g, b, 
							x_cleaned, y_cleaned, z_cleaned, r_cleaned, g_cleaned, b_cleaned);  
	
	serial_downsample_point_cloud(effective_point_count, x_cleaned, y_cleaned, z_cleaned, r_cleaned, g_cleaned, b_cleaned); 
	
	
	
	std::string map_file_name = "result_map_key_generation.txt";
	ofstream offile;
	offile.open(map_file_name.c_str(), ios::trunc);
	if(offile.is_open()) { 
		std::map<Key3D, VoxelBody, VoxelKeyComparator>::iterator itr; 
		for(itr=voxel_map_downsampled.begin(); itr!=voxel_map_downsampled.end(); itr++){
			offile<<itr->first.getX()<<"\t"<<itr->first.getY()<<"\t"<<itr->first.getZ()<<"\t"<<unsigned(itr->second.get_r())<<"\t"<<unsigned(itr->second.get_g())<<"\t"<<unsigned(itr->second.get_b())<<std::endl;  
		}
		offile<<"\n\n"; 
		//std::cout<<"data written"<<std::endl; 											
								
	}
	offile.close();
	 
	free(x);
    free(y);
	free(z);
	free(r);
	free(g);
	free(b);
	free(x_rounded); 
	free(y_rounded); 
	free(z_rounded);
	
	free(x_cleaned); 
	free(y_cleaned); 
	free(z_cleaned); 
	free(r_cleaned); 
	free(g_cleaned); 
	free(b_cleaned); 

	cudaFree( d_x );
	cudaFree( d_y );
	cudaFree( d_z );
	cudaFree(d_x_rounded); 
	cudaFree(d_y_rounded); 
	cudaFree(d_z_rounded); 
	
	return EXIT_SUCCESS; 	
}
