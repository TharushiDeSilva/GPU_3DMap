#include "iostream"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_ros/transforms.h>
#include <fstream>
#include "sensor_msgs/PointCloud2.h"
#include <sensor_msgs/PointCloud.h>
#include <time.h>
#include <string>
#include "math.h"
#include <stdio.h>
#include <map>
#include <iterator>
#include <boost/lexical_cast.hpp>

class Key3D{
	
private: 
	float x; 
	float y; 
	float z; 

public: 
	//Null constructor
	Key3D(){} 

	//preferred constructor
	Key3D(float x_in, float y_in, float z_in){
		x = x_in; 
		y = y_in; 
		z = z_in;  
	}

	//retrieve values for testing
	const float& getX() const {
		return x; 
	}
	const float& getY() const {
		return y; 
	}
	const float& getZ() const {
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
	~Key3D(){} 

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


class VoxelBody{		
private: 
	int point_count; 
	uint8_t r; 
	uint8_t g; 
	uint8_t b; 

public: 
	//Null constructor

	VoxelBody(){
		point_count = 0; 
		r = 0; 
		g = 0; 
		b = 0; 
	} 

	//full constructor
	VoxelBody(uint8_t r_in, uint8_t g_in, uint8_t b_in){
		point_count = 1; 	
		r = r_in; 
		g = g_in; 
		b = b_in; 						
	}
	// desctructor
	~VoxelBody(){} 

	void updateVoxel(uint8_t r_in, uint8_t g_in, uint8_t b_in){
		point_count += 1; 
		if(r == 0 and g == 0 and b == 0){		// a previously mis initialized points
			r = r_in; 
			g = g_in; 
			b = b_in;
		}else{								// average the value within the voxel 
			r = (r + r_in)/2; 
			g = (g + g_in)/2; 
			b = (b + b_in)/2;
		}
	}

	const uint8_t& get_r() const { return r; }
	const uint8_t& get_g() const { return g; }
	const uint8_t& get_b() const { return b; }
	const int& get_point_count() const {return point_count; }
	
};


int array_size = 0; // the original size of point cloud (this is always 307200 for Microsoft Kinect for XBox360)

// float *x, *y, *z; // for allocating position values of the points 
// u_int8_t *r, *g, *b; // for color values of the point cloud 
float *x_global, *y_global, *z_global; // for allocating position values of the points 
u_int8_t *r_global, *g_global, *b_global; // for color values of the point cloud 

float *x_rounded, *y_rounded, *z_rounded; 

int effective_point_count = 0; 	// how many not null points are there in the point cloud 
static std::map<Key3D, VoxelBody, VoxelKeyComparator> voxel_map_downsampled; 

static int size_position = 0; 
static int size_color = 0; 

using namespace std; 
using namespace pcl; 
//using namespace octomap; 

int cudamain(float *x, float *y, float *z, int array_size, int size_position); 	// cuda main function declaration. definition is in template.cu file 
float *get_x_rounded();
float *get_y_rounded(); 
float *get_z_rounded(); 


//time_t start_main, end_main, start_callback, end_callback;
 
static int x1=0; 
void cloud_callback_temp(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input){		// used to extract PointCloud2 and number of points 
	x1+=1; 
	sensor_msgs::PointCloud2 in_cloud_2 = *input; 
	sensor_msgs::PointCloud cloud_converted; 
   	sensor_msgs::convertPointCloud2ToPointCloud(in_cloud_2, cloud_converted); 
   	array_size = cloud_converted.points.size();
	
	float *x, *y, *z; 
	uint8_t *r, *g, *b;
	x = new float[array_size]; 
	y = new float[array_size]; 
	z = new float[array_size]; 
	r = new uint8_t[array_size]; 
	g = new uint8_t[array_size]; 
	b = new uint8_t[array_size]; 

	int counter = 0; 
	for(sensor_msgs::PointCloud2ConstIterator<float> it(in_cloud_2, "x"); it!=it.end(); ++it){
		x[counter] = it[0]; 
		y[counter] = it[1]; 
		z[counter] = it[2];
		counter+=1;  
		 
	}
	counter = 0; 
	for(sensor_msgs::PointCloud2ConstIterator<u_int8_t> it_color(in_cloud_2, "rgb"); it_color!=it_color.end(); ++it_color){
		r[counter] = unsigned(it_color[0]);	
		g[counter] = unsigned(it_color[1]);	
		b[counter] = unsigned(it_color[2]); 
		counter+=1; 
	}
	counter = 0; 
	// memcpy(x_global, x, size_position); 
	// memcpy(y_global, y, size_position); 
	// memcpy(z_global, z, size_position); 

	// memcpy(r_global, r, size_color);
	// memcpy(g_global, g, size_color);
	// memcpy(b_global, b, size_color);
	
	for(int i=0; i<array_size; i++){
		*(x_global+i) = x[i];  
	}
	std::cout<<x[307163]<<"\t"<<x_global[307163]<<std::endl; 
	delete[] x; 
	delete[] y; 
	delete[] z; 
	delete[] r; 
	delete[] g; 
	delete[] b; 
	
	// add the points to the global array
	// end_callback = clock(); 
	// double timestamp_callback = double(end_callback - start_callback); 
	
	array_size =0; 
	 
}

void cloud_callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input){		// used to extract PointCloud2 and number of points 
	x1+=1; 
	sensor_msgs::PointCloud2 in_cloud_2 = *input; 
	sensor_msgs::PointCloud cloud_converted; 
   	sensor_msgs::convertPointCloud2ToPointCloud(in_cloud_2, cloud_converted); 
   	array_size = cloud_converted.points.size();
	
	float *x, *y, *z; 
	uint8_t *r, *g, *b;
	x = new float[array_size]; 
	y = new float[array_size]; 
	z = new float[array_size]; 
	r = new uint8_t[array_size]; 
	g = new uint8_t[array_size]; 
	b = new uint8_t[array_size]; 

	int counter = 0; 
	for(sensor_msgs::PointCloud2ConstIterator<float> it(in_cloud_2, "x"); it!=it.end(); ++it){
		x[counter] = it[0]; 
		y[counter] = it[1]; 
		z[counter] = it[2];
		counter+=1;  
		 
	}
	counter = 0; 
	for(sensor_msgs::PointCloud2ConstIterator<u_int8_t> it_color(in_cloud_2, "rgb"); it_color!=it_color.end(); ++it_color){
		r[counter] = unsigned(it_color[0]);	
		g[counter] = unsigned(it_color[1]);	
		b[counter] = unsigned(it_color[2]); 
		counter+=1; 
	}
	counter = 0; 
	
	std::cout<<x1<<"\t"<<array_size<<"\t"<<x[307163]<<x_rounded[307163]<<std::endl; 
	delete[] x; 
	delete[] y; 
	delete[] z; 
	delete[] r; 
	delete[] g; 
	delete[] b; 

	free(x_rounded); 
	free(y_rounded); 
	free(z_rounded); 
	array_size =0; 

	 
}


int calculate_effective_point_count(int original_size, float *array){
	int count = 0; 
	for(int i=0; i<original_size; i++){		// counted the number of informative points 
		if(array[i] != 0.00f){
			count += 1; 
		}
	}
	return count; 
} 

void remove_null_points(int array_size_in, float *x_in, float *y_in, float *z_in, uint8_t *r_in, uint8_t *g_in, uint8_t *b_in, 
													float *x_out, float *y_out, float *z_out, uint8_t *r_out, uint8_t *g_out, uint8_t *b_out){
	// x, y, z are input arrays with rounded and range controlled point clouds
	// x_cropped etc. stores the new array with 0,0,0 points removed. 
	int index = 0; 
	for(int i=0; i<array_size_in; i++){			// size of original set of point cloud
		if(x_in[i] != 0.00f){
			x_out[index] = x_in[i]; 
			y_out[index] = y_in[i]; 
			z_out[index] = z_in[i]; 
			r_out[index] = r_in[i]; 
			g_out[index] = g_in[i]; 
			b_out[index] = b_in[i]; 
			index +=1; 
		}
	}
}

void downsample_point_cloud(int array_size_in, float* x_in, float* y_in, float* z_in, uint8_t* r_in, uint8_t* g_in, uint8_t* b_in){

	Key3D temp_key; 
	VoxelBody temp_body; 
	
	for(int i=0; i<array_size; i++){
		temp_key = Key3D(x_in[i], y_in[i], z_in[i]);
		temp_body = VoxelBody(r_in[i], g_in[i], b_in[i]); 
		// Keep inerting for now. will genaralize rgb values and count later. 
		voxel_map_downsampled.insert(std::make_pair<Key3D, VoxelBody>(temp_key, temp_body));
	}
	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "occupancy_map_builder");	  
	// start_main = clock(); 
	// start_callback = clock();  
	
	//allocate memory 
	size_position = array_size * sizeof(float);
	size_color = array_size * sizeof(u_int8_t);
	//allocate memory once

	// x_global = (float *)malloc( size_position );
   	// y_global = (float *)malloc( size_position );
	// z_global = (float *)malloc( size_position );
	// r_global = (uint8_t *)malloc(size_color); 
	// g_global = (uint8_t *)malloc(size_color); 
	// b_global = (uint8_t *)malloc(size_color); 

	x_rounded = (float *)malloc( size_position );
	y_rounded = (float *)malloc( size_position );
	z_rounded = (float *)malloc( size_position );

	ros::NodeHandle nh;
	//ros::Publisher laser_pub = n.advertise<sensor_msgs::LaserScan>("/laserscan/filtered", 1);  // write octomap publisher here
	ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", 5, cloud_callback);
 
	while(ros::ok()){  
		 
		ros::spinOnce();
		// end_main = clock(); 
		// double time_stamp_main = double(end_main - start_main); 
		//cudamain(x_global, y_global, z_global, array_size, size_position); 
		/*

		// calling the cuda process 
		cudamain(x, y, z, array_size, size_position); 

		// get the results of cuda function 
		// x_rounded = get_x_rounded(); 
		// y_rounded = get_y_rounded(); 
		// z_rounded = get_z_rounded(); 
		x = get_x_rounded(); 
		y = get_y_rounded(); 
		z = get_z_rounded(); 

		//effective_point_count = calculate_effective_point_count(array_size, x_rounded); 
		effective_point_count = calculate_effective_point_count(array_size, x); 

		// remove null data points from x, y, z, r, g, b 
		//---------------------------------------------------------------------------------------------------------------------------------
		//---------------------------------------------------------------------------------------------------------------------------------
		float *x_final, *y_final, *z_final; 		// pointers for final position arrays 
		uint8_t *r_final, *g_final, *b_final; 		//pointers to final colour arrays 
		
		//allocate memory for arrays
		x_final = new float[effective_point_count]; 
		y_final = new float[effective_point_count]; 
		z_final = new float[effective_point_count]; 
		r_final = new uint8_t[effective_point_count]; 
		g_final = new uint8_t[effective_point_count]; 
		b_final = new uint8_t[effective_point_count]; 

		int index = 0; 
		for(int i=0; i<array_size; i++){			
			if(x[i] != 0.00f){
				x_final[index] = x[i]; 
				y_final[index] = y[i]; 
				z_final[index] = z[i]; 
				r_final[index] = r[i]; 
				g_final[index] = g[i]; 
				b_final[index] = b[i]; 
				index +=1; 
			}
		}

	//------------------------------------------------------------------------------------------------------------------------------------
	//------------------------------------------------------------------------------------------------------------------------------------
	// Downsample the point cloud and add entries to the map 
	//downsample_point_cloud(effective_point_count, x_cleaned, y_cleaned, z_cleaned, r_cleaned, g_cleaned, b_cleaned); 
		Key3D *temp_key; 				//pointer to store the most recent key 
		VoxelBody *temp_body; 			//Pointer to store most recent value
		
		for(int i=0; i<effective_point_count; i++){
			*temp_key = Key3D(x_final[i], y_final[i], z_final[i]);
			*temp_body = VoxelBody(r_final[i], g_final[i], b_final[i]); 
			// Keep inerting for now. will genaralize rgb values and count later. 
			voxel_map_downsampled.insert(std::make_pair<Key3D, VoxelBody>(*temp_key, *temp_body));
			free (temp_key); 
			free (temp_body); 
		}	
		

		delete[] x_final; 
		delete[] y_final; 
		delete[] z_final; 
		delete[] r_final; 
		delete[] g_final; 
		delete[] b_final; 
		*/
		
		// free(r); 
		// free(g); 
		// free(b);  
		
	}
	return 0;
}
