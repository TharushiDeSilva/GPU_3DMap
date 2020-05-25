#include <iostream>
#include <cuda.h>
#include <cuda_runtime.h>
#include <time.h>
#include <string>
#include <cmath> 
#include "math.h"
#include <cstdlib>
#include <stdio.h>



#define THREADS_PER_BLOCK 256		// the optimal value is number of cuda cores, if (#cuda cores < max th.pr.blck). 256 for TX2
static int NUM_OF_BLOCKS = 1; 

__device__ const float resolution = 0.05f; 	// Resolution of 5 cm
__device__ const float half_resolution = resolution/2; // the half of the resolution. this value is used in calculations 
const float max_sensor_radius = 3.00f; 	// scan data further than this modulus value will not be considered. 
 
extern float *x_rounded, *y_rounded, *z_rounded; 		// the intermediate results after rounding off the x, y, z. 

__global__ void round_off_positional_coords(float *x, float *y, float *z, float *x_result, float *y_result, float *z_result){
	
	int index = threadIdx.x + blockIdx.x * blockDim.x;
	 
	if( (fabs(x[index]) < max_sensor_radius) and (fabs(y[index]) < max_sensor_radius) and (fabs(z[index]) < max_sensor_radius)){
		x_result[index] = (ceilf(x[index] / resolution))*0.05 - half_resolution;
		y_result[index] = (ceilf(y[index] / resolution))*0.05 - half_resolution; 
		z_result[index] = (ceilf(z[index] / resolution))*0.05 - half_resolution; 	
		   
		
	}else{
		 x_result[index] = 0.00f; 
		 y_result[index] = 0.00f; 
		 z_result[index] = 0.00f; 

	}
	
}


int cudamain(float *x_in, float *y_in, float *z_in, int array_size_in, int size_position_in){ 
		 
	//x_global = (float *)malloc( size_position_in);
	// x_rounded = new float[array_size_in]; 
	// y_rounded = new float[array_size_in]; 
	// z_rounded = new float[array_size_in]; 
	
	// Adjust the number of blocks to be a whole number. 
	NUM_OF_BLOCKS = (array_size_in + THREADS_PER_BLOCK-1)/THREADS_PER_BLOCK; 


	//The cuda device variables 
	float *d_x, *d_y, *d_z;
	float *d_x_rounded, *d_y_rounded, *d_z_rounded; 
	
	cudaMalloc( (void **) &d_x, size_position_in);
	cudaMalloc( (void **) &d_y, size_position_in);
	cudaMalloc( (void **) &d_z, size_position_in);
	
	cudaMalloc( (void **) &d_x_rounded, size_position_in);
	cudaMalloc( (void **) &d_y_rounded, size_position_in);
	cudaMalloc( (void **) &d_z_rounded, size_position_in);

    cudaMemcpy( d_x, x_in, size_position_in, cudaMemcpyHostToDevice );
	cudaMemcpy( d_y, y_in, size_position_in, cudaMemcpyHostToDevice );
	cudaMemcpy( d_z, z_in, size_position_in, cudaMemcpyHostToDevice );
	
	// GPU process START---------------------------------------------------------------------------------------------------------------------------------------------------------------
	//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	//double startc, endc; 
	//startc = clock();
	round_off_positional_coords<<<NUM_OF_BLOCKS, THREADS_PER_BLOCK>>>(d_x, d_y, d_z, d_x_rounded, d_y_rounded, d_z_rounded);
	//endc = clock();
	//double time_parallel = (double)(endc-startc); 
	//std::cout<<"Time to generate voxel array: "<<time_parallel<<std::endl; 
	cudaMemcpy( x_rounded, d_x_rounded, size_position_in, cudaMemcpyDeviceToHost );
	cudaMemcpy( y_rounded, d_y_rounded, size_position_in, cudaMemcpyDeviceToHost );
	cudaMemcpy( z_rounded, d_z_rounded, size_position_in, cudaMemcpyDeviceToHost );
	// GPU process END----------------------------------------------------------------------------------------------------------------------------------------------------------------
	//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	
	cudaFree( d_x );
	cudaFree( d_y );
	cudaFree( d_z );
	cudaFree(d_x_rounded); 
	cudaFree(d_y_rounded); 
	cudaFree(d_z_rounded); 
	
	return EXIT_SUCCESS; 	
}
