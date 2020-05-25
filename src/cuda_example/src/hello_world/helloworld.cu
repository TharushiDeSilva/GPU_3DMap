// This is the real Hello World for CUDA!
//It takes the string 'Hello', prints it, then passes it to cuda with an array of offsets
// Then the offsets are added in parallel to produce the string world! 
#include <stdio.h>

#include <cuda.h>
#include <cuda_runtime.h>

const int N=16; 
const int blocksize = 16;

__global__ void hello(char *a, int *b)
{
	a[threadIdx.x] += b[threadIdx.x];
}

int testmain()
{	
	printf("Hello from CPU\n");
	char a[N] = "Hello \0\0\0\0\0\0";
	int b[N] = {15, 10, 6, 0, -11, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

	char *ad; 
	int *bd; 
	const int csize = N*sizeof(char); 
	const int isize = N*sizeof(int); 

	printf("%s", a); 	
	
	cudaMalloc( (void**)&ad, csize); 
	cudaMalloc( (void**)&bd, isize); 
	cudaMemcpy(ad, a, csize, cudaMemcpyHostToDevice); 
	cudaMemcpy(bd, b, isize, cudaMemcpyHostToDevice); 

	dim3 dimBlock(blocksize, 1); 
	dim3 dimGrid(1,1); 
	hello<<<dimGrid, dimBlock>>>(ad, bd); 

	cudaError_t err = cudaGetLastError(); 
	if(err != cudaSuccess){
		printf("error: %s\n", cudaGetErrorString(err)); 
	}
	cudaMemcpy(a, ad, csize, cudaMemcpyDeviceToHost); 
	cudaFree(ad); 
	cudaFree(bd); 

	printf("%s\n", a);



	//Get the Device Properties. Very Important in handling blocks and threads.  
	cudaDeviceProp prop;
 	int count;
	cudaGetDeviceCount((&count)); 
	printf("%d\n", count); 
	
	for (int i=0; i< count; i++) {
		 cudaGetDeviceProperties( &prop, i);
		 printf( " --- General Information for device %d ---\n", i );
		 printf( "Name: %s\n", prop.name );
		 printf( "Compute capability: %d.%d\n", prop.major, prop.minor );
		 printf( "Clock rate: %d\n", prop.clockRate );
		 printf( "Device copy overlap: " );
		 if (prop.deviceOverlap)
		 printf( "Enabled\n" );
		 else
		 printf( "Disabled\n" );
		 printf( "Kernel execition timeout : " );
		 if (prop.kernelExecTimeoutEnabled)
		 printf( "Enabled\n" );
		 else
		 printf( "Disabled\n" );
		 printf( " --- Memory Information for device %d ---\n", i );
		 printf( "Total global mem: %ld\n", prop.totalGlobalMem );
		 printf( "Total constant Mem: %ld\n", prop.totalConstMem );
		 printf( "Max mem pitch: %ld\n", prop.memPitch );
		 printf( "Texture Alignment: %ld\n", prop.textureAlignment );
		 printf( " --- MP Information for device %d ---\n", i );
		 printf( "Multiprocessor count: %d\n",
		 prop.multiProcessorCount );
		 printf( "Shared mem per mp: %ld\n", prop.sharedMemPerBlock );
		 printf( "Registers per mp: %d\n", prop.regsPerBlock );
		 printf( "Threads in warp: %d\n", prop.warpSize );
		 printf( "Max threads per block: %d\n",
		 prop.maxThreadsPerBlock );
		 printf( "Max thread dimensions: (%d, %d, %d)\n",
		 prop.maxThreadsDim[0], prop.maxThreadsDim[1],
		 prop.maxThreadsDim[2] );
		 printf( "Max grid dimensions: (%d, %d, %d)\n",
		 prop.maxGridSize[0], prop.maxGridSize[1],
		 prop.maxGridSize[2] );
		 printf( "\n" );

	}
	
	cudaDeviceSynchronize(); 
	return EXIT_SUCCESS; 	
}
