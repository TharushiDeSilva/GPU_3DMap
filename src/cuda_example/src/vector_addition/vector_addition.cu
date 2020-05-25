#include <stdio.h>
#include <time.h> 
#include <cuda.h>
#include <cuda_runtime.h>


__global__ void add(int *a, int *b, int *c, int n) {
	int index = threadIdx.x + blockIdx.x * blockDim.x;
	if(index <n){
		c[index] = a[index] + b[index];
	}
}


#define N (2048*2048)
#define THREADS_PER_BLOCK 512


int testmain(void){
	
	int *a, *b, *c; 
	int *d_a, *d_b, *d_c; 
	int size = N*sizeof(int); 
	
	//Allocate space for device copies a, b, c
	cudaMalloc((void **)&d_a, size); 
	cudaMalloc((void **)&d_b, size); 
	cudaMalloc((void **)&d_c, size); 
	
	//Allocate space for host copies of a, b, c, and setup input values
	a = (int *)malloc(size); //random_ints(a, N); 
	b = (int *)malloc(size); //random_ints(b, N); 
	c = (int *)malloc(size);
	
	//Initialize two array in a simple way
	for(int i=0; i<N; i++){
		a[i] = -i; 
		b[i] = i*i; 
	}
	
	//Copy inputs to device 
	cudaMemcpy(d_a, a, size, cudaMemcpyHostToDevice);
	cudaMemcpy(d_b, b, size, cudaMemcpyHostToDevice); 
	
	
	//Launch add kernel on GPU with N blocks
	add<<<(N + THREADS_PER_BLOCK-1)/THREADS_PER_BLOCK, THREADS_PER_BLOCK>>>(d_a, d_b, d_c, N); 
	
	
	//Copy results back to host
	cudaMemcpy(c, d_c, size, cudaMemcpyDeviceToHost);

	
	//clean up
	free(a); 
	free(b); 
	free(c);
	
	cudaFree(d_a); 
	cudaFree(d_b); 
	cudaFree(d_c);
	
	return 0;
}


