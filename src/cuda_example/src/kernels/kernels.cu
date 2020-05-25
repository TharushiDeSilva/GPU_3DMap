// This is for simple addition of two integers using cuda 
#include <stdio.h>

#include <cuda.h>
#include <cuda_runtime.h>

#define N (36000000)
#define THREADS_PER_BLOCK 36
static int NUM_OF_BLOCKS = 1;  

__global__ void VecAdd(double* A, double* B, double* C)
{
    int i = threadIdx.x;    // one dimensional vector array
    C[i] = A[i] + B[i];
}

int testmain(void){
    
    NUM_OF_BLOCKS = N/THREADS_PER_BLOCK; 
    
    clock_t start, end;

    double *a, *b, *c;                      //the host copies
    double *d_a, *d_b, *d_c;                // device copies

    int size = N * sizeof( double );

    a = (double *)malloc( size );
    b = (double *)malloc( size );
    c = (double *)malloc( size );

    for( int i = 0; i < N; i++ ){
        a[i] = b[i] = i;
        c[i] = 0;
    }
    //Memory allocation for device copies of vectors
    cudaMalloc( (void **) &d_a, size );
    cudaMalloc( (void **) &d_b, size );
    cudaMalloc( (void **) &d_c, size );

    //copying the vectors to allocated memory 
    cudaMemcpy( d_a, a, size, cudaMemcpyHostToDevice );
    cudaMemcpy( d_b, b, size, cudaMemcpyHostToDevice );
    
    start = clock();
    
    VecAdd<<< NUM_OF_BLOCKS, THREADS_PER_BLOCK >>>( d_a, d_b, d_c );
	
	end = clock();
    
    cudaMemcpy( c, d_c, size, cudaMemcpyDeviceToHost );

    //Freeing the allocated memory 
    free(a);
    free(b);
    free(c);
    cudaFree( d_a );
    cudaFree( d_b );
    cudaFree( d_c );

    
    double time = (double)(end-start);
    printf("CUDA: %f Clock cycles\n",time);
    return 0;  
}
