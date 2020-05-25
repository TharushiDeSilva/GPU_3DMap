#include <stdio.h>
#include <time.h>

//#define N (1024*1024)
//#define M (10000)
//#define THREADS_PER_BLOCK 1024

#define N (51200000)
//#define M (5000)
#define THREADS_PER_BLOCK 36
static int NUM_OF_BLOCKS = 1;  

void serial_add(double *a, double *b, double *c, int n/*, int m */){
    for(int index=0;index<n;index++){
        //for(int j=0;j<m;j++){
            c[index] = a[index]*a[index] + b[index]*b[index];
            //printf("%d,",index); 
        //}
    }
}

__global__ void vector_add(double *a, double *b, double *c){
    int index = blockIdx.x * blockDim.x + threadIdx.x;
        //for(int j=0;j<M;j++){
            c[index] = a[index]*a[index] + b[index]*b[index];
        //}
}

int testmain(){
	
    if(N%THREADS_PER_BLOCK ==0){
    		NUM_OF_BLOCKS = N/THREADS_PER_BLOCK; 
    }
    else{
    	int pad = THREADS_PER_BLOCK - N%THREADS_PER_BLOCK; 
    	NUM_OF_BLOCKS = (N+pad)/THREADS_PER_BLOCK; 
    }
    
    printf("Num of Blocks: [%d]\tNum of threads: [%d]\n", NUM_OF_BLOCKS, THREADS_PER_BLOCK); 
    clock_t start, end;

    double *a, *b, *c;
    int size = N * sizeof( double );

    a = (double *)malloc( size );
    b = (double *)malloc( size );
    c = (double *)malloc( size );

    for( int i = 0; i < N; i++ ){
        a[i] = b[i] = i;
        c[i] = 0;
    }

    start = clock();
    	
    	serial_add(a, b, c, N/*, M*/);
	
	end = clock();
    //printf( "c[0] = %d\n",0,c[0] );
    //printf( "c[%d] = %d\n",N-1, c[N-1] );

    

    double time1 = (double)(end-start);// /CLOCKS_PER_SEC;
    printf("Serial: %f clock cycles\n",time1);
	start = clock();
    double *d_a, *d_b, *d_c;


    cudaMalloc( (void **) &d_a, size );
    cudaMalloc( (void **) &d_b, size );
    cudaMalloc( (void **) &d_c, size );


    cudaMemcpy( d_a, a, size, cudaMemcpyHostToDevice );
    cudaMemcpy( d_b, b, size, cudaMemcpyHostToDevice );
    
    end = clock();
    double time3 = (double)(end-start);
    printf("Memory Allocation Takes : %f clock cycles\n",time3);
	
	start = clock();
    
    //vector_add<<< (N + (THREADS_PER_BLOCK-1)) / THREADS_PER_BLOCK, THREADS_PER_BLOCK >>>( d_a, d_b, d_c );
    vector_add<<< NUM_OF_BLOCKS, THREADS_PER_BLOCK >>>( d_a, d_b, d_c );
	
	end = clock();
    cudaMemcpy( c, d_c, size, cudaMemcpyDeviceToHost );


    //printf( "c[0] = %d\n",0,c[0] );
    //printf( "c[%d] = %d\n",N-1, c[N-1] );


    free(a);
    free(b);
    free(c);
    cudaFree( d_a );
    cudaFree( d_b );
    cudaFree( d_c );

    
    double time2 = (double)(end-start);// /CLOCKS_PER_SEC;
    printf("CUDA: %f Clock cycles,\n Speedup: %f\n",time2, time1/time2);

    return 0;
} 
