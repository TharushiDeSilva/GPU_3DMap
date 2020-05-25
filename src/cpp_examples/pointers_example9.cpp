// example 8 with three dimensional pointers 
#include <iostream> 
using namespace std; 

int ***ptr; 

int main(){
    ptr = new int **[2]; 

    for(int i=0; i<2; i+=1){
        *(ptr+1) = new int *[3]; 
        for(int j=0; j<3; j++){
            *(*(ptr+i)+j) = new int [4]; 
        }
    }
    //access the allocated memory 
    std::cout<<"summing values within [], assigning to pointer ans printing"<<std::endl; 
    for(int i=0; i<2; i++){
        for(int j=0; j<3; j++){
            for(int k=0; k<4; k++){
                *(*(*(ptr+i)+j)+k) = i+j+k;
                //ptr[i][j][k] = i+j+k; 
                std::cout<<"ptr["<<i<<"]["<<j<<"]["<<k<<"] = "<<ptr[i][j][k]<<std::endl; 
            }
        }
    }
    
    //delete the allocated memoey 
    for(int i=0; i<2; i+=1){
        for(int j=0; j<3; j++){
            delete[] *(*(ptr+i)+j);  
        }
        delete[] (ptr+1); 
    }
    delete[] ptr; 
    return 0; 
}