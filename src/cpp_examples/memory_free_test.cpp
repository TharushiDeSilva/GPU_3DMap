#include <iostream>
#include <stdlib.h>

using namespace std; 

int *arr; 
int *arr_copy; 
int main(){
    arr = (int *)malloc( 10 * sizeof(int));
    for(int i=0; i<10; i++){
        arr[i] = i*2; 
    }

    // for(int i=0; i<100; i++){
    //     std::cout<<i<<"  "<<arr[i]<<std::endl; 
    // }
    arr_copy = arr; 
    //free (arr); 
    std::cout<<"freed memory"<<"\n\n\n"; 
    for(int i=0; i<10; i++){
        std::cout<<i<<"  "<<*(arr_copy+i)<<std::endl; 
    }
    // for(int j=0; j<100; j++){
    //     arr[j] = j; 
    // }
    // for(int j=0; j<100; j++){
    //     std::cout<<j<<"  "<<arr[j]<<std::endl; 
    // }
    free (arr); 

    return 0; 
}