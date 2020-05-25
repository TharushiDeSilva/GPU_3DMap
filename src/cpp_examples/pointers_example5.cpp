#include <iostream>
using namespace std; 

int arr[3] = {10, 15, 20}; 
int *ptr = new int[3]; 

int main(){

    *(ptr+0) = 2; 
    ptr[1] = 4; 
    *(ptr+2) = 6; 

    // how to access pointers using either array or pointer notation 
    cout<<"arr[0] using array notation: "<<arr[0]<<std::endl; 
    cout<<"arr[1] using pointer notation: "<<*(arr+1)<<std::endl; 

    //access the base address of the array that the pointer points to
    std::cout<<"base address of array ptr points to: "<<ptr<<std::endl; 
    //access the base address of the arr
    std::cout<<"base address of array arr is: "<<arr<<std::endl; 

    return 0; 
}