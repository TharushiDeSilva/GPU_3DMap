#include <iostream>

using namespace std; 

 

int main(){
    int *ptr;
    int arr[10]; 
    for(int i=0; i<10; i++){
        arr[i] = i*2; 
    }

    std::cout<<&arr<<std::endl; 
    ptr = arr; 
    std::cout<<ptr<<std::endl; 

    std::cout<<*(ptr+3)<<std::endl; 
    
    return 0; 
}