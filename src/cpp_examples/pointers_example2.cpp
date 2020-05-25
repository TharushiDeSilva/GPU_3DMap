#include <iostream>
using namespace std;

int main(){
    int i=10; 
    int *x = &i; 
    int *y; 

    std::cout<<"the pointer x stored at the memory address "<< &x<<std::endl; 
    std::cout<<"the pointer x stores the memory address of i:  "<< x<<std::endl; 
    std::cout<<"the value of i accessed through pointer x is:  "<< *x<<std::endl; 

    //increment i by 1
    *x = *x+1; 
    std::cout<<"i through pointer = "<< *x<<" which equals i direct access: "<<i<<std::endl; 
    std::cout<<"the memory allocated to the pointer x is: "<< sizeof(x)<<" bytes"<<std::endl; 
    
    
    

    return 0; 
}