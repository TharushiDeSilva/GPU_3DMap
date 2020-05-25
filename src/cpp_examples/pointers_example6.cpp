#include <iostream> 

using namespace std; 

int **ptr2; 
int *ptr1; 
int i = 10; 

int main(){
    ptr1 = &i; 
    //assign the base address of ptr1 to ptr2
    ptr2 = &ptr1; 

    //accessing the varibale through multi dimensional pointers
    std::cout<<"Accessing i through i: "<<i<<std::endl; 
    std::cout<<"Accessing in through ptr1: "<<*ptr1<<std::endl; 
    std::cout<<"Accessing in through ptr2: "<<**ptr2<<std::endl; 
    return 0; 
}