#include <iostream>
using namespace std; 

int i=0; 
int *ptr1;      //declaring a pointer to an integer 

int main(){
    ptr1 = new int;     //new allocates memory for an unnamed integer. 
    *ptr1 = 10; 
    *ptr1 = 2 + *ptr1; 
    cout<<"dynamic memory allocation. Value of ptr1 is: "<<*ptr1<<std::endl; 

    // free up the allocated memory by deferencing. use the pointer again, by assigning it another memory address. 
    delete ptr1; 
    ptr1 = new int[3];  //same pointer is now allocated to the base address of an array 
    //access the unnamed array
    *(ptr1+0) = 2; 
    *(ptr1+1) = 4;
    *(ptr1+2) = 6; 

    cout<<"printing the array of pointers"<<endl; 
    for(i=0; i<3; i++){
        cout<<"*(ptr1"<<i<<") = "<<*(ptr1+i)<<endl; 
    }

    cout<<"Printing the array of pointers: "<<endl; 
    for(i =0; i<3; i++){
        cout<<"ptr1["<<i<<"] ="<<ptr1[i]<<endl; 
    }

    //free memory 
    delete[ ] ptr1; 

    return 0; 
}