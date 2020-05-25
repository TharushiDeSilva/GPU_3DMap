#include <iostream>
using namespace std; 

int **ptr; 
int main(){
    ptr = new int *[3]; 
    //this statement is read as
    // ptr stores the base address of an array of 3 pointers to an int array

    for(int i=0; i<3; i++){
        *(ptr + i) = new int[4]; 
        // above loop can be replaced as
        //ptr = new int[3][4];
    }
    

    //manipulate the allocated memory 
    for(int i=0; i<3; i++){
        for(int j=0; j<4; j++){
            *(*(ptr+i)+j) = i+j; 
            //also can be written as
            //ptr[i][j] = i+j; 

            cout<<"ptr["<<i<<"]["<<j<<"] = "<<ptr[i][j]<<std::endl; 
        }
    }

        //free the allocated memory. 
        //number of delete statements mush match the number of new statements 

    for(int i=0; i<3; i++){
        delete[] *(ptr + i); 
        // above loop can be replaced as
        //ptr = new int[3][4];
    }
    delete[] ptr; 
    return 0; 
}