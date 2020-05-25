#include <iostream>

using namespace std;

char ch1 = 'a', ch2 = 'b'; 
char ch3[4] = {'c', 'd', 'e', 'f'}; 
char *ptr[3]; 

int main(){

    ptr[0] = &ch1; 
    ptr[1] = &ch2; 
    ptr[2] = ch3;   // base address of ch3 is stored in ptr[2]. ptr[2] = &ch3[0]

    cout<<"Initial character stored in ptr[0] and ptr[1]: "<<*ptr[0]<<" "<<*ptr[1]<<endl; 
    *ptr[0] = 'z'; 
    *ptr[1] = 'y'; 
    cout<<"Final characters stored in ptr[0] and ptr[1]: "<<*ptr[0]<<" "<<*ptr[1]<<std::endl;   
    std::cout<<"the variable (direct access values) of ch1, and ch2 are: "<<ch1<<" "<<ch2<<std::endl; 
    std::cout<<"check the value of ptr[2]"<<*ptr[2]<<" which should be equal to: "<<ch3[0]<<endl; 


    //how to access the rest of ch3 array using ptr[2]??
    std::cout <<"print ch3[1] using ptr[2]: "<<*(ptr[2] + 1)<<endl; 
    return 0; 
}