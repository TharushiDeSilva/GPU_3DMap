#include <iostream>
using namespace std; 

//function declaration 
int * sum (int *i, int *j); 

//function definition 
int * sum (int *i, int *j){
    int *result = new int; 
    *result = *i + *j; 
    return(result); 
}
int main(){
    int p=10, q=20; 
    int *r = sum(&p, &q); 
    cout<<"the sum is: "<<*r<<endl; 
    return 0; 
}