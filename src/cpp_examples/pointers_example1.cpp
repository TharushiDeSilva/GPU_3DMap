#include <iostream>

using namespace std; 

int main(){
    for(int x=0; x<5; x++){
        for(int y=0; y<5; y++){
            for(int z=0; z<5; z++){
                cout<<x<<","<<y<<","<<z<<endl; 
            }
        }
    }
    return 0; 
}