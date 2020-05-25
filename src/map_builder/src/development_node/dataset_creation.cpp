#include <iostream>
#include <bits/stdc++.h> 
#include <fstream>
#include <string>

using namespace std; 

int main(){ 
 
	ifstream ifile;
    ifile.open ("free_endpoints.txt", ios::in);
    float x_map, y_map, z_map;  
     int i=0;   
    while(ifile >>x_map>>y_map>>z_map){
        //std::cout<<x_map<<"\t"<<y_map<<"\t"<<z_map<<endl; 
        i+=1; 
    }
    ifile.close(); 
    cout<<i<<endl; 
	return 0; 	
}
