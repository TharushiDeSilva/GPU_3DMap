// evaluate the data structures in morton code and octree

#include "iostream"
#include "ros/ros.h"
#include <boost/unordered_map.hpp>

using namespace std; 
boost::unordered::unordered_map<uint64_t, uint32_t> octree;

int cudamain(int size, float low, float high);  

float L = -9.9342446f; 
float H = 9.94333346f;
int n = 100000; 

int main(int argc, char **argv){
    ros::init(argc, argv, "color_pointcloud_sub");
    
    for(int i=0; i<10; i++){ 
        cudamain(n, L, H);
        //n = n + 100000;

        // decode and publish. measure time here 
    }
    cout<<"finished: current size is: "<<octree.size()<<endl;
    
    return 0; 
}
