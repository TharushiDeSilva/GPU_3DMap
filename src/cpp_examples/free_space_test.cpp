#include <iostream> 
#include <cmath>

using namespace std; 

const float max_sensor_radius = 3.00f; 
const float resolution = 0.05f; 

void int_bresenham3d(int x1, int y1, int z1, int x2, int y2, int z2){
    int dx = abs(x2 - x1);
    int dy = abs(y2 - y1); 
    int dz = abs(z2 - z1);
    int xs = -1,ys = -1,zs = -1;
    
    if (x2 > x1) { xs = 1; }    
    if (y2 > y1) { ys = 1; } 
    if (z2 > z1) { zs = 1; }

    if (dx >= dy and dx >= dz){
        // X is the driving axis
        std::cout<<"X is driving axis"; 
        
        int py = 2 * dy - dx; 
        int pz = 2 * dz - dx;
        while (x1 != x2){
            std::cout<<x1<<"\t"<<y1<<"\t"<<z1<<std::endl;
            x1 += xs; 
            if (py >= 0){ 
                y1 += ys; 
                py -= 2 * dx; 
            }
            if (pz >= 0){
                z1 += zs; 
                pz -= 2 * dx; 
            }
            py += 2 * dy; 
            pz += 2 * dz; 
        }
    }
    else if(dy >= dx and dy >= dz){
        // Y axis is the driving axis
        std::cout<<"y is driving axis";

        int px = 2 * dx - dy; 
        int pz = 2 * dz - dy; 
        while (y1 != y2){ 
            std::cout<<x1<<"\t"<<y1<<"\t"<<z1<<std::endl;
            y1 += ys; 
            if (px >= 0){ 
                x1 += xs; 
                px -= 2 * dy;
            }
            if (pz >= 0){ 
                z1 += zs; 
                pz -= 2 * dy;
            } 
            px += 2 * dx; 
            pz += 2 * dz;
        } 
    }
    else{
        // Z axis is the drivinf axis
        std::cout<<"z is driving axis";

        int py = 2*dy - dz;       // slope error 
        int px = 2*dx - dz; 

        while(z1 != z2){
            std::cout<<x1<<"\t"<<y1<<"\t"<<z1<<std::endl;
            z1 += zs; 
            if (py >= 0){ 
                y1 += ys; 
                py -= 2*dz; 
            }
            if (px >= 0){ 
                x1 += xs ;
                px -= 2*dz; 
            }
            py += 2*dy; 
            px += 2*dx; 
        }
    }

}

void float_bresenham3d(float x1, float y1, float z1, float x2, float y2, float z2){
    float dx = abs(x2 - x1);
    float dy = abs(y2 - y1); 
    float dz = abs(z2 - z1);
    
    float xs = -1*resolution;
    float ys = -1*resolution;
    float zs = -1*resolution;
    
    if (x2 > x1) { xs = resolution; }    
    if (y2 > y1) { ys = resolution; } 
    if (z2 > z1) { zs = resolution; }

    if (dx >= dy and dx >= dz){
        // X is the driving axis
        std::cout<<"X is driving axis"; 
        
        float py = 2 * dy - dx; 
        float pz = 2 * dz - dx;
        while (abs(x1-x2)>resolution/2){
            std::cout<<x1<<"\t"<<y1<<"\t"<<z1<<std::endl;
            x1 += xs; 
            if (py >= 0){ 
                y1 += ys; 
                py -= 2 * dx; 
            }
            if (pz >= 0){
                z1 += zs; 
                pz -= 2 * dx; 
            }
            py += 2 * dy; 
            pz += 2 * dz; 
        }
    }
    else if(dy >= dx and dy >= dz){
        // Y axis is the driving axis
        std::cout<<"y is driving axis";
        
        float px = 2 * dx - dy; 
        float pz = 2 * dz - dy; 
        while (abs(y1-y2)>resolution/2){ 
            std::cout<<x1<<"\t"<<y1<<"\t"<<z1<<std::endl;
            y1 += ys; 
            if (px >= 0){ 
                x1 += xs; 
                px -= 2 * dy;
            }
            if (pz >= 0){ 
                z1 += zs; 
                pz -= 2 * dy;
            } 
            px += 2 * dx; 
            pz += 2 * dz;
        } 
    }
    else{
        // Z axis is the driving axis
        std::cout<<"z is driving axis";
        
        float py = 2*dy - dz;       // slope error 
        float px = 2*dx - dz; 

        while(abs(z1-z2)>resolution/2){
            std::cout<<x1<<"\t"<<y1<<"\t"<<z1<<std::endl;
            z1 += zs; 
            if (py >= 0){ 
                y1 += ys; 
                py -= 2*dz; 
            }
            if (px >= 0){ 
                x1 += xs ;
                px -= 2*dz; 
            }
            py += 2*dy; 
            px += 2*dx; 
        }
    }

}

int main(){

    int number_of_steps; 
    number_of_steps = max_sensor_radius/resolution; 
    //std::cout<<number_of_steps<<std::endl; 
    
    //int_bresenham3d(0, 0, 0, 15, 10, 12); 
    //float_bresenham3d(0.025f, 0.025f, 0.025f, 2.975f, 2.975f, 2.975f); 
    for(int i=0; i<307200*60; i++){
        std::cout<<i<<endl; 
    }
    return 0; 
}