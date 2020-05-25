#include <iostream>
#include <stdint.h>


using namespace std; 

class Color3DPoint{ 
    
    private: 
        float x; 
        float y; 
        float z; 
        uint8_t r; 
        uint8_t g; 
        uint8_t b; 

    public:
        Color3DPoint(float x_in, float y_in, float z_in, uint8_t r_in, uint8_t g_in, uint8_t b_in){
            x = x_in; 
            y = y_in; 
            z = z_in; 
            r = r_in; 
            g = g_in; 
            b = b_in; 
        }
        Color3DPoint::~Color3DPoint(); //destructor 

        float get_x(){return x; }
        float get_y(){return y; }
        float get_z(){return z; }
        uint8_t get_r(){return r; }
        uint8_t get_g(){return g; }
        uint8_t get_b(){return b; }
}; 