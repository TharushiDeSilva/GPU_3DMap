#ifndef COLOR3DPOINT_H
#define COLOR3DPOINT_H

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
        Color3DPoint(float x_in, float y_in, float z_in, uint8_t r_in, uint8_t g_in, uint8_t b_in); 
        ~Color3DPoint(); 

        float get_x(); 
        float get_y();
        float get_z();
        uint8_t get_r();
        uint8_t get_g();
        uint8_t get_b();
}; 

#endif