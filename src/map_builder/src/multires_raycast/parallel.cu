// this program will detect dynamic objects in the octomap 

#include <iostream>
#include <bits/stdc++.h> 
#include <stdint.h>
#include <fstream>
#include <cuda.h>
#include <cuda_runtime.h>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <time.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <string>
#include <cmath> 
#include "math.h"
#include <cstdlib>
#include <stdio.h>
#include <map>
#include <iterator>
#include <boost/lexical_cast.hpp>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include "geometry_msgs/Quaternion.h"


using namespace std; 

// =========================================================================================================================
// This class file is to transport into include directory after development 
float res_list_5cm[] = {3276.80, 1638.40, 819.20, 409.60, 204.80, 102.40, 51.20, 25.60, 12.80, 6.40, 3.20, 1.60, 0.80, 0.40, 0.20, 0.10, 0.05, 0.025}; // in meters

struct OctreeNode{
    //for now we consider a depth of 16 for data nodes. max reolution = 65536 cm. float is enough 
    float X; 
    float Y; 
    float Z; 
    uint8_t R; 
    uint8_t G; 
    uint8_t B; 
    //uint8_t level;          // possible replace this with 4 bit binary code, or not to have it at all   
    int8_t Occ;  
    OctreeNode *BNE; //hhl; 
    OctreeNode *BSE; //hll; 
    OctreeNode *BSW; //lll, 
    OctreeNode *BNW; //lhl; 
    OctreeNode *TNE; //hhh; 
    OctreeNode *TSE; //hlh; 
    OctreeNode *TSW; //llh, 
	OctreeNode *TNW; //lhh; 
	
	//constructor
	OctreeNode(){
		X = 0.00f; 
		Y = 0.00f; 
		Z = 0.00f; 
		R = 255;
		G = 255; 
		B = 255; 
		Occ = 0;
		BNE = NULL; 
		BSE = NULL; 
		BSW = NULL; 
		BNW = NULL; 
		TNE = NULL; 
		TSE = NULL; 
		TSW = NULL; 
		TNW = NULL;   
	}

};

OctreeNode *current;            // these are global varibale used to travel down the tree 
OctreeNode *parent; 
queue<OctreeNode*> leaf_nodes;

class Octree{

    OctreeNode *root; 

	void insertNode(float x, float y, float z, uint8_t r, uint8_t g, uint8_t b){
        // insert this node at the depth of 16. where all the leaf nodes are 
		if(root==NULL){
			return;
		}
		int depth = 0; //root level
        current = root; 
        //cout<<"starting from root"<<endl;  

		for(depth=0; depth<16; depth++){
			if(current->BNE == NULL){
				//cout<<depth<<" next level null"<<endl; 
				
				OctreeNode *node_BNE = new OctreeNode; 
				node_BNE->X = current->X + res_list_5cm[depth+2];
				node_BNE->Y = current->Y + res_list_5cm[depth+2]; 
				node_BNE->Z = current->Z - res_list_5cm[depth+2]; 
				current->BNE = node_BNE;
				//cout<<"established BNE node of level "<<depth<<endl; 
				
				OctreeNode *node_BSE = new OctreeNode; 
				node_BSE->X = current->X + res_list_5cm[depth+2];
				node_BSE->Y = current->Y - res_list_5cm[depth+2]; 
				node_BSE->Z = current->Z - res_list_5cm[depth+2];
				current->BSE = node_BSE;
				//cout<<"established BSE node of level "<<depth<<endl; 

				OctreeNode *node_BSW = new OctreeNode; 
				node_BSW->X = current->X - res_list_5cm[depth+2];
				node_BSW->Y = current->Y - res_list_5cm[depth+2]; 
				node_BSW->Z = current->Z - res_list_5cm[depth+2]; 
				current->BSW = node_BSW;
				//cout<<"established BSW node of level "<<depth<<endl;

				OctreeNode *node_BNW = new OctreeNode; 
				node_BNW->X = current->X - res_list_5cm[depth+2];
				node_BNW->Y = current->Y + res_list_5cm[depth+2]; 
				node_BNW->Z = current->Z - res_list_5cm[depth+2];
				current->BNW = node_BNW;
				//cout<<"established BNW node of level "<<depth<<endl;

				OctreeNode *node_TNE = new OctreeNode; 
				node_TNE->X = current->X + res_list_5cm[depth+2];
				node_TNE->Y = current->Y + res_list_5cm[depth+2]; 
				node_TNE->Z = current->Z + res_list_5cm[depth+2];
				current->TNE = node_TNE;
				//cout<<"established TNE node of level "<<depth<<endl; 
				
				OctreeNode *node_TSE = new OctreeNode; 
				node_TSE->X = current->X + res_list_5cm[depth+2];
				node_TSE->Y = current->Y - res_list_5cm[depth+2]; 
				node_TSE->Z = current->Z + res_list_5cm[depth+2];
				current->TSE = node_TSE;
				//cout<<"established TSE node of level "<<depth<<endl; 

				OctreeNode *node_TSW = new OctreeNode; 
				node_TSW->X = current->X - res_list_5cm[depth+2];
				node_TSW->Y = current->Y - res_list_5cm[depth+2]; 
				node_TSW->Z = current->Z + res_list_5cm[depth+2];
				current->TSW = node_TSW;
				//cout<<"established TSW node of level "<<depth<<endl;

				OctreeNode *node_TNW = new OctreeNode; 
				node_TNW->X = current->X - res_list_5cm[depth+2];
				node_TNW->Y = current->Y + res_list_5cm[depth+2]; 
				node_TNW->Z = current->Z + res_list_5cm[depth+2];
				current->TNW = node_TNW;
				//cout<<"established TNW node of level "<<depth<<endl;

				// maintain leaf nodes index. 
				if(depth == 15){
                    leaf_nodes.push(current->BNE);
                    leaf_nodes.push(current->BSE); 
                    leaf_nodes.push(current->BSW); 
                    leaf_nodes.push(current->BNW); 
                    leaf_nodes.push(current->TNE);
                    leaf_nodes.push(current->TSE); 
                    leaf_nodes.push(current->TSW); 
                    leaf_nodes.push(current->TNW);  
                }

			}else{
				//cout<<depth<<" next level not null"<<endl; 
				//cout<<"next level x  "<<current->BNE->X<<endl; 
			}

			if(x>current->X && y>current->Y && z<current->Z){
				current = current->BNE; 
			}else if(x>current->X && y<current->Y && z<current->Z){
				current = current->BSE; 
			}else if(x<current->X && y<current->Y && z<current->Z){
				current = current->BSW; 
			}else if(x<current->X && y>current->Y && z<current->Z){
				current = current->BNW; 
			}else if(x>current->X && y>current->Y && z>current->Z){
				current = current->TNE; 
			}else if(x>current->X && y<current->Y && z>current->Z){
				current = current->TSE; 
			}else if(x<current->X && y<current->Y && z>current->Z){
				current = current->TSW; 
			}else if(x<current->X && y>current->Y && z>current->Z){
				current = current->TNW; 
			}else{
				//std::cout<<"something else"<<endl; 
			}
		}

		// now we are at level 16. 
		// "current" pointer indicates to the exact node the voxel should be added. 
		if(current->Occ == 0){		// means not initialized
			current->Occ = 1; 
			current->R = r;
			current->G = g;
			current->B = b; 
		}else if(current->Occ == -1){	// we do not need to recognize this node as unknown 
			current->Occ = 1; 
			current->R = (current->R + r)/2;
			current->G = (current->G + g)/2;
			current->B = (current->B + b)/2;
		}else if(current->Occ <16){
			current->Occ += 1; 
			current->R = (current->R + r)/2;
			current->G = (current->G + g)/2;
			current->B = (current->B + b)/2;
		}else{
			// no case according to algorithm
		}
	}
	void insert_free_Node(float x, float y, float z){
        // insert this node at the depth of 16. where all the leaf nodes are 
		if(root==NULL){
			return;
		}
		int depth = 0; //root level
        current = root; 
        //cout<<"starting from root"<<endl;  

		for(depth=0; depth<16; depth++){
			if(current->BNE == NULL){
				//cout<<depth<<" next level null"<<endl; 
				
				OctreeNode *node_BNE = new OctreeNode; 
				node_BNE->X = current->X + res_list_5cm[depth+2];
				node_BNE->Y = current->Y + res_list_5cm[depth+2]; 
				node_BNE->Z = current->Z - res_list_5cm[depth+2]; 
				current->BNE = node_BNE;
				//cout<<"established BNE node of level "<<depth<<endl; 
				
				OctreeNode *node_BSE = new OctreeNode; 
				node_BSE->X = current->X + res_list_5cm[depth+2];
				node_BSE->Y = current->Y - res_list_5cm[depth+2]; 
				node_BSE->Z = current->Z - res_list_5cm[depth+2];
				current->BSE = node_BSE;
				//cout<<"established BSE node of level "<<depth<<endl; 

				OctreeNode *node_BSW = new OctreeNode; 
				node_BSW->X = current->X - res_list_5cm[depth+2];
				node_BSW->Y = current->Y - res_list_5cm[depth+2]; 
				node_BSW->Z = current->Z - res_list_5cm[depth+2]; 
				current->BSW = node_BSW;
				//cout<<"established BSW node of level "<<depth<<endl;

				OctreeNode *node_BNW = new OctreeNode; 
				node_BNW->X = current->X - res_list_5cm[depth+2];
				node_BNW->Y = current->Y + res_list_5cm[depth+2]; 
				node_BNW->Z = current->Z - res_list_5cm[depth+2];
				current->BNW = node_BNW;
				//cout<<"established BNW node of level "<<depth<<endl;

				OctreeNode *node_TNE = new OctreeNode; 
				node_TNE->X = current->X + res_list_5cm[depth+2];
				node_TNE->Y = current->Y + res_list_5cm[depth+2]; 
				node_TNE->Z = current->Z + res_list_5cm[depth+2];
				current->TNE = node_TNE;
				//cout<<"established TNE node of level "<<depth<<endl; 
				
				OctreeNode *node_TSE = new OctreeNode; 
				node_TSE->X = current->X + res_list_5cm[depth+2];
				node_TSE->Y = current->Y - res_list_5cm[depth+2]; 
				node_TSE->Z = current->Z + res_list_5cm[depth+2];
				current->TSE = node_TSE;
				//cout<<"established TSE node of level "<<depth<<endl; 

				OctreeNode *node_TSW = new OctreeNode; 
				node_TSW->X = current->X - res_list_5cm[depth+2];
				node_TSW->Y = current->Y - res_list_5cm[depth+2]; 
				node_TSW->Z = current->Z + res_list_5cm[depth+2];
				current->TSW = node_TSW;
				//cout<<"established TSW node of level "<<depth<<endl;

				OctreeNode *node_TNW = new OctreeNode; 
				node_TNW->X = current->X - res_list_5cm[depth+2];
				node_TNW->Y = current->Y + res_list_5cm[depth+2]; 
				node_TNW->Z = current->Z + res_list_5cm[depth+2];
				current->TNW = node_TNW;
				//cout<<"established TNW node of level "<<depth<<endl;

				// maintain leaf nodes index. 
				if(depth == 15){
                    leaf_nodes.push(current->BNE);
                    leaf_nodes.push(current->BSE); 
                    leaf_nodes.push(current->BSW); 
                    leaf_nodes.push(current->BNW); 
                    leaf_nodes.push(current->TNE);
                    leaf_nodes.push(current->TSE); 
                    leaf_nodes.push(current->TSW); 
                    leaf_nodes.push(current->TNW);  
				}
				
			}else{
				//cout<<depth<<" next level not null"<<endl; 
				//cout<<"next level x  "<<current->BNE->X<<endl; 
			}

			if(x>current->X && y>current->Y && z<current->Z){
				current = current->BNE; 
			}else if(x>current->X && y<current->Y && z<current->Z){
				current = current->BSE; 
			}else if(x<current->X && y<current->Y && z<current->Z){
				current = current->BSW; 
			}else if(x<current->X && y>current->Y && z<current->Z){
				current = current->BNW; 
			}else if(x>current->X && y>current->Y && z>current->Z){
				current = current->TNE; 
			}else if(x>current->X && y<current->Y && z>current->Z){
				current = current->TSE; 
			}else if(x<current->X && y<current->Y && z>current->Z){
				current = current->TSW; 
			}else if(x<current->X && y>current->Y && z>current->Z){
				current = current->TNW; 
			}else{
				//std::cout<<"something else"<<endl; 
			}
		}

		// now we are at level 16. 
		// "current" pointer indicates to the exact node the voxel should be added. 
		
		if(current->Occ == 1){	// we do not need to recognize this node as unknown 
			current->Occ = -1; 
		}else if(current->Occ > -16){
			current->Occ -= 1; 
		}else{
			// no case according to algorithm
		}
    }
    
    void inOrderTraverse(OctreeNode *node){
        if(node == NULL){
            return; 
        }
        inOrderTraverse(node->TNW);  
        inOrderTraverse(node->TNE); 
        inOrderTraverse(node->TSE); 
        inOrderTraverse(node->TSW); 
        inOrderTraverse(node->BNW); 
        inOrderTraverse(node->BNE); 
        inOrderTraverse(node->BSE); 
        inOrderTraverse(node->BSW); 
        std::cout<<"("<<node->X<<", "<<node->Y<<", "<<node->Z<<")\n";
    }

    void put_nodes_in_queue(OctreeNode *node){
        if(node == NULL){
            return; 
        }
        put_nodes_in_queue(node->TNW);  
        put_nodes_in_queue(node->TNE); 
        put_nodes_in_queue(node->TSE); 
        put_nodes_in_queue(node->TSW); 
        put_nodes_in_queue(node->BNW); 
        put_nodes_in_queue(node->BNE); 
        put_nodes_in_queue(node->BSE); 
        put_nodes_in_queue(node->BSW); 
		if(node->Occ > 0){
			leaf_nodes.push(node); 
		}
        //std::cout<<"pushing: "<<node<<std::endl; 
    }

    void postOrderDelete(){
        if(root == NULL){
            return; 
        }
        stack<OctreeNode*> node_stack; 
        stack<char> id_stack; 
        OctreeNode *current = new OctreeNode; 
        current = root; 
        node_stack.push(current); 

        while(!node_stack.empty()){
            if(current->TNW == NULL && current->TNE == NULL && current->TSE == NULL && current->TSW == NULL &&
                current->BNW == NULL && current->BNE == NULL && current->BSE == NULL && current->BSW == NULL){
                    // no children 
                    node_stack.pop(); 
                    if(!node_stack.empty()){
                        // not at root
                        current = node_stack.top(); 
                        if(id_stack.top() == '0'){
                            current->TNW = NULL; 
                            id_stack.pop(); 
                        }else if(id_stack.top() == '1'){
                            current->TNE = NULL; 
                            id_stack.pop(); 
                        }else if(id_stack.top() == '2'){
                            current->TSE = NULL; 
                            id_stack.pop(); 
                        }else if(id_stack.top() == '3'){
                            current->TSW = NULL; 
                            id_stack.pop();
                        }else if(id_stack.top() == '4'){
                            current->BNW = NULL; 
                            id_stack.pop();
                        }else if(id_stack.top() == '5'){
                            current->BNE = NULL; 
                            id_stack.pop();
                        }else if(id_stack.top() == '6'){
                            current->BSE = NULL; 
                            id_stack.pop();
                        }else{
                            current->BSW = NULL; 
                            id_stack.pop();
                        }
                }else{
                    // if we've come to the root
                    //std::cout<<"deleting: "<<root->Key<<endl; 
                    root = NULL; 

                }
            }else if(current->TNW != NULL){
                current = current->TNW; 
                node_stack.push(current); 
                id_stack.push('0');
            }else if(current->TNE != NULL){
                current = current->TNE; 
                node_stack.push(current); 
                id_stack.push('1');
            }else if(current->TSE != NULL){
                current = current->TSE; 
                node_stack.push(current); 
                id_stack.push('2');
            }else if(current->TSW != NULL){
                current = current->TSW; 
                node_stack.push(current); 
                id_stack.push('3');
            }else if(current->BNW != NULL){
                current = current->BNW; 
                node_stack.push(current); 
                id_stack.push('4');
            }else if(current->BNE != NULL){
                current = current->BNE; 
                node_stack.push(current); 
                id_stack.push('5');
            }else if(current->BSE != NULL){
                current = current->BSE; 
                node_stack.push(current); 
                id_stack.push('6');
            }else if(current->BSW != NULL){
                current = current->BSW; 
                node_stack.push(current); 
                id_stack.push('7');
            }else{}
        }
    }

    OctreeNode *searchNode(OctreeNode *node, float x, float y, float z){
        if(node == NULL){
            return NULL; 
        }else if((node->X == x) && (node->Y == y) && (node->Z == z)){
            return node; 
        }else if((node->X >= x) && (node->Y >= y) && (node->Z >= z)){
            return searchNode(node->TNW, x, y, z); 
        }else if((node->X >= x) && (node->Y >= y) && (node->Z <= z)){
            return searchNode(node->TNE, x, y, z); 
        }else if((node->X >= x) && (node->Y <= y) && (node->Z >= z)){
            return searchNode(node->TSE, x, y, z); 
        }else if((node->X >= x) && (node->Y <= y) && (node->Z <= z)){
            return searchNode(node->TSW, x, y, z); 
        }else if((node->X <= x) && (node->Y >= y) && (node->Z >= z)){
            return searchNode(node->BNW, x, y, z); 
        }else if((node->X <= x) && (node->Y >= y) && (node->Z <= z)){
            return searchNode(node->BNE, x, y, z); 
        }else if((node->X <= x) && (node->Y <= y) && (node->Z >= z)){
            return searchNode(node->BSE, x, y, z); 
        }else if(node->X <= x && node->Y <= y && node->Z <= z){
            return searchNode(node->BSW, x, y, z);
        }else{
            return node;   
        }
    }
    
    OctreeNode *findMinNode(){
        if(root == NULL){
            return NULL; 
        }
        //OctreeNode *current = new OctreeNode; 
        current = NULL; 
        current = root; 
        while(current->TNW != NULL || current->TNE != NULL || current->TSE != NULL || current->TSW != NULL 
            || current->BNW != NULL || current->BNE != NULL || current->BSE != NULL || current->BSW != NULL){
                // while the current node has any children
                if(current->TNW != NULL){
                    current = current->TNW;                        
                }else if(current->TNE != NULL){
                    current = current->TNE;                       
                }else if(current->TSE != NULL){
                    current = current->TSE;
                }else if(current->TSW != NULL){
                    current = current->TSW;          
                }else if(current->BNW != NULL){
                    current = current->BNW; 
                }else if(current->BNE != NULL){
                    current = current->BNE;                    
                }else if(current->BSE != NULL){
                    current = current->BSE;         
                }else if(current->BSW != NULL){
                    current = current->BSW;                      
                }else{
                    // No children. no case
                }
            }
        return current; 
    }
    
    OctreeNode *findMaxNode(){
        if(root == NULL){
            return NULL; 
        }else{
            OctreeNode *current = new OctreeNode; 
            current = root; 
            while(current->TNW != NULL || current->TNE != NULL || current->TSE != NULL || current->TSW != NULL 
                || current->BNW != NULL || current->BNE != NULL || current->BSE != NULL || current->BSW != NULL){
                // while the current node has any children
                if(current->BSW != NULL){
                    current = current->BSW;                        
                }else if(current->BSE != NULL){
                    current = current->BSE;                       
                }else if(current->BNE != NULL){
                    current = current->BNE;
                }else if(current->BNW != NULL){
                    current = current->BNW;          
                }else if(current->TSW != NULL){
                    current = current->TSW; 
                }else if(current->TSE != NULL){
                    current = current->TSE;                    
                }else if(current->TNE != NULL){
                    current = current->TNE;         
                }else if(current->TNW != NULL){
                    current = current->TNW;                      
                }else{
                    // No children. no case
                }
            }
            return current; 
        }
    }
        void destroy(OctreeNode *root){
            if(root == NULL){
                return;
            }else{
                destroy(root->TNW);
                destroy(root->TNE);
                destroy(root->TSE); 
                destroy(root->TSW); 
                destroy(root->BNW);
                destroy(root->BNE);
                destroy(root->BSE); 
                destroy(root->BSW); 
                delete root;
            }
        }  

    public:
        Octree(OctreeNode *node){
            root = node; 
        }

        Octree(float leaf_resolution){
            OctreeNode *root_node = new OctreeNode; 
            root_node->X = 0.00f; 
            root_node->Y = 0.00f;
            root_node->Z = 0.00f; 
            //root_node->Occ = 0;     // unknown - so zero
            //root_node->level = 0; 
            //root_node->R = 255; 
            //root_node->G = 255; 
            //root_node->B = 255; 
            //root_node->TNW = NULL;
            //root_node->TNE = NULL; 
            //root_node->TSE = NULL;
            //root_node->TSW = NULL;
            //root_node->BNW = NULL;
            //root_node->BNE = NULL; 
            //root_node->BSE = NULL;
            //oot_node->BSW = NULL;
            
            root = root_node; 
        }
        ~Octree(){
            //postOrderDelete(); 
            destroy(root); 
            std::cout<<"destroyed"<<endl; 
        }
        void insert(float x, float y, float z, uint8_t r, uint8_t g, uint8_t b){
            insertNode(x, y, z, r, g, b); 
        }

        void insert_free(float x, float y, float z){
            insert_free_Node(x, y, z); 
        }
        void display(){
            inOrderTraverse(root); 
            std::cout<<std::endl; 
        }

        void put_in_queue(){
            put_nodes_in_queue(root); 
        }

        void searchNode(float x, float y, float z){
            root = searchNode(root, x, y, z); 
        }
        OctreeNode *begin(){
            return findMinNode(); 
        }
        OctreeNode *end(){
            return findMaxNode(); 
        }
        
        OctreeNode *getRoot(){
            return root; 
        }

        OctreeNode &getRoot_reference(){
            return *root; // return root as a memory reference. 
        }     
};

const float epsilon = 0.0125; // epsilon to compare two floats. this value depends on the resolution we consider. (resolution /4 or lower )

// ==============================================================================================================================

//using namespace std; 

#define THREADS_PER_BLOCK 256		// the optimal value is number of cuda cores, if (#cuda cores < max th.pr.blck). 256 for TX2
static int NUM_OF_BLOCKS = 1; 

__device__ const float resolution = 0.05f; 	// Resolution of 5 cm
__device__ const float half_resolution = resolution/2; // the half of the resolution. this value is used in calculations 
 
const float max_sensor_radius = 3.00f; 	// scan data further than this modulus value will not be considered. 
const float max_free_radius = 15.00f;   // scan data further than this will not be considered for freespace calculation 


__global__ void generate_node_arrays(float* x, float* y, float* z, float* x_camera_array, float* y_camera_array, float* z_camera_array, 
    float* x_result, float* y_result, float* z_result, float* x_round, float* y_round, float* z_round,
	float x_trans, float y_trans, float z_trans, double sin_a, double sin_b, double sin_g, double cos_a, double cos_b, double cos_g){
	
	int index = threadIdx.x + blockIdx.x * blockDim.x;
	int steps = max_sensor_radius/resolution; 
       
    // mark free space 
    float x1 = 0.025f, y1 = 0.025f, z1 = 0.025f; //this is the starting point of all 
    float x2 = x_camera_array[index], y2 = y_camera_array[index], z2 = z_camera_array[index];   // this is the end point for free scans 

    if( (fabs(x[index]) < max_sensor_radius) and (fabs(y[index]) < max_sensor_radius) and (fabs(z[index]) < max_sensor_radius)){
        x2 = x[index]; 
        y2 = y[index];
        z2 = z[index]; 
        // the obstacle points can be marked for voxel centers. 
        float x_temp = x[index]*cos_a*cos_b + y[index]*cos_a*sin_b*sin_g - y[index]*sin_a*cos_g + z[index]*cos_a*sin_b*cos_g + z[index]*sin_a*sin_g + x_trans;  
        float y_temp = x[index]*sin_a*cos_b + y[index]*sin_a*sin_b*sin_g + y[index]*cos_a*cos_g + z[index]*sin_a*sin_b*cos_g - z[index]*cos_a*sin_g + y_trans; 
        float z_temp = x[index]*sin_b*-1 + y[index]*cos_b*sin_g + z[index]*cos_b*cos_g + z_trans; 
        
        x_round[index] = (ceilf(x_temp / resolution))*0.05 - half_resolution;
        y_round[index] = (ceilf(y_temp / resolution))*0.05 - half_resolution; 
        z_round[index] = (ceilf(z_temp / resolution))*0.05 - half_resolution;

    }else{
        x_round[index] = 0.00f;
        y_round[index] = 0.00f; 
        z_round[index] = 0.00f;   
    }
                    
    // start ray casting 

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
        //std::cout<<"X is driving axis"; 
        
        float py = 2 * dy - dx; 
        float pz = 2 * dz - dx;
        int sub_index = 0; 
        while (abs(x1-x2)>resolution/2 && (sub_index < steps)){
            
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
            
            float x_free_temp = x1*cos_a*cos_b + y1*cos_a*sin_b*sin_g - y1*sin_a*cos_g + z1*cos_a*sin_b*cos_g + z1*sin_a*sin_g + x_trans;  
            float y_free_temp = x1*sin_a*cos_b + y1*sin_a*sin_b*sin_g + y1*cos_a*cos_g + z1*sin_a*sin_b*cos_g - z1*cos_a*sin_g + y_trans; 
            float z_free_temp = x1*sin_b*-1 + y1*cos_b*sin_g + z1*cos_b*cos_g + z_trans; 
            
            x_result[index*steps + sub_index] = (ceil(x_free_temp / resolution))*0.05 - half_resolution;
            y_result[index*steps + sub_index] = (ceil(y_free_temp / resolution))*0.05 - half_resolution;  
            z_result[index*steps + sub_index] = (ceil(z_free_temp / resolution))*0.05 - half_resolution; 
            sub_index +=1; 
            
        }
        
        for(int j=sub_index; j<steps; j++){
            x_result[index*steps + j] = 0.00f; 
            y_result[index*steps + j] = 0.00f; 
            z_result[index*steps + j] = 0.00f; 
        }
    }
    else if(dy >= dx and dy >= dz){
        // Y axis is the driving axis
        float px = 2 * dx - dy; 
        float pz = 2 * dz - dy; 
        int sub_index = 0;
        while (abs(y1-y2)>resolution/2 && (sub_index < steps)){ 
            //std::cout<<x1<<"\t"<<y1<<"\t"<<z1<<std::endl;
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

            float x_free_temp = x1*cos_a*cos_b + y1*cos_a*sin_b*sin_g - y1*sin_a*cos_g + z1*cos_a*sin_b*cos_g + z1*sin_a*sin_g + x_trans;  
            float y_free_temp = x1*sin_a*cos_b + y1*sin_a*sin_b*sin_g + y1*cos_a*cos_g + z1*sin_a*sin_b*cos_g - z1*cos_a*sin_g + y_trans; 
            float z_free_temp = x1*sin_b*-1 + y1*cos_b*sin_g + z1*cos_b*cos_g + z_trans; 
            
            x_result[index*steps + sub_index] = (ceil(x_free_temp / resolution))*0.05 - half_resolution;
            y_result[index*steps + sub_index] = (ceil(y_free_temp / resolution))*0.05 - half_resolution;  
            z_result[index*steps + sub_index] = (ceil(z_free_temp / resolution))*0.05 - half_resolution; 
            sub_index +=1; 
        }
        
        for(int j=sub_index; j<steps; j++){
            x_result[index*steps + j] = 0.00f; 
            y_result[index*steps + j] = 0.00f; 
            z_result[index*steps + j] = 0.00f; 
        } 
    }
    else{
        // Z axis is the driving axis
        
        float py = 2*dy - dz;       // slope error 
        float px = 2*dx - dz; 
        int sub_index = 0;
        while(abs(z1-z2)>resolution/2 && (sub_index < steps)){
            //std::cout<<x1<<"\t"<<y1<<"\t"<<z1<<std::endl;
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
            
            float x_free_temp = x1*cos_a*cos_b + y1*cos_a*sin_b*sin_g - y1*sin_a*cos_g + z1*cos_a*sin_b*cos_g + z1*sin_a*sin_g + x_trans;  
            float y_free_temp = x1*sin_a*cos_b + y1*sin_a*sin_b*sin_g + y1*cos_a*cos_g + z1*sin_a*sin_b*cos_g - z1*cos_a*sin_g + y_trans; 
            float z_free_temp = x1*sin_b*-1 + y1*cos_b*sin_g + z1*cos_b*cos_g + z_trans; 
            
            x_result[index*steps + sub_index] = (ceil(x_free_temp / resolution))*0.05 - half_resolution;
            y_result[index*steps + sub_index] = (ceil(y_free_temp / resolution))*0.05 - half_resolution;  
            z_result[index*steps + sub_index] = (ceil(z_free_temp / resolution))*0.05 - half_resolution; 
            
            sub_index +=1; 
        }
        
        for(int j=sub_index; j<steps; j++){
            x_result[index*steps + j] = 0.00f; 
            y_result[index*steps + j] = 0.00f; 
            z_result[index*steps + j] = 0.00f; 
        } 
    }	           	
}

Octree tree(half_resolution);  

float *x_end, *y_end, *z_end; // endpoints 
float *d_x_end, *d_y_end, *d_z_end;  //device arrays 
static bool setup_grid = false;
static bool end_freed = false; 

int cudamain(sensor_msgs::PointCloud2 point_cloud_std, nav_msgs::Odometry odom_message_std, int size){ 
    double starttotal, endtotal; 
	starttotal = clock();
	//make_range_array(resolution, max_sensor_radius); 
	int array_size = size; 	 
    
    
    double start1, end1; 
	start1 = clock();
    // end4 = clock();
	// double time4 = (double)(end4 - start4);
    
    // convert quaternion orientation into roll, pitch, yaw representation 
	//double roll, pitch, yaw;
	double roll, pitch, yaw; 
	tf::Quaternion quat;
    tf::quaternionMsgToTF(odom_message_std.pose.pose.orientation, quat);	
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
	float x_position = (float) odom_message_std.pose.pose.position.x; 
	float y_position = (float) odom_message_std.pose.pose.position.y; 
	float z_position = (float) odom_message_std.pose.pose.position.z; 

	double sin_gamma = sin(roll); 
	double sin_beta = sin(pitch); 
	double sin_alpha = sin(yaw); 
	double cos_gamma = cos(roll); 
	double cos_beta = cos(pitch); 
	double cos_alpha = cos(yaw); 

	
	//std::cout<<"alpha: "<<yaw<<"  sin alpha: "<<sin_alpha<<std::endl;
	//std::cout<<"\n\n"; 

	int counter = 0; 
	int effective_point_count = 0; 
	//declare the arrray sets before reading the point cloud values 
	
	float *x, *y, *z; // for allocating position values of the points 
	float *x_rounded, *y_rounded, *z_rounded; 							// the intermediate results after rounding off the x, y, z, original values to the resolution 
	u_int8_t *r, *g, *b; // for color values of the point cloud 
	float *x_free, *y_free, *z_free;

	int size_position = array_size * sizeof(float);
	int size_color = array_size * sizeof(u_int8_t);
    int max_step_count = max_sensor_radius/resolution; 
    int max_free_voxel_count = array_size * max_sensor_radius/resolution; 
	int size_free_array = max_free_voxel_count * sizeof(float); 

	x = (float *)malloc( size_position );
   	y = (float *)malloc( size_position );
	z = (float *)malloc( size_position );

	x_rounded = (float *)malloc( size_position );
   	y_rounded = (float *)malloc( size_position );
	z_rounded = (float *)malloc( size_position );

	r = (u_int8_t *)malloc( size_color );
    g = (u_int8_t *)malloc( size_color );
	b = (u_int8_t *)malloc( size_color );
    
    x_free = (float *)malloc( size_free_array );
	y_free = (float *)malloc( size_free_array );
    z_free = (float *)malloc( size_free_array );
    
    //run this code once 
    if(!setup_grid){
        cout<<"setting up once"<<endl;
        x_end = (float *)malloc( size_position );
   	    y_end = (float *)malloc( size_position );
	    z_end = (float *)malloc( size_position );
        cudaMalloc( (void **) &d_x_end, size_position);
        cudaMalloc( (void **) &d_y_end, size_position);
        cudaMalloc( (void **) &d_z_end, size_position);
        
        ifstream ifile;
        ifile.open ("free_endpoints.txt", ios::in);
        float x_map, y_map, z_map;  
        int i=0;   
        while(ifile >>x_map>>y_map>>z_map){
            x_end[i] = x_map; 
            y_end[i] = y_map;
            z_end[i] = z_map; 
            i+=1; 
        }
        ifile.close();

        cudaMemcpy( d_x_end, x_end, size_position, cudaMemcpyHostToDevice );
        cudaMemcpy( d_y_end, y_end, size_position, cudaMemcpyHostToDevice );
        cudaMemcpy( d_z_end, z_end, size_position, cudaMemcpyHostToDevice );

        setup_grid = true; // not to run this code again 
    }

    
    // positional data vector generation 
	for(sensor_msgs::PointCloud2ConstIterator<float> it(point_cloud_std, "x"); it!=it.end(); ++it){
		y[counter] = it[0] * -1; 
		z[counter] = it[1] * -1;
		x[counter] = it[2];
		counter+=1;  
		
	}
	counter = 0; 
    for(sensor_msgs::PointCloud2ConstIterator<u_int8_t> it_color(point_cloud_std, "rgb"); it_color!=it_color.end(); ++it_color){
		b[counter] = unsigned(it_color[0]);	
		g[counter] = unsigned(it_color[1]);	
		r[counter] = unsigned(it_color[2]); 
		counter+=1; 
	}
	counter = 0; 
    
    // double start1, end1; 
	// start1 = clock();
    end1 = clock();
	double time1 = (double)(end1 - start1);
    
	// Adjust the number of blocks to be a whole number. 
	NUM_OF_BLOCKS = (array_size + THREADS_PER_BLOCK-1)/THREADS_PER_BLOCK; 


	//The cuda device variables 
	float *d_x, *d_y, *d_z;
	float *d_x_rounded, *d_y_rounded, *d_z_rounded; 
	float *d_x_free, *d_y_free, *d_z_free; 
    
    cudaMalloc( (void **) &d_x, size_position);
	cudaMalloc( (void **) &d_y, size_position);
	cudaMalloc( (void **) &d_z, size_position);
	
	cudaMalloc( (void **) &d_x_rounded, size_position);
	cudaMalloc( (void **) &d_y_rounded, size_position);
    cudaMalloc( (void **) &d_z_rounded, size_position);
    cudaMalloc( (void **) &d_x_free, size_free_array);
	cudaMalloc( (void **) &d_y_free, size_free_array);
	cudaMalloc( (void **) &d_z_free, size_free_array);

    cudaMemcpy( d_x, x, size_position, cudaMemcpyHostToDevice );
	cudaMemcpy( d_y, y, size_position, cudaMemcpyHostToDevice );
	cudaMemcpy( d_z, z, size_position, cudaMemcpyHostToDevice );
	
	// GPU process START---------------------------------------------------------------------------------------------------------------------------------------------------------------
	//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	
	double start2, end2; 
	start2 = clock();

    generate_node_arrays<<<NUM_OF_BLOCKS, THREADS_PER_BLOCK>>>(d_x, d_y, d_z, d_x_end, d_y_end, d_z_end,  
                                                                d_x_free, d_y_free, d_z_free, d_x_rounded, d_y_rounded, d_z_rounded, 
        x_position, y_position, z_position, sin_alpha, sin_beta, sin_gamma, cos_alpha, cos_beta, cos_gamma);
    
    // double start1, end1; 
	// start1 = clock();
    end2 = clock();
    double time2 = (double)(end2 - start2);
    
	cudaMemcpy( x_rounded, d_x_rounded, size_position, cudaMemcpyDeviceToHost );
	cudaMemcpy( y_rounded, d_y_rounded, size_position, cudaMemcpyDeviceToHost );
	cudaMemcpy( z_rounded, d_z_rounded, size_position, cudaMemcpyDeviceToHost );
    
    cudaMemcpy( x_free, d_x_free, size_free_array, cudaMemcpyDeviceToHost );
	cudaMemcpy( y_free, d_y_free, size_free_array, cudaMemcpyDeviceToHost );
	cudaMemcpy( z_free, d_z_free, size_free_array, cudaMemcpyDeviceToHost );

	
	// GPU process END----------------------------------------------------------------------------------------------------------------------------------------------------------------
	//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	
    current = new OctreeNode;   //initiating OctreeNodes
    parent = new OctreeNode; 
    
    double start3, end3; 
	start3 = clock();
    
    for(int i=0; i<array_size; i++){
        if(x_rounded[i] != 0.00f){
            tree.insert(x_rounded[i], y_rounded[i], z_rounded[i], r[i], g[i], b[i]); 
        }
    }
    
    end3 = clock();
    double time3 = (double)(end3 - start3);
    double start4, end4; 
	start4 = clock();
    
    for(int i=0; i<max_free_voxel_count; i+=max_step_count){
        for(int j=0; j<max_step_count-1; j++){
            int k = i+j; 
            if(x_free[k+1] == 0.00f){
                //std::cout<<"last voxel: "<<k+1<<endl; 
                break; 
            }else{
                //if(j ==59){std::cout<<"at the edge"<<std::endl; }
                tree.insert_free(x_free[k], y_free[k], z_free[k]);  
            }
        }
    }		
    
	// double start1, end1; 
	// start1 = clock();
    end4 = clock();
    double time4 = (double)(end4 - start4);
    
    double start5, end5; 
	start5 = clock();
    // end1 = clock();
	// double time1 = (double)(end1 - start1);
    
    std::queue<OctreeNode*> temp_nodes; 
	temp_nodes = leaf_nodes; 
    
    std::string map_file_name = "octree_nodes.txt";
    //std::string floor_file_name = "nav_nodes.txt";
    ofstream offile;    
    //ofstream navfile; 

    offile.open(map_file_name.c_str(), ios::trunc);
    //navfile.open(floor_file_name.c_str(), ios::trunc);
    
	if(offile.is_open() /*&& navfile.is_open()*/) { 
		while(!temp_nodes.empty()){
            current = temp_nodes.front(); 
            if(current->Occ > 0){
				offile<<current->X<<"\t"<<current->Y<<"\t"<<current->Z<<"\t"<<unsigned(current->R)<<"\t"<<unsigned(current->G)<<"\t"<<unsigned(current->B)<<std::endl;  
            }
            /*if(current->Z == 0.075f){
                navfile<<current->X<<"\t"<<current->Y<<"\t"<<current->Z<<"\t"<<signed(current->Occ)<<std::endl;
            }*/
            temp_nodes.pop(); 
        }				
	}
	
    offile.close();

    end5 = clock();
	double time5 = (double)(end5 - start5);
    
    

    free(x);
    free(y);
	free(z);
	free(r);
	free(g);
	free(b);
	free(x_rounded); 
	free(y_rounded); 
    free(z_rounded);
    
    free(x_free); 
    free(y_free); 
    free(z_free); 
    
    if(!end_freed){
        free(x_end); 
        free(y_end); 
        free(z_end);
        end_freed = true; 
    }
	
	cudaFree( d_x );
	cudaFree( d_y );
	cudaFree( d_z );
	cudaFree(d_x_rounded); 
	cudaFree(d_y_rounded); 
	cudaFree(d_z_rounded); 
    cudaFree(d_x_free); 
    cudaFree(d_y_free); 
    cudaFree(d_z_free); 
    
    endtotal = clock();
    double timetotal = (double)(endtotal - starttotal); 
    
    std::cout<<time1<<"\t"<<time2<<"\t"<<time3<<"\t"<<time4<<"\t"<<time5<<"\t"<<timetotal<<endl; 

	return EXIT_SUCCESS; 	
}
