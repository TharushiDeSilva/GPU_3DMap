//K. T. D. S. De Silva ---- University of Moratuwa 
// comapare traditional octree and morton codes
//---------------------------------------------------------------
 
#include <iostream>
#include <bits/stdc++.h> 
#include <stdint.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <time.h>
#include <string>
#include <cmath> 
#include "math.h"
#include <cstdlib>
#include <ctime>
#include <stdio.h>
#include <boost/unordered_map.hpp>
#include <iterator>

using namespace std; 
float res_list_5cm[] = {3276.80, 1638.40, 819.20, 409.60, 204.80, 102.40, 51.20, 25.60, 12.80, 6.40, 3.20, 1.60, 0.80, 0.40, 0.20, 0.10, 0.05, 0.025}; // in meters
/*
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
            //std::cout<<"destroyed"<<endl; 
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
*/
#define THREADS_PER_BLOCK 256		// the optimal value is number of cuda cores, if (#cuda cores < max th.pr.blck). 256 for TX2
static int NUM_OF_BLOCKS = 1; 

__device__ const int half_axis_length = 32768; 
__device__ const float resolution = 0.05f; 	// Resolution of 5 cm
__device__ const float half_resolution = resolution/2; // the half of the resolution. this value is used in calculations 
 
const float max_sensor_radius = 3.00f; 	// scan data further than this modulus value will not be considered. 

__global__ void generate_codes(float* x, float* y, float* z, uint8_t* r, uint8_t* g, uint8_t* b, uint64_t* morton_code, uint32_t* rgb_result){
     
    int index = threadIdx.x + blockIdx.x * blockDim.x;
    
    // convert the float into an unsigned integer 
    uint64_t x_original = uint64_t(half_axis_length + ceilf(x[index]/ resolution) -1 );
    uint64_t y_original = uint64_t(half_axis_length + ceilf(y[index] / resolution) -1 );
    uint64_t z_original = uint64_t(half_axis_length + ceilf(z[index] / resolution) -1 );

    // x_oroginal = 0000000000000000 0000000000000000 0000000000000000 1111111111111111
    uint64_t x_temp = (x_original<<32) & (0b0000000000000000100000000000000000000000000000000000000000000000); 
    // x_temp = 0000000000000000 1000000000000000 0000000000000000 0000000000000000
    x_temp = ((x_original<<30) & (0b0000000000000000000100000000000000000000000000000000000000000000)) | x_temp; 
    // x_temp = 0000000000000000 1001000000000000 0000000000000000 0000000000000000
    x_temp = ((x_original<<28) & (0b0000000000000000000000100000000000000000000000000000000000000000)) | x_temp;
    // x_temp = 0000000000000000 1001001000000000 0000000000000000 0000000000000000
    x_temp = ((x_original<<26) & (0b0000000000000000000000000100000000000000000000000000000000000000)) | x_temp;
    // x_temp = 0000000000000000 1001001001000000 0000000000000000 0000000000000000
    x_temp = ((x_original<<24) & (0b0000000000000000000000000000100000000000000000000000000000000000)) | x_temp;
    // x_temp = 0000000000000000 1001001001001000 0000000000000000 0000000000000000
    x_temp = ((x_original<<22) & (0b0000000000000000000000000000000100000000000000000000000000000000)) | x_temp;
    // x_temp = 0000000000000000 1001001001001001 0000000000000000 0000000000000000
    x_temp = ((x_original<<20) & (0b0000000000000000000000000000000000100000000000000000000000000000)) | x_temp;
    // x_temp = 0000000000000000 1001001001001001 0010000000000000 0000000000000000
    x_temp = ((x_original<<18) & (0b0000000000000000000000000000000000000100000000000000000000000000)) | x_temp;
    // x_temp = 0000000000000000 1001001001001001 0010010000000000 0000000000000000
    x_temp = ((x_original<<16) & (0b0000000000000000000000000000000000000000100000000000000000000000)) | x_temp;
    // x_temp = 0000000000000000 1001001001001001 0010010010000000 0000000000000000
    x_temp = ((x_original<<14) & (0b0000000000000000000000000000000000000000000100000000000000000000)) | x_temp;
    // x_temp = 0000000000000000 1001001001001001 0010010010010000 0000000000000000
    x_temp = ((x_original<<12) & (0b0000000000000000000000000000000000000000000000100000000000000000)) | x_temp;
    // x_temp = 0000000000000000 1001001001001001 0010010010010010 0000000000000000
    x_temp = ((x_original<<10) & (0b0000000000000000000000000000000000000000000000000100000000000000)) | x_temp;
    // x_temp = 0000000000000000 1001001001001001 0010010010010010 0100000000000000
    x_temp = ((x_original<<8) &  (0b0000000000000000000000000000000000000000000000000000100000000000)) | x_temp;
    // x_temp = 0000000000000000 1001001001001001 0010010010010010 0100100000000000
    x_temp = ((x_original<<6) &  (0b0000000000000000000000000000000000000000000000000000000100000000)) | x_temp;
    // x_temp = 0000000000000000 1001001001001001 0010010010010010 0100100100000000
    x_temp = ((x_original<<4) &  (0b0000000000000000000000000000000000000000000000000000000000100000)) | x_temp;
    // x_temp = 0000000000000000 1001001001001001 0010010010010010 0100100100100000
    x_temp = ((x_original<<2) &  (0b0000000000000000000000000000000000000000000000000000000000000100)) | x_temp;
    // x_temp = 0000000000000000 1001001001001001 0010010010010010 0100100100100100
    x_temp = (x_temp>>2); 

    uint64_t m_code = x_temp; 

    // y_oroginal = 0000000000000000 0000000000000000 0000000000000000 1111111111111111
    uint64_t y_temp = (y_original<<32) & (0b0000000000000000100000000000000000000000000000000000000000000000); 
    // y_temp = 0000000000000000 1000000000000000 0000000000000000 0000000000000000
    y_temp = ((y_original<<30) & (0b0000000000000000000100000000000000000000000000000000000000000000)) | y_temp; 
    // y_temp = 0000000000000000 1001000000000000 0000000000000000 0000000000000000
    y_temp = ((y_original<<28) & (0b0000000000000000000000100000000000000000000000000000000000000000)) | y_temp;
    // y_temp = 0000000000000000 1001001000000000 0000000000000000 0000000000000000
    y_temp = ((y_original<<26) & (0b0000000000000000000000000100000000000000000000000000000000000000)) | y_temp;
    // y_temp = 0000000000000000 1001001001000000 0000000000000000 0000000000000000
    y_temp = ((y_original<<24) & (0b0000000000000000000000000000100000000000000000000000000000000000)) | y_temp;
    // y_temp = 0000000000000000 1001001001001000 0000000000000000 0000000000000000
    y_temp = ((y_original<<22) & (0b0000000000000000000000000000000100000000000000000000000000000000)) | y_temp;
    // y_temp = 0000000000000000 1001001001001001 0000000000000000 0000000000000000
    y_temp = ((y_original<<20) & (0b0000000000000000000000000000000000100000000000000000000000000000)) | y_temp;
    // y_temp = 0000000000000000 1001001001001001 0010000000000000 0000000000000000
    y_temp = ((y_original<<18) & (0b0000000000000000000000000000000000000100000000000000000000000000)) | y_temp;
    // y_temp = 0000000000000000 1001001001001001 0010010000000000 0000000000000000
    y_temp = ((y_original<<16) & (0b0000000000000000000000000000000000000000100000000000000000000000)) | y_temp;
    // y_temp = 0000000000000000 1001001001001001 0010010010000000 0000000000000000
    y_temp = ((y_original<<14) & (0b0000000000000000000000000000000000000000000100000000000000000000)) | y_temp;
    // y_temp = 0000000000000000 1001001001001001 0010010010010000 0000000000000000
    y_temp = ((y_original<<12) & (0b0000000000000000000000000000000000000000000000100000000000000000)) | y_temp;
    // y_temp = 0000000000000000 1001001001001001 0010010010010010 0000000000000000
    y_temp = ((y_original<<10) & (0b0000000000000000000000000000000000000000000000000100000000000000)) | y_temp;
    // y_temp = 0000000000000000 1001001001001001 0010010010010010 0100000000000000
    y_temp = ((y_original<<8) &  (0b0000000000000000000000000000000000000000000000000000100000000000)) | y_temp;
    // y_temp = 0000000000000000 1001001001001001 0010010010010010 0100100000000000
    y_temp = ((y_original<<6) &  (0b0000000000000000000000000000000000000000000000000000000100000000)) | y_temp;
    // y_temp = 0000000000000000 1001001001001001 0010010010010010 0100100100000000
    y_temp = ((y_original<<4) &  (0b0000000000000000000000000000000000000000000000000000000000100000)) | y_temp;
    // y_temp = 0000000000000000 1001001001001001 0010010010010010 0100100100100000
    y_temp = ((y_original<<2) &  (0b0000000000000000000000000000000000000000000000000000000000000100)) | y_temp;
    // y_temp = 0000000000000000 1001001001001001 0010010010010010 0100100100100100
    y_temp = (y_temp>>1); 

    m_code = (m_code | y_temp); 

    // z_oroginal = 0000000000000000 0000000000000000 0000000000000000 1111111111111111
    uint64_t z_temp = (z_original<<32) & (0b0000000000000000100000000000000000000000000000000000000000000000); 
    // z_temp = 0000000000000000 1000000000000000 0000000000000000 0000000000000000
    z_temp = ((z_original<<30) & (0b0000000000000000000100000000000000000000000000000000000000000000)) | z_temp; 
    // z_temp = 0000000000000000 1001000000000000 0000000000000000 0000000000000000
    z_temp = ((z_original<<28) & (0b0000000000000000000000100000000000000000000000000000000000000000)) | z_temp;
    // z_temp = 0000000000000000 1001001000000000 0000000000000000 0000000000000000
    z_temp = ((z_original<<26) & (0b0000000000000000000000000100000000000000000000000000000000000000)) | z_temp;
    // z_temp = 0000000000000000 1001001001000000 0000000000000000 0000000000000000
    z_temp = ((z_original<<24) & (0b0000000000000000000000000000100000000000000000000000000000000000)) | z_temp;
    // z_temp = 0000000000000000 1001001001001000 0000000000000000 0000000000000000
    z_temp = ((z_original<<22) & (0b0000000000000000000000000000000100000000000000000000000000000000)) | z_temp;
    // z_temp = 0000000000000000 1001001001001001 0000000000000000 0000000000000000
    z_temp = ((z_original<<20) & (0b0000000000000000000000000000000000100000000000000000000000000000)) | z_temp;
    // z_temp = 0000000000000000 1001001001001001 0010000000000000 0000000000000000
    z_temp = ((z_original<<18) & (0b0000000000000000000000000000000000000100000000000000000000000000)) | z_temp;
    // z_temp = 0000000000000000 1001001001001001 0010010000000000 0000000000000000
    z_temp = ((z_original<<16) & (0b0000000000000000000000000000000000000000100000000000000000000000)) | z_temp;
    // z_temp = 0000000000000000 1001001001001001 0010010010000000 0000000000000000
    z_temp = ((z_original<<14) & (0b0000000000000000000000000000000000000000000100000000000000000000)) | z_temp;
    // z_temp = 0000000000000000 1001001001001001 0010010010010000 0000000000000000
    z_temp = ((z_original<<12) & (0b0000000000000000000000000000000000000000000000100000000000000000)) | z_temp;
    // z_temp = 0000000000000000 1001001001001001 0010010010010010 0000000000000000
    z_temp = ((z_original<<10) & (0b0000000000000000000000000000000000000000000000000100000000000000)) | z_temp;
    // z_temp = 0000000000000000 1001001001001001 0010010010010010 0100000000000000
    z_temp = ((z_original<<8) &  (0b0000000000000000000000000000000000000000000000000000100000000000)) | z_temp;
    // z_temp = 0000000000000000 1001001001001001 0010010010010010 0100100000000000
    z_temp = ((z_original<<6) &  (0b0000000000000000000000000000000000000000000000000000000100000000)) | z_temp;
    // z_temp = 0000000000000000 1001001001001001 0010010010010010 0100100100000000
    z_temp = ((z_original<<4) &  (0b0000000000000000000000000000000000000000000000000000000000100000)) | z_temp;
    // z_temp = 0000000000000000 1001001001001001 0010010010010010 0100100100100000
    z_temp = ((z_original<<2) &  (0b0000000000000000000000000000000000000000000000000000000000000100)) | z_temp;
    // z_temp = 0000000000000000 1001001001001001 0010010010010010 0100100100100100
    
    m_code = (m_code | z_temp); 
    
    morton_code[index] = m_code; 

    // calculate rgbo value. 
    uint32_t r_temp = r[index] & 0b00000000000000000000000011111111; 
    uint32_t g_temp = g[index] & 0b00000000000000000000000011111111; 
    uint32_t b_temp = b[index] & 0b00000000000000000000000011111111; 
    uint32_t rgb_code = (r_temp<<24) | (g_temp<<16) | (b_temp<<8) | 0b00000000000000000000000000010001; // occ for obstcale nodes 
    rgb_result[index] = rgb_code; 
}
/*
__global__ void generate_voxels(float* x, float* y, float* z, float* x_result, float* y_result, float* z_result){
    
    int index = threadIdx.x + blockIdx.x * blockDim.x;
			
    x_result[index] = (ceilf(x[index] / resolution))*0.05 - half_resolution;
    y_result[index] = (ceilf(y[index] / resolution))*0.05 - half_resolution; 
    z_result[index] = (ceilf(z[index] / resolution))*0.05 - half_resolution;

}
*/

extern boost::unordered::unordered_map<uint64_t, uint32_t> octree;

int cudamain(int size, float low, float high){ 
    
    // Octree tree(half_resolution);
    
    // current = new OctreeNode;   //initiating OctreeNodes
    // parent = new OctreeNode;
    
    //make_range_array(resolution, max_sensor_radius); 
	int array_size = size;  	 
    // float LO = -2.9342446f; 
    // float HI = 2.94333346f; 
    float LO = low; 
    float HI = high;

    float *x, *y, *z; // for allocating position values of the points 
    float *x_rounded, *y_rounded, *z_rounded;
	uint64_t *mcode_arr; 	// the intermediate results after rounding off the x, y, z, original values to the resolution 
	uint8_t *r, *g, *b; // for color values of the point cloud 
	uint32_t *rgbo_arr; 

	int size_position = array_size * sizeof(float);
    int size_color = array_size * sizeof(uint8_t);
    int max_step_count = max_sensor_radius/resolution; 
    int size_morton_array = array_size * sizeof(uint64_t);
    int size_rgb = array_size * sizeof(uint32_t);
    
	x = (float *)malloc( size_position );
   	y = (float *)malloc( size_position );
	z = (float *)malloc( size_position );
    
    r = (uint8_t *)malloc( size_color );
    g = (uint8_t *)malloc( size_color );
    b = (uint8_t *)malloc( size_color );
    
    srand (static_cast <unsigned> (time(0)));
    
    //populate arrays
    for(int i=0; i<array_size; i++){
        float rx = LO + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(HI-LO)));
        x[i] = rx; 
        float ry = LO + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(HI-LO)));
        y[i] = ry; 
        float rz = LO + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(HI-LO)));
        z[i] = rz; 
    }
 
    int max = 255; int min = 0; 
    //populate arrays
    for(int i=0; i<array_size; i++){
        int rr = min + (rand() % static_cast<int>(max - min + 1)); 
        r[i] = uint8_t(rr); 
        int rg = min + (rand() % static_cast<int>(max - min + 1)); 
        g[i] = uint8_t(rg); 
        int rb = min + (rand() % static_cast<int>(max - min + 1)); 
        b[i] = uint8_t(rb); 
    }

    // for(int i=0; i<array_size; i++){
    //     std::cout<<i<<"\t"<<x[i]<<"\t"<<y[i]<<"\t"<<z[i]<<"\t"<<unsigned(r[i])<<"\t"<<unsigned(g[i])<<"\t"<<unsigned(b[i])<<std::endl;
    // }

    
    x_rounded = (float *)malloc( size_position );
   	y_rounded = (float *)malloc( size_position );
	z_rounded = (float *)malloc( size_position );
    
	mcode_arr = (uint64_t *)malloc( size_morton_array );
    rgbo_arr = (uint32_t *)malloc( size_rgb );

	NUM_OF_BLOCKS = (array_size + THREADS_PER_BLOCK-1)/THREADS_PER_BLOCK; 

	//The cuda device variables 
    float *d_x, *d_y, *d_z;
    float *d_x_rounded, *d_y_rounded, *d_z_rounded;
    uint8_t *d_r, *d_g, *d_b; 
	uint64_t *d_mcode_arr; 
    uint32_t *d_rgbo_arr; 
    
	cudaMalloc( (void **) &d_x, size_position);
	cudaMalloc( (void **) &d_y, size_position);
    cudaMalloc( (void **) &d_z, size_position);

    cudaMalloc( (void **) &d_x_rounded, size_position);
	cudaMalloc( (void **) &d_y_rounded, size_position);
    cudaMalloc( (void **) &d_z_rounded, size_position);
    
    cudaMalloc( (void **) &d_r, size_color);
	cudaMalloc( (void **) &d_g, size_color);
	cudaMalloc( (void **) &d_b, size_color);
    
    // the kernel outputs
    cudaMalloc( (void **) &d_mcode_arr, size_morton_array);
	cudaMalloc( (void **) &d_rgbo_arr, size_rgb);

    double start1, end1; 
    start1 = clock();

    cudaMemcpy( d_x, x, size_position, cudaMemcpyHostToDevice );
	cudaMemcpy( d_y, y, size_position, cudaMemcpyHostToDevice );
    cudaMemcpy( d_z, z, size_position, cudaMemcpyHostToDevice );
    
    end1 = clock();
    double time1 = (double)(end1 - start1);
    
    double start2, end2; 
    start2 = clock();

    cudaMemcpy( d_r, r, size_color, cudaMemcpyHostToDevice );
    cudaMemcpy( d_g, g, size_color, cudaMemcpyHostToDevice );
    cudaMemcpy( d_b, b, size_color, cudaMemcpyHostToDevice );
	
	// MORTON CODE BASED OCTREE ALGORITHM ---------------------------------------------------------------------------------------------------------- 
    generate_codes<<<NUM_OF_BLOCKS, THREADS_PER_BLOCK>>>(d_x, d_y, d_z, d_r, d_g, d_b, d_mcode_arr, d_rgbo_arr);
	                                 
    cudaMemcpy( mcode_arr, d_mcode_arr, size_morton_array, cudaMemcpyDeviceToHost );
    cudaMemcpy( rgbo_arr, d_rgbo_arr, size_rgb, cudaMemcpyDeviceToHost );
	 
    for(int i=0; i<array_size; i++){
        
        uint64_t morton_code = mcode_arr[i]; 
        boost::unordered::unordered_map<uint64_t, uint32_t>::iterator itr;  // to find the key 
        itr = octree.find(morton_code);
        if (itr == octree.end()){
            uint32_t value = rgbo_arr[i]; 
            octree.insert(std::make_pair<uint64_t, uint32_t>(morton_code, value));
            
        }else{
            uint32_t rgbo_map = (itr->second); 
            uint32_t occ_old = rgbo_map & 0b00000000000000000000000000111111;
            rgbo_map = rgbo_arr[i] & 0b11111111111111111111111100000000;             // this is the final variable 
            
            if(occ_old == 0b00000000000000000000000000001100){  // previous value is 12, adding 3 will make it unknown 
                rgbo_map = rgbo_map | 0b00000000000000000000000000010001; // so make it 17
            }else if(occ_old < 0b00000000000000000000000000100000){
                // value less than 32. max occupancy 
                occ_old +=4;                                        // obstacle nodes are more critical 
                rgbo_map = rgbo_map | occ_old; 
            }else{
                // no use from this case
            }
            itr->second = rgbo_map; 
        }
           
    }
    end2 = clock();
    double time2 = (double)(end2 - start2);

    // pointer based octree process started --------------------------------------------------------------------------------------------------
    /*
    double start3, end3; 
    start3 = clock();

    generate_voxels<<<NUM_OF_BLOCKS, THREADS_PER_BLOCK>>>(d_x, d_y, d_z, d_x_rounded, d_y_rounded, d_z_rounded);
    
    cudaMemcpy( x_rounded, d_x_rounded, size_position, cudaMemcpyDeviceToHost );
    cudaMemcpy( y_rounded, d_y_rounded, size_position, cudaMemcpyDeviceToHost );
    cudaMemcpy( z_rounded, d_z_rounded, size_position, cudaMemcpyDeviceToHost );
    
    for(int i=0; i<array_size; i++){
        if(x_rounded[i] != 0.00f){
            tree.insert(x_rounded[i], y_rounded[i], z_rounded[i], r[i], g[i], b[i]); 
        }
    }
    
    end3 = clock();
    double time3 = (double)(end3 - start3);
    */

    //std::cout<<array_size<<"\t"<<time1+time2<<"\t"<<time1+time3<<std::endl;
    
    free(x);
    free(y);
	free(z);
	free(r);
	free(g);
	free(b); 
    free(mcode_arr); 
    free(rgbo_arr);
    free(x_rounded);
    free(y_rounded);
    free(z_rounded);
    
	cudaFree( d_x );
	cudaFree( d_y );
    cudaFree( d_z );
    cudaFree( d_r );
    cudaFree( d_g );
    cudaFree( d_b );
    cudaFree(d_mcode_arr); 
    cudaFree(d_rgbo_arr); 
    cudaFree(d_x_rounded);
    cudaFree(d_y_rounded);
    cudaFree(d_z_rounded); 
   
	return EXIT_SUCCESS; 	
}
