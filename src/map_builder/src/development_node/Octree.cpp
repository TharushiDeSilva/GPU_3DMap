#include <iostream>
#include <fstream>
#include <bits/stdc++.h> 
#include <stdint.h>

using namespace std; 

float res_list_5cm[] = {3276.8, 1638.4, 819.2, 409.6, 204.8, 102.4, 51.2, 25.6, 12.8, 6.4, 3.2, 1.6, 0.8, 0.4, 0.2, 0.1, 0.05, 0.025}; // in meters
float res_list_1cm[] = {65536, 32768, 16384, 8192, 4096, 2048, 1024, 512, 256, 128, 64, 32, 16, 8, 4, 2, 1, 0.5}; // in cm 

struct OctreeNode{
    //for now we consider a depth of 16 for data nodes. max reolution = 65536 cm. float is enough 
    float X; 
    float Y; 
    float Z; 
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

};


OctreeNode *current;            // these are global varibale used to travel down the tree 
OctreeNode *parent; 
queue<OctreeNode*> leaf_nodes;

class Octree{
    OctreeNode *root; 
    
    void destroy(OctreeNode *node){
        if(node == NULL){
            return; 
        }else{
            if(node->BNE != NULL){
                destroy(node->BNE); 
                destroy(node->BSE);
                destroy(node->BSW); 
                destroy(node->BNW); 
                destroy(node->TNE); 
                destroy(node->TSE);
                destroy(node->TSW); 
                destroy(node->TNW); 
            } 
            //std::cout<<"deleting: "<<node->X<<"\t"<<node->Y<<"\t"<<node->Z<<endl; 
            delete node; 
        }
    } 

    void insertNode(float x, float y, float z, bool occupied){
        // insert this node at the depth of 16. where all the leaf nodes are 
        int depth = 0; //root level
        current = root; 
         
        while(depth < 16){
            //int depth = current->level; 
            if(current->BNE == NULL){ 
                //std::cout<<"in the level "<<depth<<"\tresolution: "<<current->X<<","<<current->Y<<","<<current->Z<<"Initializing next level..."<<std::endl;
                //std::cout<<"this level not initialized"<<endl;
                //means this level has not been initialized yet. 
                //Initialize the next depth of nodes
                OctreeNode *BNE_node = new OctreeNode; 
                BNE_node->X = current->X + res_list_5cm[depth+2]; 
                BNE_node->Y = current->Y + res_list_5cm[depth+2];
                BNE_node->Z = current->Z - res_list_5cm[depth+2];
                //BNE_node->level = depth+1; 
                BNE_node->Occ = 0; 
                current->BNE = BNE_node; 
                OctreeNode *BSE_node = new OctreeNode; 
                BSE_node->X = current->X + res_list_5cm[depth+2]; 
                BSE_node->Y = current->Y - res_list_5cm[depth+2];
                BSE_node->Z = current->Z - res_list_5cm[depth+2];
                //BSE_node->level = depth+1; 
                BSE_node->Occ = 0;
                current->BSE = BSE_node;
                OctreeNode *BSW_node = new OctreeNode; 
                BSW_node->X = current->X - res_list_5cm[depth+2]; 
                BSW_node->Y = current->Y - res_list_5cm[depth+2];
                BSW_node->Z = current->Z - res_list_5cm[depth+2];
                //BSW_node->level = depth+1; 
                BSW_node->Occ = 0;
                current->BSW = BSW_node;
                OctreeNode *BNW_node = new OctreeNode; 
                BNW_node->X = current->X - res_list_5cm[depth+2]; 
                BNW_node->Y = current->Y + res_list_5cm[depth+2];
                BNW_node->Z = current->Z - res_list_5cm[depth+2];
                //BNW_node->level = depth+1; 
                BNW_node->Occ = 0;
                current->BNW = BNW_node;
                OctreeNode *TNE_node = new OctreeNode; 
                TNE_node->X = current->X + res_list_5cm[depth+2]; 
                TNE_node->Y = current->Y + res_list_5cm[depth+2];
                TNE_node->Z = current->Z + res_list_5cm[depth+2];
                //TNE_node->level = depth+1; 
                TNE_node->Occ = 0; 
                current->TNE = TNE_node; 
                OctreeNode *TSE_node = new OctreeNode; 
                TSE_node->X = current->X + res_list_5cm[depth+2]; 
                TSE_node->Y = current->Y - res_list_5cm[depth+2];
                TSE_node->Z = current->Z + res_list_5cm[depth+2];
                //TSE_node->level = depth+1; 
                TSE_node->Occ = 0;
                current->TSE = TSE_node;
                OctreeNode *TSW_node = new OctreeNode; 
                TSW_node->X = current->X - res_list_5cm[depth+2]; 
                TSW_node->Y = current->Y - res_list_5cm[depth+2];
                TSW_node->Z = current->Z + res_list_5cm[depth+2];
                //TSW_node->level = depth+1; 
                TSW_node->Occ = 0;
                current->TSW = TSW_node;
                OctreeNode *TNW_node = new OctreeNode; 
                TNW_node->X = current->X - res_list_5cm[depth+2]; 
                TNW_node->Y = current->Y + res_list_5cm[depth+2];
                TNW_node->Z = current->Z + res_list_5cm[depth+2];
                //TNW_node->level = depth+1; 
                TNW_node->Occ = 0;
                current->TNW = TNW_node;

                // if we've finished initializing leaf nodes, we can make a reference list to them. 
                // this way we don't have to traverse the tree when publishing. 
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

                if(x > current->X && y > current->Y && z < current->Z){
                    current = current->BNE; 
                    //std::cout<<"Going BNE "<<current->X<<","<<current->Y<<","<<current->Z<<std::endl;
                }else if(x > current->X && y < current->Y && z < current->Z){
                    current = current->BSE;
                    //std::cout<<"Going BSE "<<current->X<<","<<current->Y<<","<<current->Z<<std::endl; 
                }else if(x < current->X && y < current->Y && z < current->Z){
                    current = current->BSW;
                    //std::cout<<"Going BSW "<<current->X<<","<<current->Y<<","<<current->Z<<std::endl;  
                }else if(x < current->X && y > current->Y && z < current->Z){
                    current = current->BNW; 
                    //std::cout<<"Going BNW "<<current->X<<","<<current->Y<<","<<current->Z<<std::endl;
                }else if(x > current->X && y > current->Y && z > current->Z){
                    current = current->TNE; 
                    //std::cout<<"Going TNE "<<current->X<<","<<current->Y<<","<<current->Z<<std::endl;
                }else if(x > current->X && y < current->Y && z > current->Z){
                    current = current->TSE; 
                    //std::cout<<"Going TSE "<<current->X<<","<<current->Y<<","<<current->Z<<std::endl;
                }else if(x < current->X && y < current->Y && z > current->Z){
                    current = current->TSW; 
                    //std::cout<<"Going TSW "<<current->X<<","<<current->Y<<","<<current->Z<<std::endl;
                }else if(x < current->X && y > current->Y && z > current->Z){
                    current = current->TNW; 
                    //std::cout<<"Going TNW "<<current->X<<","<<current->Y<<","<<current->Z<<std::endl;
                }else{
                    // do nothing
                }

            }else{
                // the level has been initialized before 
                //search the current node
                //std::cout<<"this level has been initialized before"<<endl; 
                if(x > current->X && y > current->Y && z < current->Z){
                    current = current->BNE; 
                    //std::cout<<"Going BNE "<<current->X<<","<<current->Y<<","<<current->Z<<std::endl;
                }else if(x > current->X && y < current->Y && z < current->Z){
                    current = current->BSE; 
                    //std::cout<<"Going BSE "<<current->X<<","<<current->Y<<","<<current->Z<<std::endl;
                }else if(x < current->X && y < current->Y && z < current->Z){
                    current = current->BSW; 
                    //std::cout<<"Going BSW "<<current->X<<","<<current->Y<<","<<current->Z<<std::endl;
                }else if(x < current->X && y > current->Y && z < current->Z){
                    current = current->BNW; 
                    //std::cout<<"Going BNW "<<current->X<<","<<current->Y<<","<<current->Z<<std::endl;
                }else if(x > current->X && y > current->Y && z > current->Z){
                    current = current->TNE;
                    //std::cout<<"Going TNE "<<current->X<<","<<current->Y<<","<<current->Z<<std::endl; 
                }else if(x > current->X && y < current->Y && z > current->Z){
                    current = current->TSE; 
                    //std::cout<<"Going TSE "<<current->X<<","<<current->Y<<","<<current->Z<<std::endl; 
                }else if(x < current->X && y < current->Y && z > current->Z){
                    current = current->TSW; 
                    //std::cout<<"Going TSW "<<current->X<<","<<current->Y<<","<<current->Z<<std::endl; 
                }else if(x < current->X && y > current->Y && z > current->Z){
                    current = current->TNW; 
                    //std::cout<<"Going TNW "<<current->X<<","<<current->Y<<","<<current->Z<<std::endl; 
                }else{
                    // do nothing 
                }
            }
            depth+=1; 
        }
        // now we're at the depth 16. this is a leaf node 
        // update the current leaf node. 
        if(occupied && current->Occ == -1){         // to make sure we don't label them as unknown 
            current->Occ =1; 
        }else if(!occupied && current->Occ == 1){   // to make sure we don't label them as unknown
            current->Occ = -1; 
        }else if(occupied && current->Occ <16){
            current->Occ +=1; 
        }else if(!occupied && current->Occ>-16){
            current->Occ -=1; 
        }else{
            //do nothing 
        }
        //std::cout<<"done inserting"<<std::endl; 
    }
    

    public:
        Octree(OctreeNode *node){       // don't use this constructor 
            root = node; 
        }
        Octree(float leaf_resolution){  // in this case, leaf resolution is 1cm
            OctreeNode *root_node = new OctreeNode; 
            root_node->X = 0.00f; 
            root_node->Y = 0.00f;
            root_node->Z = 0.00f; 
            root_node->Occ = 0;     // unknown - so zero
            //root_node->level = 0; 
            
            root_node->TNW = NULL;
            root_node->TNE = NULL; 
            root_node->TSE = NULL;
            root_node->TSW = NULL;
            root_node->BNW = NULL;
            root_node->BNE = NULL; 
            root_node->BSE = NULL;
            root_node->BSW = NULL;
            
            root = root_node; 
        }

        ~Octree(){
            std::cout<<"desctructor called"<<endl; 
            destroy(root); 
            std::cout<<"destroyed"<<endl; 
        }

        OctreeNode *getRoot(){
            return root; 
        }

        OctreeNode &getRoot_reference(){
            return *root; // return root as a memory reference. 
        }   
        // make sure that you insert with correct precision/resolution 
        void insert(float x, float y, float z, bool status){
            insertNode(x, y, z, status); 
        }  
}; 



int main(){
    parent = new OctreeNode; 
    current = new OctreeNode; 

    Octree tree(0.01); 
    std::cout<<tree.getRoot()->X<<endl; 
    tree.insert(341.5, 254.5, 354.5, true);
    tree.insert(341.5, 254.5, 355.5, true); 
    tree.insert(340.5, 255.5, 355.5, true); 

    float x = -8.075f; 
    int count =1; 
    while(x<8.00f){
        tree.insert(x, x, x, true); 
        x+=0.05f; 
        count+=1; 
    }
    std::cout<<"inserted: "<<count<<" number of points"<<endl; 
    // while(!leaf_nodes.empty()){
    //         current = leaf_nodes.front(); 
    //         std::cout<<current->X<<"\t"<<current->Y<<"\t"<<current->Z<<"\t"<<unsigned(current->Occ)<<std::endl;   
    //         leaf_nodes.pop(); 
    // }	 
    return 0;
}