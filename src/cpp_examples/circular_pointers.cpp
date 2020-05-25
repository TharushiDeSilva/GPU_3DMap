#include <iostream> 
#include <bits/stdc++.h> 

using namespace std; 

struct BSTNode{
    int Key; 
    BSTNode *Left; 
    BSTNode *Right; 
}; 

BSTNode *parent, *current; // for traversing, and inserting 

class BST{
    private: 
        BSTNode *root;

        void postOrderDelete(){
            
            if(root == NULL){
                return; 
            }
            stack<BSTNode*> node_stack; 
            stack<char> id_stack; 
            
            current = root; 
            node_stack.push(current); 

            while(!node_stack.empty()){
                if(current->Left ==NULL && current->Right == NULL){
                    // no children 
                    node_stack.pop(); 
                    //std::cout<<"deleting: "<<current->Key<<endl; 
                    if(!node_stack.empty()){
                        // not at root
                        current = node_stack.top(); 
                        if(id_stack.top() == 'l'){
                            //current->Left = NULL; 
                            delete(current->Left); 
                            id_stack.pop(); 
                        }else{
                            //current->Right = NULL; 
                            delete(current->Right); 
                            id_stack.pop(); 
                        }
                    }else{
                        // if we've come to the root
                        //std::cout<<"deleting: "<<root->Key<<endl; 
                        //delete root; 
                        

                    }
                }else if(current->Left != NULL){
                    current = current->Left; 
                    node_stack.push(current); 
                    id_stack.push('l'); 
                }else if(current->Right != NULL){
                    current = current->Right; 
                    node_stack.push(current); 
                    id_stack.push('r'); 
                }else{}
            }
        }

        void destroy(BSTNode *root){
            if(root == NULL){
                return;
            }else{
                destroy(root->Left);
                destroy(root->Right);
                delete root;
            }
        }   


        void insert_node(int x){
            BSTNode *node = new BSTNode; 
            node->Key = x; 
            node->Left = node->Right = NULL; 
            if(root==NULL){
                root = node; 
                return; 
            }
            current = root;
            parent = root; 
            while(current != NULL){
                if(current->Key == x){
                    return; 
                }else if(current->Key > x){
                    parent = current; 
                    current = current->Left; 
                }else{
                    parent = current; 
                    current = current->Right; 
                }
            }
            // finish loop after current is NULL; 
            if(parent->Key > x){
                parent->Left = node; 
            }else if(parent->Key <x){
                parent->Right = node; 
            }else{
                // no such case. terminated equal case from above. 
            }
        }

    public: 
        BST(BSTNode *node){
            root = node; 
        }

        ~BST(){
            // desctructor
            //destroy(root); 
            postOrderDelete(); 
            cout<<"destructed"<<endl; 
        }

        BSTNode *getRoot(){
            return root; 
        }

        void insert(int val){
            insert_node(val); 
        }

}; 
 
int main(){
    // create the root 
    BSTNode *root = new BSTNode; 
    BSTNode *newnode = new BSTNode; 
    parent = new BSTNode; 
    current = new BSTNode; 

    root->Key = 0; 
    root->Left = root->Right = NULL; 
    //cout<<"root's location: "<<root<<"\n\n"<<endl; 
    
    while(true){
    
        BST tree(root);        
        
        for(int i=1; i<10000; i++){ 
            tree.insert(i); 
        }
    }
}

