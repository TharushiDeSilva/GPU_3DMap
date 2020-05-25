#include <iostream> 
#include <bits/stdc++.h> 
#include <string>

using namespace std; 
struct BSTNode{
    int Key; 
    BSTNode *Left; 
    BSTNode *Right; 
};

class BST{

    BSTNode *root; //the root node
    stack<BSTNode*> st;   

    BSTNode *makeEmptyNode(BSTNode *node){
        if(node == NULL){
            return NULL; 
            {
                makeEmptyNode(node->Left); 
                makeEmptyNode(node->Right); 
                delete node; 
            }
            return NULL; 
        }
    }

    void insert_node(int key){
        BSTNode *node = new BSTNode; 
        node->Key = key; 
        node->Left = node->Right = NULL; 
        if(root == NULL){
            root = node; 
            return; 
        }
        BSTNode *current = root; 
        BSTNode *parent = root;  
         
        while(current != NULL){
            if(current->Key > key){
                parent = current; 
                current = current->Left; 
            }else if(current->Key < key){
                parent = current; 
                current = current->Right; 
            }else{
                return; 
            }
        }
        if(parent->Key > key){
            parent->Left = node; 
        }else{
            parent->Right = node; 
        } 
         
    }


    void insert_2(int key){
        
    }


    BSTNode *findMinNode(BSTNode *node){
        if(node == NULL){
            return NULL; 
        }else if(node->Left == NULL){
            return node; 
        }else{
            return findMinNode(node->Left); 
        }
        
    }
    
    BSTNode *findMaxNode(BSTNode *node){
        if(node == NULL){
            return NULL; 
        }else if(node->Right == NULL){
            return node; 
        }else{
            return findMaxNode(node->Right); 
        }
    }

    void inOrderTraverse(BSTNode *node){
        if(node == NULL){
            return; 
        }
        inOrderTraverse(node->Left); 
        cout<<node->Key<<"  "; 
        inOrderTraverse(node->Right); 
    }
    
    void postOrderDelete(){
        if(root == NULL){
            return; 
        }
        stack<BSTNode*> node_stack; 
        stack<char> id_stack; 
        BSTNode *current = root; 
        
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
                        current->Left = NULL; 
                        id_stack.pop(); 
                    }else{
                        current->Right = NULL; 
                        id_stack.pop(); 
                    }
                }else{
                    // if we've come to the root
                    //std::cout<<"deleting: "<<root->Key<<endl; 
                    root = NULL;
                     

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

    BSTNode *searchNode(BSTNode *node, int key){
        if(node==NULL){
            return NULL; 
        }else if(node->Key = key){
            return node; 
        }else if(node->Key > key){
            return searchNode(node->Left, key); 
        }else{
            return searchNode(node->Right, key); 
        }
    }


    public:
        BST() {
            root = NULL; 
        }
        ~BST(){
            if(root != NULL){
                postOrderDelete(); 
            } 
        }
        void insert(int key){
            //root = insertNode(key, root); 
            insert_node(key); 
        }
        void display(){
            inOrderTraverse(root); 
            std::cout<<std::endl; 
        }

        void searchNode(int key){
            root = searchNode(root, key); 
        }

        void delete_all(){
            postOrderDelete(); 
        }

        BSTNode *begin(){
            return findMinNode(root); 
        }
        BSTNode *end(){
            return findMaxNode(root); 
        }
        BSTNode *getRoot(){
            return root; 
        }
        
    
};

class BSTIterator{

    private:
        stack<BSTNode*> node_stack; 
        stack<char> flags; 

    public:
        BSTIterator(BST tree){     // the constructor
            if(tree.getRoot() == NULL){
                return; 
            }
            BSTNode *current = new BSTNode; 
            current = tree.getRoot(); 
            node_stack.push(current); 

            while(current->Left != NULL){
                //cout<<currrent->Key<<"\t"; 
                current = current->Left; 
                node_stack.push(current); 
                flags.push('l'); 
            }
        }
    
        // at the end of construction, we are at the begining of the tree.
        BSTNode *getCurrent_node(){
            return node_stack.top();  
        }

        void next_node(){
            if(node_stack.empty()){
                return; 
            }
            BSTNode *current = node_stack.top(); 
                   
            if(current->Left == NULL && current->Right == NULL){
                std::cout<<"node: "<<current->Key<<endl; 
                node_stack.pop(); 
            }
        }
};


int main(){

while(true){
    BST tree; 
    tree.insert(200); 
    for(int i= 220; i<=400; i++){   
        tree.insert(i);
        tree.insert(-i);     
    }
    tree.insert(210); 
    for(int i= 430; i<=550; i++){       
        tree.insert(i);       
    }
    tree.insert(410); 
    cout<<"tree complete"<<endl; 
    tree.display(); 

    // BSTIterator itr(tree); 
    // std::cout<<"\n\n"<<endl; 
    // for(int i=0; i<30; i++){
    //     itr.next_node();
    // }
     
    
}

return 0;
}