//#include<iostream>
//#include<vector>
#include<bits/stdc++.h>
using namespace std;

typedef pair<int,int> Pair;
typedef tuple<double,int,int> tup;
#define  Inf numeric_limits<double>::infinity();

typedef struct node{
    double f = Inf;
    double g = Inf;
    double h = Inf;
    Pair parent = make_pair(-1,-1);
    bool visit = false;
    node(){};
}Node;

void aStarSearch(vector<vector<int>> grid,Pair src,Pair goal);

void path_g(vector<vector<int>>& grid,Pair& goal,vector<vector<Node>>& visited){
    int r = goal.first;
    int c = goal.second;
    while(r != -1){

        Node temp_node = visited[r][c];
        grid[r][c] = 8;
        r = temp_node.parent.first;
        c = temp_node.parent.second;
    }
    for(auto i : grid){
        for(auto j :i){
            cout<<j<<", ";
        }
        cout<<endl;
    }
}
double  hrs(int r_new,int c_new,int goal_r,int goal_c){
    return sqrt(pow((r_new-goal_r),2)+pow((c_new-goal_c),2));
}

void aStarSearch(vector<vector<int>> grid,Pair src,Pair goal){
    int rows = grid.size();
    int cols = grid[0].size();
    vector<vector<Node>> visited(rows,vector<Node>(cols));
    visited[src.first][src.second].f = 0;
    visited[src.first][src.second].g = 0; 
    visited[src.first][src.second].h = 0;  
    vector<Node> path;
    int r_src = src.first;
    int c_src = src.second;
    priority_queue<tup,vector<tup>,greater<tup>> pq;
    pq.push(make_tuple(0.0,r_src,c_src));
    while(!pq.empty()){
        tup temp = pq.top();
        double cost = get<0>(temp);
        int r = get<1>(temp);
        int c = get<2>(temp);
        pq.pop();

        if(visited[r][c].visit)
            continue;
        visited[r][c].visit = true;
        
        if(r == goal.first && c ==goal.second){
            cout<<"path"<<endl;
            path_g(grid,goal,visited);
            return;
        }
        
        vector<int> d = {1,0,-1,1,1,-1,-1,0,1};
        for(int i =0;i<d.size()-1;i++){
            int r_new = r+d[i];
            int c_new = c+d[i+1];
            if(r_new <0 || r_new >=rows || c_new<0|| c_new>=cols )
                continue;
            Node new_node = visited[r_new][c_new];
            if(grid[r_new][c_new] && !new_node.visit 
            && new_node.f> cost + 1+ hrs(r_new,c_new,goal.first,goal.second)){
                new_node.g =  cost+1;
                new_node.h =  hrs(r_new,c_new,goal.first,goal.second);
                new_node.f = new_node.g+new_node.h;
                new_node.parent = make_pair(r,c);  

                pq.push(make_tuple(new_node.f,r_new,c_new));
                visited[r_new][c_new] = new_node;
                //cout<<r_new<<" "<<c_new<<endl;
            }

        }
    }
    cout<<"exit";
}
int main() 
{ 
    /* Description of the Grid- 
     1--> The cell is not blocked 
     0--> The cell is blocked    */
    vector<vector<int>> grid = 
    { 
        { 1, 0, 1, 1, 1, 1, 0, 1, 1, 1 }, 
        { 1, 1, 1, 0, 1, 1, 1, 0, 1, 1 }, 
        { 1, 1, 1, 0, 1, 1, 0, 1, 0, 1 }, 
        { 0, 0, 1, 0, 1, 0, 0, 0, 0, 1 }, 
        { 1, 1, 1, 0, 1, 1, 1, 0, 1, 0 }, 
        { 1, 0, 1, 1, 1, 1, 0, 1, 0, 0 }, 
        { 1, 0, 0, 0, 0, 1, 0, 0, 0, 1 }, 
        { 1, 0, 1, 1, 1, 1, 0, 1, 1, 1 }, 
        { 1, 1, 1, 0, 0, 0, 1, 0, 0, 1 } 
    }; 
  
    // Source is the left-most bottom-most corner 
    Pair src = make_pair(8, 8); 
  
    // Destination is the left-most top-most corner 
    Pair dest = make_pair(0, 0); 
  
    aStarSearch(grid, src, dest); 
  
    return(0); 
}