#include <iostream>
#include <vector>
#include "GraphMethod.h"
#include <stack>
#include <queue>
#include <map>
#include <set>
#include <list>
#include <utility>

using namespace std;

bool BFS(Graph* graph, char option, int vertex, ofstream *fout)
{

    *fout<<"======== BFS ========\n";

    map<int,int>* copiedGraph = new map<int,int>[graph->getSize()+1];
    if(option == 'N'){
        for(int i = 1;i<=graph->getSize();i++){
            graph->getAdjacentEdges(i,&copiedGraph[i]);
        }
        *fout<<"Undirected Graph BFS result\n";
    }
    else if(option == 'Y'){
        for(int i = 1;i<=graph->getSize();i++){
            graph->getAdjacentEdgesDirect(i,&copiedGraph[i]);
        }
        *fout<<"Directed Graph BFS result\n";
    }

    *fout<<"startvertex: "<<vertex<<'\n';

    bool* visit = new bool[graph->getSize()+1];
    queue<int> bfsQueue;
    bool start = 0;

    for(int i = 0;i<=graph->getSize();i++){
        visit[i] = 0;
    }

    bfsQueue.push(vertex);
    visit[vertex] = 1;

    while(!bfsQueue.empty()){
        int frontVertex = bfsQueue.front();
        bfsQueue.pop();
        
        if(!start){
            *fout<<frontVertex;
            start = 1;
        }
        else
            *fout<<" -> "<<frontVertex;

        for(auto iter = copiedGraph[frontVertex].begin();iter != copiedGraph[frontVertex].end();iter++){
            if(!visit[iter->first]){
                bfsQueue.push(iter->first);
                visit[iter->first] = 1;
            }
        }
    }

    *fout<<"\n=====================\n\n";

    delete[] visit;

    for(int i = 0;i<=graph->getSize();i++){
        copiedGraph[i].clear();
    }

    delete[] copiedGraph;
}

bool DFS(Graph* graph, char option, int vertex, ofstream *fout)
{

    *fout<<"======== DFS ========\n";


   map<int,int>* copiedGraph = new map<int,int>[graph->getSize()+1];
    if(option == 'N'){
        for(int i = 1;i<=graph->getSize();i++){
            graph->getAdjacentEdges(i,&copiedGraph[i]);
        }
        *fout<<"Undirected Graph DFS result\n";
    }
    else if(option == 'Y'){
        for(int i = 1;i<=graph->getSize();i++){
            graph->getAdjacentEdgesDirect(i,&copiedGraph[i]);
        }
        *fout<<"Directed Graph DFS result\n";
    }

    *fout<<"startvertex: "<<vertex<<'\n';

    bool* visit = new bool[graph->getSize()+1];
    stack<int> dfsStack;
    bool start = 0;

    for(int i = 0;i<=graph->getSize();i++){
        visit[i] = 0;
    }

    dfsStack.push(vertex);
    visit[vertex] = 1;

    while(!dfsStack.empty()){
        int topVertex = dfsStack.top();
        dfsStack.pop();

        if(!start){
            *fout<<topVertex;
            start = 1;
        }
        else
            *fout<<" -> "<<topVertex;

        for(auto iter = copiedGraph[topVertex].rbegin();iter != copiedGraph[topVertex].rend();iter++){
            if(!visit[iter->first]){
                dfsStack.push(iter->first);
                visit[iter->first] = 1;
            }
        }
    }

    *fout<<"\n=====================\n\n";

    delete[] visit;

    for(int i = 0;i<=graph->getSize();i++){
        copiedGraph[i].clear();
    }

    delete[] copiedGraph;
 }

bool Kruskal(Graph* graph,ofstream* fout)
{
   
    edge* mst = new edge[graph->getSize()-1];
    int mst_idx = 0;
    edge* sorted_edge = new edge[graph->getSize()*graph->getSize()];
    int edge_idx = 0;


    for(int i = 1;i<=graph->getSize();i++){
        map<int,int>* map_tmp = new map<int,int>;
        graph->getAdjacentEdgesDirect(i,map_tmp);

        for(auto iter = map_tmp->begin();iter != map_tmp->end();iter++){
            sorted_edge[edge_idx].from = i;
            sorted_edge[edge_idx].to = iter->first;
            sorted_edge[edge_idx].weight = iter->second;
            edge_idx++;
        }
    }

    my_qsort(sorted_edge,0,edge_idx-1);

    set<int>* disjointSet = new set<int>[graph->getSize()];
    
    for(int i = 0;i<graph->getSize();i++){
        disjointSet[i].insert(i);
    }

    int edgeCnt = 0;

    while(mst_idx < graph->getSize()-1 && edgeCnt != edge_idx){
        edge cur_edge = sorted_edge[edgeCnt];
        edgeCnt++;
        
        // isCycle
        if(disjointSet[cur_edge.from].find(cur_edge.to) == disjointSet[cur_edge.from].end() && disjointSet[cur_edge.to].find(cur_edge.from) == disjointSet[cur_edge.to].end()){

        }
    }


    
}

bool Dijkstra(Graph* graph, char option, int vertex)
{
   
}

bool Bellmanford(Graph* graph, char option, int s_vertex, int e_vertex) 
{
   
}

bool FLOYD(Graph* graph, char option)
{
   
}

bool KWANGWOON(Graph* graph, int vertex) {

}

void my_qsort(edge* arr,int start,int end){
    if(start<end){
        if(end-start+1 <= 6){
            my_insertionSort(arr,start,end);
        }
        else{
            int mid = partition(arr,start,end);
            my_qsort(arr,start,mid-1);
            my_qsort(arr,mid+1,end);
        }
    }
}

int partition(edge* arr,int start,int end){
    int i = start+1;
    int j = end;
    int pivot = start;

    while(i<j){
        if(arr[i].weight > arr[pivot].weight && arr[j].weight < arr[pivot].weight){
            edge tmp = arr[i];
            arr[i] = arr[j];
            arr[j] = tmp;
            i++;
            j--;
        }
        else if(arr[i].weight > arr[pivot].weight){
            j--;
        }
        else if(arr[j].weight < arr[pivot].weight){
            i++;
        }
        else{
            i++;
            j--;
        }
    }

    while(1){
        edge tmp = arr[pivot];
        arr[pivot] = arr[pivot +1];
        arr[pivot+1] = tmp;
        pivot++;

        if(pivot == end || arr[pivot].weight < arr[pivot+1].weight){
            break;
        }
    }

    return pivot;
}

void my_insertionSort(edge* arr,int start,int end){
    
    for(int j = start + 1;j<=end;j++){
        for(int i = j - 1; i>=start;i--){
            if(arr[i+1].weight < arr[i].weight){
                edge tmp = arr[i+1];
                arr[i+1] = arr[i];
                arr[i] = tmp;
            }
            else break;
        }
    }
}