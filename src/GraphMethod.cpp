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
   
    Graph* MST = new ListGraph(0,graph->getSize());
    int mst_idx = 0;
    edge* sorted_edge = new edge[graph->getSize()*graph->getSize()];
    int edge_idx = 0;


    for(int i = 1;i<=graph->getSize();i++){
        map<int,int>* map_tmp = new map<int,int>;
        graph->getAdjacentEdges(i,map_tmp);

        for(auto iter = map_tmp->begin();iter != map_tmp->end();iter++){
            sorted_edge[edge_idx].from = i;
            sorted_edge[edge_idx].to = iter->first;
            sorted_edge[edge_idx].weight = iter->second;
            edge_idx++;
        }
        map_tmp->clear();
        delete map_tmp;
    }

    my_qsort(sorted_edge,0,edge_idx-1);

    int* parent = new int[edge_idx+1];

    for(int i = 0;i<=edge_idx;i++){
        parent[i] = -1;
    }

    int edgeCnt = edge_idx;
    edge_idx = 0;

    while(mst_idx < graph->getSize()-1 && edge_idx != edgeCnt){
        edge cur_edge = sorted_edge[edge_idx++];
        if(simpleFind(parent,cur_edge.from) != simpleFind(parent,cur_edge.to)){
            MST->insertEdge(cur_edge.from,cur_edge.to,cur_edge.weight);
            mst_idx++;
            simpleUnion(parent,simpleFind(parent,cur_edge.from),simpleFind(parent,cur_edge.to));
        }
    }

    if(mst_idx != graph->getSize()-1){
        return 0;
    }

//     ====== Kruskal =======
    // [1] 2(4)3(1)4(1)
    // [2] 1(4)
    // [3] 1(1)
    // [4] 1(1)5(6)
    // [5] 4(6)
    // cost: 12
    // =====================

    *fout<<"====== Kruskal =======\n";

    int cost = 0;

    for(int i = 1;i<=MST->getSize();i++){
        map<int,int>* tmp = new map<int,int>;
        MST->getAdjacentEdges(i,tmp);

        *fout<<'['<<i<<"] ";
        for(auto iter = tmp->begin();iter != tmp->end();iter++){
            *fout<<iter->first<<'('<<iter->second<<')';
            cost += iter->second;
        }
        *fout<<'\n';

        tmp->clear();
        delete tmp;
    }

    *fout<<"cost: "<<cost/2<<'\n';
    *fout<<"=====================\n\n";

    delete[] parent;
    delete[] sorted_edge;
    delete MST;

    return 1;

}

bool Dijkstra(Graph* graph, char option, int vertex,ofstream* fout)
{

    *fout<<"====== Dijkstra =======\n";

   map<int,int>* copiedGraph = new map<int,int>[graph->getSize()+1];
    if(option == 'N'){
        for(int i = 1;i<=graph->getSize();i++){
            graph->getAdjacentEdges(i,&copiedGraph[i]);
        }
        *fout<<"Undirected Graph Dijkstra result\n";
    }
    else if(option == 'Y'){
        for(int i = 1;i<=graph->getSize();i++){
            graph->getAdjacentEdgesDirect(i,&copiedGraph[i]);
        }
        *fout<<"Directed Graph Dijkstra result\n";
    }

    *fout<<"startvertex: "<<vertex<<'\n';

    Graph* MST = new ListGraph(0,graph->getSize());
    int mstCnt = 0;
    int* dist = new int[graph->getSize()+1];
    bool* visit = new bool[graph->getSize()+1];
    int* path = new int[graph->getSize()+1];

    const int INF = 2147483647;
    for(int i = 0;i<=graph->getSize();i++){
        dist[i] = INF;   // Set Maximum
        visit[i] = 0;
        path[i] = INF;
    }

    dist[vertex] = INF;
    visit[vertex] = 1;

    for(auto iter = copiedGraph[vertex].begin();iter!=copiedGraph[vertex].end();iter++){
        dist[iter->first] = iter->second;
        path[iter->first] = vertex;
    }

    while(mstCnt < graph->getSize()-1){
        int min = vertex;

        for(int i = 1;i<=graph->getSize();i++){
            if(dist[min] > dist[i] && !visit[i]){
                min = i;
            }
        } 

        visit[min] = true;

        for(auto iter = copiedGraph[min].begin();iter!=copiedGraph[min].end();iter++){
            if(dist[min]+iter->second < dist[iter->first] && !visit[iter->first]){
                dist[iter->first] = dist[min]+iter->second;
                path[iter->first] = min;
            }
        }
        mstCnt++;

    }

    /*
        [2] 1 -> 2 (4)
        [3] 1 -> 3 (1)
        [4] 1 -> 4 (1)
        [5] 1 -> 4 -> 5 (7)

    */

   for(int i = 1;i<=graph->getSize();i++){
    if(i == vertex) continue;

    if(path[i] == INF){
        *fout<<'['<<i<<"] x\n";
        continue;
    }

    int* tmp = new int[graph->getSize()];
    int tmpCnt = 1;
    int curNode = i;

    *fout<<'['<<i<<"] "<<vertex;

    tmp[0] = i;

    while(path[curNode] != vertex){
        tmp[tmpCnt++] = path[curNode];
        curNode = path[curNode];
    }

    while(tmpCnt > 0)
        *fout<<" -> "<<tmp[--tmpCnt];

    *fout<<'('<<dist[i]<<')'<<'\n';

    

   }

   *fout<<"=====================\n\n";








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

int simpleFind(int* parent,int i){
    int cur = i;
    while(parent[cur] >= 0){
        cur = parent[cur];
    }
    return cur;
}

void simpleUnion(int* parent,int i,int j){
    parent[i] = j;
}