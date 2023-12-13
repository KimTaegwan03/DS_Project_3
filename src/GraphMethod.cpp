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

bool BFS(Graph* graph, char option, int vertex, ofstream *fout) // Function of BFS
{

    *fout<<"======== BFS ========\n";

    map<int,int>* copiedGraph = new map<int,int>[graph->getSize()+1];  

    if(option == 'N'){                                      // Undirected BFS
        for(int i = 1;i<=graph->getSize();i++){
            graph->getAdjacentEdges(i,&copiedGraph[i]); 
        }
        *fout<<"Undirected Graph BFS result\n";
    }
    else if(option == 'Y'){                                 // Directed BFS
        for(int i = 1;i<=graph->getSize();i++){
            graph->getAdjacentEdgesDirect(i,&copiedGraph[i]);
        }
        *fout<<"Directed Graph BFS result\n";
    }

    *fout<<"startvertex: "<<vertex<<'\n';

    bool* visit = new bool[graph->getSize()+1];     // Check visited
    queue<int> bfsQueue;
    bool start = 0;

    for(int i = 0;i<=graph->getSize();i++){
        visit[i] = 0;
    }

    bfsQueue.push(vertex);      // Push start vertex to queue
    visit[vertex] = 1;

    while(!bfsQueue.empty()){
        int frontVertex = bfsQueue.front();
        bfsQueue.pop();
        
        // Print format
        if(!start){
            *fout<<frontVertex;
            start = 1;
        }
        else
            *fout<<" -> "<<frontVertex;


        // Push connected vertex, which is not visited
        for(auto iter = copiedGraph[frontVertex].begin();iter != copiedGraph[frontVertex].end();iter++){
            if(!visit[iter->first]){
                bfsQueue.push(iter->first);
                visit[iter->first] = 1;
            }
        }
    }

    *fout<<"\n=====================\n\n";

    // Memory free part
    delete[] visit;
    for(int i = 0;i<=graph->getSize();i++){
        copiedGraph[i].clear();
    }
    delete[] copiedGraph;

    return 1;
}

bool DFS(Graph* graph, char option, int vertex, ofstream *fout)     // Function of DFS
{  
    *fout<<"======== DFS ========\n";

   map<int,int>* copiedGraph = new map<int,int>[graph->getSize()+1];

    if(option == 'N'){                                         // Undirected DFS
        for(int i = 1;i<=graph->getSize();i++){
            graph->getAdjacentEdges(i,&copiedGraph[i]);
        }
        *fout<<"Undirected Graph DFS result\n";
    }
    else if(option == 'Y'){                                     // Directed DFS
        for(int i = 1;i<=graph->getSize();i++){
            graph->getAdjacentEdgesDirect(i,&copiedGraph[i]);
        }
        *fout<<"Directed Graph DFS result\n";
    }

    *fout<<"startvertex: "<<vertex<<'\n';

    bool* visit = new bool[graph->getSize()+1];     // Check visited array
    stack<int> dfsStack;
    bool start = 0;

    for(int i = 0;i<=graph->getSize();i++){
        visit[i] = 0;
    }

    dfsStack.push(vertex);      // Push start vertex to stack

    while(!dfsStack.empty()){
        int topVertex = dfsStack.top();
        dfsStack.pop();

        if(!visit[topVertex]){
            visit[topVertex] = 1;

            // Print format
            if(!start){
                *fout<<topVertex;
                start = 1;
            }
            else
                *fout<<" -> "<<topVertex;

            // Push connected vertex, which is not visited
            for(auto iter = copiedGraph[topVertex].rbegin();iter != copiedGraph[topVertex].rend();iter++){
                if(!visit[iter->first]){
                    dfsStack.push(iter->first);
                }
            }
        }
    }

    *fout<<"\n=====================\n\n";

    // Memory free part
    delete[] visit;
    for(int i = 0;i<=graph->getSize();i++){
        copiedGraph[i].clear();
    }
    delete[] copiedGraph;

    return 1;
 }

bool Kruskal(Graph* graph,ofstream* fout)   // Function of Kruskal
{
    Graph* MST = new ListGraph(0,graph->getSize());     // New graph for Minimun Spanning Tree
    int mst_idx = 0;

    edge* sorted_edge = new edge[graph->getSize()*graph->getSize()];    // array of edge, the structure of from, to vertex and weight. 
    int edge_idx = 0;

    // Initialize array of edges
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

    // Sorting array
    my_qsort(sorted_edge,0,edge_idx-1);

    // Disjoint set
    int* parent = new int[edge_idx+1];

    for(int i = 0;i<=edge_idx;i++){
        parent[i] = -1;
    }

    int edgeCnt = edge_idx;
    edge_idx = 0;

    // Check the shortest edge. Add the edge, which doesn't make cycle.
    while(mst_idx < graph->getSize()-1 && edge_idx != edgeCnt){
        edge cur_edge = sorted_edge[edge_idx++];
        if(simpleFind(parent,cur_edge.from) != simpleFind(parent,cur_edge.to)){
            MST->insertEdge(cur_edge.from,cur_edge.to,cur_edge.weight);
            mst_idx++;
            simpleUnion(parent,simpleFind(parent,cur_edge.from),simpleFind(parent,cur_edge.to));
        }
    }

    if(mst_idx != graph->getSize()-1){  // There is no spanning tree
        return false;
    }

    // Print part
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

    // Memory free part
    delete[] parent;
    delete[] sorted_edge;
    delete MST;

    return true;

}

bool Dijkstra(Graph* graph, char option, int vertex,ofstream* fout)     // Function of Dijkstra
{
    *fout<<"====== Dijkstra =======\n";

   map<int,int>* copiedGraph = new map<int,int>[graph->getSize()+1];

    if(option == 'N'){                                          // Undirected Dijkstra
        for(int i = 1;i<=graph->getSize();i++){
            graph->getAdjacentEdges(i,&copiedGraph[i]);
        }
        *fout<<"Undirected Graph Dijkstra result\n";
    }
    else if(option == 'Y'){                                     // Directed Dijkstra
        for(int i = 1;i<=graph->getSize();i++){
            graph->getAdjacentEdgesDirect(i,&copiedGraph[i]);
        }
        *fout<<"Directed Graph Dijkstra result\n";
    }
    *fout<<"startvertex: "<<vertex<<'\n';

    int mstCnt = 0;

    // Arrays for Dijkstra
    int* dist = new int[graph->getSize()+1];        // Distance from start vertex to index vertex
    bool* visit = new bool[graph->getSize()+1];     // Check visited vertex
    int* path = new int[graph->getSize()+1];        // Check where it's coming from

    const int INF = 1000000000;

    // Initialize
    for(int i = 0;i<=graph->getSize();i++){
        dist[i] = INF;
        visit[i] = 0;
        path[i] = INF;
    }

    // Check start vertex
    dist[vertex] = INF;
    visit[vertex] = 1;

    // Set distance near start vertex
    for(auto iter = copiedGraph[vertex].begin();iter!=copiedGraph[vertex].end();iter++){
        dist[iter->first] = iter->second;
        path[iter->first] = vertex;
    }

    while(mstCnt < graph->getSize()-1){
        int min = vertex;

        // Find minimum distance
        for(int i = 1;i<=graph->getSize();i++){
            if(dist[min] > dist[i] && !visit[i]){
                min = i;
            }
        }

        visit[min] = true;

        // Update distance from the min vertex
        for(auto iter = copiedGraph[min].begin();iter!=copiedGraph[min].end();iter++){
            if(dist[min]+iter->second < dist[iter->first] && !visit[iter->first]){
                dist[iter->first] = dist[min]+iter->second;
                path[iter->first] = min;
            }
        }
        mstCnt++;

    }

    // Print part
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

        delete[] tmp;
    }

    *fout<<"=====================\n\n";

    // Memory free part
    delete[] path;
    delete[] visit;
    delete[] dist;
    for(int i = 0;i<graph->getSize()+1;i++){
        copiedGraph[i].clear();
    }
    delete[] copiedGraph;

    return 1;
}

bool Bellmanford(Graph* graph, char option, int s_vertex, int e_vertex, ofstream* fout)     // Function of Bellman-Ford
{
    const int INF = 1000000000;

   map<int,int>* copiedGraph = new map<int,int>[graph->getSize()+1];

    if(option == 'N'){                                         // Undirected Bellman-Ford
        for(int i = 1;i<=graph->getSize();i++){
            graph->getAdjacentEdges(i,&copiedGraph[i]);
        }
        
    }
    else if(option == 'Y'){                                     // Directed Bellman-Ford
        for(int i = 1;i<=graph->getSize();i++){
            graph->getAdjacentEdgesDirect(i,&copiedGraph[i]);
        }
        
    }
    else return 0;

    // Arrays for Bellman-Ford
    int* dist = new int[graph->getSize()+1];
    int* path = new int[graph->getSize()+1];

    // Initialize
    for(int i= 0;i<=graph->getSize();i++){
        dist[i] = INF;
        path[i] = s_vertex;
    }
    for(auto iter = copiedGraph[s_vertex].begin();iter!=copiedGraph[s_vertex].end();iter++){
        dist[iter->first] = iter->second;
    }

    // Bellman-Ford Algorithm
    for(int k = 2; k<=graph->getSize()-1;k++){      // Number of edges
        for(int v = 1;v<=graph->getSize();v++){     // destination vertex
            if(v==s_vertex) continue;
            if(dist[v] != INF){
                for(auto iter = copiedGraph[v].begin();iter!=copiedGraph[v].end();iter++){

                    // dist[w] = min(dist[w],dist[v]+length[v][w])
                    if(dist[iter->first] > dist[v] + iter->second){
                        dist[iter->first] = dist[v] + iter->second;
                        path[iter->first] = v;
                    }
                }
            }
        }
    }

    // Check negative cycle
    for(int v = 1;v<=graph->getSize();v++){
        if(v==s_vertex) continue;
        if(dist[v] != INF){
            for(auto iter = copiedGraph[v].begin();iter!=copiedGraph[v].end();iter++){
                if(dist[iter->first] > dist[v] + iter->second){
                    return 0;
                }
            }
        }
    }

    // Print part
    int* tmp = new int[graph->getSize()];
    int idxCnt = 0;
    int curIdx = e_vertex;

    while(path[curIdx] != s_vertex){
        tmp[idxCnt++] = path[curIdx];
        curIdx = path[curIdx];
    }

    *fout<<"====== Bellman-Ford =======\n";

    if(option == 'N')
        *fout<<"Undirected Graph Bellman-Ford result\n";
    else
        *fout<<"Directed Graph Bellman-Ford result\n";


    if(dist[e_vertex] == INF){
        *fout<<"x\n";
    }
    else{
        *fout<<s_vertex;

        for(int i = idxCnt - 1;i>=0;i--){
            *fout<<" -> "<<tmp[i];
        }

        *fout<<" -> "<<e_vertex;

        *fout<<"\ncost: "<<dist[e_vertex]<<'\n';
    }
    *fout<<"=====================\n\n";


    // Memory free part
    for(int i = 0;i<=graph->getType();i++){
        copiedGraph[i].clear();
    }
    delete[] copiedGraph;
    delete[] dist;
    delete[] path;
    delete[] tmp;

    return 1;
}

bool FLOYD(Graph* graph, char option, ofstream* fout)       // Function of FLOYD
{
   const int INF = 1000000000;

   map<int,int>* copiedGraph = new map<int,int>[graph->getSize()+1];

    if(option == 'N'){                                  // Undirected FLOYD
        for(int i = 1;i<=graph->getSize();i++){
            graph->getAdjacentEdges(i,&copiedGraph[i]);
        }
        
    }
    else if(option == 'Y'){                             // Directed FLOYD
        for(int i = 1;i<=graph->getSize();i++){
            graph->getAdjacentEdgesDirect(i,&copiedGraph[i]);
        }
        
    }
    else return 0;

    // 2D array for FLOYD
    int**dist = new int*[graph->getSize()+1];
    dist[0] = new int[(graph->getSize()+1)*(graph->getSize()+1)];

    for(int i = 1;i<=graph->getSize();i++){
        dist[i] = &dist[0][(graph->getSize()+1)*i];

        // Initialize
        for(int j = 1;j<=graph->getSize();j++){
            dist[i][j] = INF;
            if(i==j) dist[i][j] = 0;
        }
    }
    for(int i = 1;i<=graph->getSize();i++){
        for(auto iter = copiedGraph[i].begin();iter!= copiedGraph[i].end();iter++){
            dist[i][iter->first] = iter->second;
        }
    }

    // FLOYD Algorithm
    for(int k = 1;k<=graph->getSize();k++){
        for(int i = 1;i<=graph->getSize();i++){
            for(int j = 1;j<=graph->getSize();j++){
                dist[i][j] = min(dist[i][j],dist[i][k] + dist[k][j]);
            }
        }
    }

    // Check negative cycle
    for(int k = 1;k<=graph->getSize();k++){
        for(int i = 1;i<=graph->getSize();i++){
            for(int j = 1;j<=graph->getSize();j++){
                if(dist[i][j] > dist[i][k] + dist[k][j])
                    return 0;
            }
        }
    }

    // Print part
    *fout<<"======== FLOYD ========\n";

    if(option == 'Y')
        *fout<<"Directed Graph FLOYD result\n\t";
    else
        *fout<<"Undirected Graph FLOYD result\n\t";

    for(int i = 1;i<=graph->getSize();i++){
        *fout<<'['<<i<<"]\t";
    }

    *fout<<'\n';

    for(int i = 1;i<=graph->getSize();i++){
        *fout<<'['<<i<<"]\t";
        for(int j = 1;j<=graph->getSize();j++){
            if(dist[i][j] == INF) *fout<<"x\t";
            else
                *fout<<dist[i][j]<<'\t';
        }
        *fout<<'\n';
    }

    *fout<<"==========================\n\n";

    // Memory free part
    for(int i = 0;i<=graph->getType();i++){
        copiedGraph[i].clear();
    }
    delete[] copiedGraph;
    delete[] dist[0];
    delete[] dist;

    return 1;

}

bool KWANGWOON(Graph* graph, int vertex, ofstream* fout)
{   
    // Segment Tree
    vector<int>* seg = new vector<int>[graph->getSize()+1];
    int* visit = new int[graph->getSize()+1];
    
    // Initialize of segment tree
    for(int i = 1;i<=graph->getSize();i++){
        seg[i].resize(((ListGraph*)graph)->getKWgraph(i).size()*4);
        visit[i] = 0;
        KWInit(seg[i],0,((ListGraph*)graph)->getKWgraph(i).size()-1,1);
    }

    int curVertex = 1;
    visit[1] = 1;

    // Print format
    *fout<<"======== KWANGWOON========\n";
    *fout<<"startvertex: 1\n1";

    // KwangWoon algorithm
    for(int i = 1;i<=graph->getSize();i++){
        if(i!=1)
            *fout<<" -> "<<curVertex;

        // Update segment tree of vertex, which is near vertex of current vertex
        for(int j = 0;j<((ListGraph*)graph)->getKWgraph(curVertex).size();j++){
            int cnt = 0;
            int curNode = ((ListGraph*)graph)->getKWgraph(curVertex)[j];
            if(!visit[curNode]){
                for(int k = 0;((ListGraph*)graph)->getKWgraph(curNode).size();k++){
                    if(curVertex == ((ListGraph*)graph)->getKWgraph(curNode)[k]) break;
                    cnt++;
                }
                KWUpdate(seg[curNode],0,((ListGraph*)graph)->getKWgraph(curNode).size()-1,cnt,1,-1);
            }
        }

        // Even
        if(seg[curVertex][1]%2 == 0){
            for(int j = 0;j<((ListGraph*)graph)->getKWgraph(curVertex).size();j++){
                if(!visit[((ListGraph*)graph)->getKWgraph(curVertex)[j]]){
                    curVertex = ((ListGraph*)graph)->getKWgraph(curVertex)[j];
                    visit[curVertex] = 1;
                    break;
                }
            }
        }
        // Odd
        else{
            for(int j = ((ListGraph*)graph)->getKWgraph(curVertex).size()-1;j>=0;j--){
                if(!visit[((ListGraph*)graph)->getKWgraph(curVertex)[j]]){
                    curVertex = ((ListGraph*)graph)->getKWgraph(curVertex)[j];
                    visit[curVertex] = 1;
                    break;
                }
            }
        }
    }
    *fout<<"\n============================\n\n";


    // Memory free part
    for(int i = 0;i<=graph->getSize();i++){
        seg[i].clear();
    }
    delete[] seg;
    delete[] visit;

    return 1;
}

void my_qsort(edge* arr,int start,int end)      // My quicksort function
{
    if(start<end){
        if(end-start+1 <= 6){                   // size of array less than 6, insertion sort
            my_insertionSort(arr,start,end);
        }
        else{                                      // else quick sort
            int mid = partition(arr,start,end);
            my_qsort(arr,start,mid-1);
            my_qsort(arr,mid+1,end);
        }
    }
}

int partition(edge* arr,int start,int end){     // On left side of pivot, less than pivot,
    int i = start;                              // On right side of pivot, larger than pivot
    int j = end;
    edge pivot = arr[start];

    while(i<j){
        while(pivot.weight < arr[j].weight) j--;
        while(i < j && pivot.weight >= arr[i].weight) i++;

        edge tmp = arr[i];
        arr[i] = arr[j];
        arr[j] = tmp;
        
    }

    arr[start] = arr[i];
    arr[i] = pivot;

    return i;
}

void my_insertionSort(edge* arr,int start,int end){     // My insertion sort
    int i,j;
    for(j = start + 1;j<=end;j++){
        edge key = arr[j];
        for(i = j - 1; i>=start && key.weight < arr[i].weight;i--){
            arr[i+1] = arr[i];
        }

        arr[i+1] = key;
    }
}

int simpleFind(int* parent,int i){      // Find root of disjoint set
    int cur = i;
    while(parent[cur] >= 0){
        cur = parent[cur];
    }
    return cur;
}

void simpleUnion(int* parent,int i,int j){  // Union disjoint sets
    parent[i] = j;
}

int KWInit(vector<int>& tree,int start,int end,int idx){    // Initialize of segment tree for Kwangwoon algorithm
    if(start == end) return tree[idx] = 1;

    int mid = (start+end) / 2;

    return tree[idx] = KWInit(tree,start,mid,2*idx) + KWInit(tree,mid+1,end,2*idx+1);
}

void KWUpdate(vector<int>& tree,int start,int end,int what,int idx,int val){    // Update segment tree
    if(what < start || what > end) return;

    tree[idx] += val;

    if(start != end){
        int mid = (start+end)/2;
        KWUpdate(tree,start,mid,what,2*idx,val);
        KWUpdate(tree,mid+1,end,what,2*idx+1,val);
    }
}