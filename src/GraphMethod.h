#ifndef _GRAPHMETHOD_H_
#define _GRAPHMETHOD_H_

#include "ListGraph.h"
#include "MatrixGraph.h"

bool BFS(Graph* graph, char option, int vertex, ofstream *fout);     
bool DFS(Graph* graph, char option,  int vertex , ofstream *fout);     
bool KWANGWOON(Graph* graph, int vertex,ofstream* fout);  
bool Kruskal(Graph* graph,ofstream* fout);
bool Dijkstra(Graph* graph, char option, int vertex,ofstream* fout);    //Dijkstra
bool Bellmanford(Graph* graph, char option, int s_vertex, int e_vertex, ofstream* fout); //Bellman - Ford
bool FLOYD(Graph* graph, char option, ofstream* fout);   //FLoyd

struct edge{
    int from;
    int to;
    int weight;
};

void my_qsort(edge* arr,int start,int end);
int partition(edge* arr,int start,int end);
void my_insertionSort(edge* arr,int start,int end);

int simpleFind(int* parent,int i);
void simpleUnion(int* parent,int i,int j);

int KWInit(vector<int>& tree,int start,int end,int idx);
void KWUpdate(vector<int>& tree,int start,int end,int what,int idx,int val);

#endif
