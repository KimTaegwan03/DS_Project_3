#ifndef _LIST_H_
#define _LIST_H_

#include "Graph.h"

class ListGraph : public Graph{	
private:
	map < int, int >* m_List;		//map index = from vertex, map key = to vertex, map value = edge weight
	vector<int>* kw_graph;
	
public:	
	ListGraph(bool type, int size);	// 0 = List, 1 = Matrix
	~ListGraph();
		
	void getAdjacentEdges(int vertex, map<int, int>* m);	
	void getAdjacentEdgesDirect(int vertex, map<int, int>* m);
	void insertEdge(int from, int to, int weight);	
	bool printGraph(ofstream *fout);

};

#endif