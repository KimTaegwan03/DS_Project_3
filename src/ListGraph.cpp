#include "ListGraph.h"
#include <iostream>
#include <utility>

using namespace std;

ListGraph::ListGraph(bool type, int size) : Graph(type, size)
{
	m_List = new map<int,int>[size+1];    // key = dest, value = weight
	kw_graph = new vector<int>[size+1];

}

ListGraph::~ListGraph()	
{
	for(int i = 0;i<=m_Size;i++){
		m_List[i].clear();
		kw_graph[i].clear();
	}

	delete[] m_List;
	delete[] kw_graph;
	
}

void ListGraph::getAdjacentEdges(int vertex, map<int, int>* m)	 //Definition of getAdjacentEdges(No Direction == Undirected)
{
	// Insert edge from vertex
	for(auto iter = m_List[vertex].begin(); iter != m_List[vertex].end() ; iter++){
		m->insert({iter->first,iter->second});
	}

	// Insert edge to the vertex
	for(int i = 1;i<=m_Size;i++){
			if(m_List[i].find(vertex) != m_List[i].end()){
				m->insert({i,m_List[i].find(vertex)->second});
			}
		}
}

void ListGraph::getAdjacentEdgesDirect(int vertex, map<int, int>* m)	//Definition of getAdjacentEdges(Directed graph)
{	
	// Insert edge from the vertex
	for(auto iter = m_List[vertex].begin(); iter != m_List[vertex].end() ; iter++){
		m->insert({iter->first,iter->second});
	}
}

void ListGraph::insertEdge(int from, int to, int weight) //Definition of insertEdge
{
	if(from <= m_Size){
		m_List[from].insert({to,weight});
		kw_graph[from].push_back(to);
		kw_graph[to].push_back(from);
		sort(kw_graph[from].begin(),kw_graph[from].end());
		sort(kw_graph[to].begin(),kw_graph[to].end());
	}
}

bool ListGraph::printGraph(ofstream *fout)	//Definition of print Graph
{
	*fout<<"======== PRINT========\n";

	for(int i = 1;i<=m_Size;i++){
		*fout<<'['<<i<<']';
		for(auto iter = m_List[i].begin();iter!=m_List[i].end();iter++){
			*fout<<"-> ("<<iter->first<<','<<iter->second<<") ";
		}
		*fout<<'\n';
	}
	
	*fout<<"=====================\n\n";
}

vector<int> ListGraph::getKWgraph(int vertex){
	return kw_graph[vertex];
}