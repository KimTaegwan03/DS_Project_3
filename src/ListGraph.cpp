#include "ListGraph.h"
#include <iostream>
#include <utility>

using namespace std;

ListGraph::ListGraph(bool type, int size) : Graph(type, size)
{
	m_List = new map<int,int>[size+1];    // key = dest, value = weight
}

ListGraph::~ListGraph()	
{
	
}

void ListGraph::getAdjacentEdges(int vertex, map<int, int>* m)	 //Definition of getAdjacentEdges(No Direction == Undirected)
{
	for(auto iter = m_List[vertex].begin(); iter != m_List[vertex].end() ; iter++){
		int fst = iter->first;
		int snd = iter->second;
		
		m->insert({iter->first,iter->second});

		
	}
	for(int i = 1;i<=m_Size;i++){
			if(m_List[i].find(vertex) != m_List[i].end()){
				m->insert({i,m_List[i].find(vertex)->second});
			}
		}
}

void ListGraph::getAdjacentEdgesDirect(int vertex, map<int, int>* m)	//Definition of getAdjacentEdges(Directed graph)
{
	for(auto iter = m_List[vertex].begin(); iter != m_List[vertex].end() ; iter++){
		m->insert({iter->first,iter->second});
	}
}

void ListGraph::insertEdge(int from, int to, int weight) //Definition of insertEdge
{
	if(from <= m_Size)
		m_List[from].insert({to,weight});
}

bool ListGraph::printGraph(ofstream *fout)	//Definition of print Graph
{
	for(int i = 1;i<=m_Size;i++){
		*fout<<i<<'\n';
		for(auto j = m_List[i].begin(); j!= m_List[i].end();j++){
			*fout<<j->first<<' '<<j->second<<'\n';
		}
	}

	return 1;
}