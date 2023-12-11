#include "MatrixGraph.h"
#include <iostream>
#include <vector>
#include <string>

MatrixGraph::MatrixGraph(bool type, int size) : Graph(type, size)
{
    m_Mat = new int*[size+1];
    m_Mat[0] = new int[(size+1)*(size+1)];

    for(int i = 1;i<size+1;i++){
        m_Mat[i] = &m_Mat[0][(size+1)* i];
    }

    for(int i = 0;i<=size;i++){
        for(int j = 0;j<=size;j++){
            m_Mat[i][j] = 0;
        }
    }

}

MatrixGraph::~MatrixGraph()
{
    delete[] m_Mat[0];
    delete[] m_Mat;
}

void MatrixGraph::getAdjacentEdges(int vertex, map<int, int>* m)
{	
    // Insert edge from the vertex
    for(int j = 1;j<=m_Size;j++){
        if(m_Mat[vertex][j] != 0){
            m->insert({j,m_Mat[vertex][j]});
        }
    }

    // Insert edge to the vertex
    for(int i = 1;i<=m_Size;i++){
        if(m_Mat[i][vertex] != 0){
            m->insert({i,m_Mat[i][vertex]});
        }
    }
}

void MatrixGraph::getAdjacentEdgesDirect(int vertex, map<int, int>* m)
{
    // Insert edge from the vertex
	for(int j = 1;j<=m_Size;j++){
        if(m_Mat[vertex][j] != 0){
            m->insert({j,m_Mat[vertex][j]});
        }
    }
}

void MatrixGraph::insertEdge(int from, int to, int weight)	
{
	m_Mat[from][to] = weight;
}

bool MatrixGraph::printGraph(ofstream *fout)	
{
	*fout<<"======== PRINT========\n\t";

    for(int i = 1;i<=m_Size;i++){
        *fout<<"["<<i<<"]\t";
    }

    *fout<<'\n';

    for(int i = 1;i<=m_Size;i++){
        *fout<<"["<<i<<"]\t";
        for(int j = 1;j<=m_Size;j++){
            *fout<<m_Mat[i][j]<<'\t';
        }
        *fout<<'\n';
    }

    *fout<<"=====================\n\n";

}
