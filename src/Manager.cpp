#include "Manager.h"
#include "GraphMethod.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <string>

Manager::Manager()	
{
	graph = nullptr;	
	fout.open("log.txt", ios::app);
	load = 0;	//Anything is not loaded
	cmd = new char[256];
}

Manager::~Manager()
{
	if(load)	//if graph is loaded, delete graph
		delete graph;	
	if(fout.is_open())	//if fout is opened, close file
		fout.close();	//close log.txt File
}

void Manager::run(const char* command_txt){
	ifstream fin;	//Command File Input File Stream
	fin.open(command_txt, ios_base::in);//File open with read mode
		
	if(!fin) { //If command File cannot be read, Print error
		fout<<"command file open error"<<endl;
		return;	//Return
	}

	while(!fin.eof()){
		fin.getline(cmd,256);
		char* p = strtok(cmd," ");

		if(!strcmp(p,"LOAD")){
			p = strtok(NULL,"\r");

			if(load){ 	// if graph already exist
				delete graph;		// remove the graph
				graph = nullptr;
				load = 0;
			}

			if(!strcmp(p,"graph_L.txt")){
				LOAD("graph_L.txt");
				load = 1;
			}
			else if(!strcmp(p,"graph_M.txt")){
				LOAD("graph_M.txt");
				load = 1;
			}
			else{
				printErrorCode(100);
			}
		}
		else if(!strcmp(p,"PRINT")){
			if(load)
				PRINT();
			else
				printErrorCode(200);
		}

		else if(!strcmp(p,"BFS")){
			p = strtok(NULL," ");

			if(!strcmp(p,"Y") || !strcmp(p,"N")){	// BFS
				char option = p[0];

				p = strtok(NULL," ");
				if(strlen(p))
					mBFS(option,atoi(p));
				else
					printErrorCode(300);
			}
			else{
				printErrorCode(300);
			}
		}
		else if(!strcmp(p,"DFS")){
			p = strtok(NULL," ");

			if(!strcmp(p,"Y") || !strcmp(p,"N")){	// BFS
				char option = p[0];

				p = strtok(NULL," ");
				if(strlen(p))
					mDFS(option,atoi(p));
				else
					printErrorCode(400);
			}
			else{
				printErrorCode(400);
			}
		}
		else if(!strcmp(p,"KWANGWOON")){
			//mKwoonWoon()
		}
		else if(!strcmp(p,"KRUSKAL")){
			mKRUSKAL();
		}



	}



	
	fin.close();
	return;
}

bool Manager::LOAD(const char* filename)
{
	ifstream fgraph;
	fgraph.open(filename);

	if(!fgraph) { //If command File cannot be read, Print error
		printErrorCode(100);
		return 0;	//Return
	}

	fgraph.getline(cmd,256);

	if(!strcmp(cmd,"L")){
		unsigned int vertexCnt = 0;
		int vertex = 0;

		fgraph.getline(cmd,256);

		vertexCnt = atoi(cmd);

		graph = new ListGraph(0,vertexCnt);

		while(!fgraph.eof()){
			fgraph.getline(cmd,256);

			int len = strlen(cmd);
			int blank = 0;
			for(int i = 0;i<len;i++){
				if(cmd[i]==' ') blank++;
			}

			if(blank == 0){	// new vertex
				vertex = atoi(cmd);
			}
			else{
				char* toVertexChar = strtok(cmd," ");
				int toVertexInt = atoi(toVertexChar);

				char* weightChar = strtok(NULL," ");
				int weight = atoi(weightChar);

				graph->insertEdge(vertex,toVertexInt,weight);
			}
		}
	}
}

bool Manager::PRINT()	
{
	fout<<"======== PRINT========\n";
	//[1]-> (2,6) -> (3,2)

	if(graph->getType() == 0){
		for(int i = 1;i<=graph->getSize();i++){
			map<int,int>* printgraph = new map<int,int>;
			graph->getAdjacentEdgesDirect(i,printgraph);

			fout<<'['<<i<<']';
			for(auto iter = printgraph->begin();iter!=printgraph->end();iter++){
				fout<<"-> ("<<iter->first<<','<<iter->second<<") ";
			}
			fout<<'\n';
		}
	}
	else{

		// To Do:
		// Matrix print

	}

	fout<<"=====================\n\n";

}

bool Manager::mBFS(char option, int vertex)	
{


	// To Do :
	// add vertex check condition



	BFS(graph,option,vertex,&fout);
}

bool Manager::mDFS(char option, int vertex)	
{


	// To Do :
	// add vertex check condition


	DFS(graph,option,vertex,&fout);

	
}

bool Manager::mDIJKSTRA(char option, int vertex)	
{
	
}

bool Manager::mKRUSKAL()
{
	if(load)
		Kruskal(graph,&fout);
	else
		printErrorCode(600);

	return 1;
}

bool Manager::mBELLMANFORD(char option, int s_vertex, int e_vertex) 
{
	
}

bool Manager::mFLOYD(char option)
{
	
}

bool Manager::mKwoonWoon(int vertex) {
	
}

void Manager::printErrorCode(int n)
{
	fout<<"========ERROR======="<<endl;
	fout<<n<<endl;
	fout<<"===================="<<endl << endl;
}


