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
	
	delete[] cmd;
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
				if(strlen(p)){
					int vertex = atoi(p);
					p = strtok(NULL," ");
					if(p) printErrorCode(300);

					mBFS(option,vertex);
				}
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
				if(strlen(p)){
					int vertex = atoi(p);
					p = strtok(NULL," ");
					if(p) printErrorCode(400);

					mDFS(option,vertex);
				}
				else
					printErrorCode(400);
			}
			else{
				printErrorCode(400);
			}
		}
		else if(!strcmp(p,"KWANGWOON")){
			mKwoonWoon(1);
		}
		else if(!strcmp(p,"KRUSKAL")){
			mKRUSKAL();
		}
		else if(!strcmp(p,"DIJKSTRA")){
			p = strtok(NULL," ");

			if(!strcmp(p,"Y") || !strcmp(p,"N")){	// BFS
				char option = p[0];

				p = strtok(NULL," ");
				if(strlen(p)){
					int vertex = atoi(p);
					p = strtok(NULL," ");
					if(p) printErrorCode(700);

					mDIJKSTRA(option,vertex);
				}
				else
					printErrorCode(700);
			}
			else{
				printErrorCode(700);
			}
		}
		else if(!strcmp(p,"BELLMANFORD")){
			p = strtok(NULL," ");
			if(!strcmp(p,"Y") || !strcmp(p,"N")){	// BFS
				char option = p[0];

				p = strtok(NULL," ");
				if(strlen(p)){
					int s_v = atoi(p);

					p = strtok(NULL," ");
					if(strlen(p)){
						int e_v = atoi(p);

						p = strtok(NULL," ");
						if(p) printErrorCode(800);

						mBELLMANFORD(option,s_v,e_v);
					}
					else{
						printErrorCode(800);
					}
				}
				else
					printErrorCode(800);
			}
			else{
				printErrorCode(800);
			}
		}
		else if(!strcmp(p,"FLOYD")){
			p = strtok(NULL," ");

			if(!strcmp(p,"Y") || !strcmp(p,"N")){
				mFLOYD(p[0]);
			}
			else{
				printErrorCode(900);
			}
		}
		else if(!strcmp(p,"EXIT")){
			fin.close();
			return;	
		}
		else{
			printErrorCode(1000);
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
	else if(!strcmp(cmd,"M")){
		unsigned int vertexCnt = 0;
		int vertex = 0;

		fgraph.getline(cmd,256);
		vertexCnt = atoi(cmd);
		graph = new MatrixGraph(1,vertexCnt);

		for(int i = 1;i<=vertexCnt;i++){
			fgraph.getline(cmd,256);

			char* p = strtok(cmd," ");

			for(int j = 1;j<=vertexCnt;j++){
				graph->insertEdge(i,j,atoi(p));

				p = strtok(NULL," ");
			}
		}
	}

	fout<<"======== LOAD ========\n";
	fout<<"Success\n";
	fout<<"=====================\n\n";
}

bool Manager::PRINT()	
{
	graph->printGraph(&fout);

}

bool Manager::mBFS(char option, int vertex)	
{
	if(load){
		if(0 < vertex && vertex <= graph->getSize())
			BFS(graph,option,vertex,&fout);
		else
			printErrorCode(300);
	}
	else
		printErrorCode(300);
}

bool Manager::mDFS(char option, int vertex)	
{
	if(load){
		if(0 < vertex && vertex <= graph->getSize())
			DFS(graph,option,vertex,&fout);
		else
			printErrorCode(400);
	}
	else
		printErrorCode(400);
}

bool Manager::mDIJKSTRA(char option, int vertex)	
{
	if(load){
		if(0 < vertex && vertex <= graph->getSize())
			Dijkstra(graph,option,vertex,&fout);
		else
			printErrorCode(700);
	}
	else
		printErrorCode(700);
}

bool Manager::mKRUSKAL()
{
	if(load){
		if(!Kruskal(graph,&fout)){
			printErrorCode(600);
		}
	}
	else
		printErrorCode(600);
}

bool Manager::mBELLMANFORD(char option, int s_vertex, int e_vertex) 
{
	if(load){
		if(0 < s_vertex && s_vertex <= graph->getSize() && 0 < e_vertex && e_vertex <= graph->getSize()){
			if(!Bellmanford(graph,option,s_vertex,e_vertex,&fout)){
				printErrorCode(800);
			}
		}
		else
			printErrorCode(800);
	}
	else
		printErrorCode(800);
}

bool Manager::mFLOYD(char option)
{
	if(load){
		if(!FLOYD(graph,option,&fout))
			printErrorCode(900);
	}
	else
		printErrorCode(900);
}

bool Manager::mKwoonWoon(int vertex) {
	if(load)
		KWANGWOON(graph,vertex,&fout);
	else
		printErrorCode(500);
}

void Manager::printErrorCode(int n)
{
	fout<<"========ERROR======="<<endl;
	fout<<n<<endl;
	fout<<"===================="<<endl << endl;
}


