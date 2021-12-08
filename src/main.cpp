#include<iostream>
#include<iomanip>
#include<fstream>
#include<string>
#include<stdlib.h>
#include"graph.h"
using namespace std;

void readFile(Graph& graph, string& filename);

int main()
{
    Graph graph;
    string filename = "roadNet-TX.txt";
    readFile(graph, filename);


}

void readFile(Graph& graph, string& filename)
{
    ifstream reader(filename);
	string newLine = "";

    //skip the first 4 lines
    for(int i = 0; i < 4; i++)
    {
        getline(reader, newLine);
    }

    int from, to, space;
    int rngWeight;
    while(getline(reader, newLine))
    {
        rngWeight = rand()%1584+2113;
        space = newLine.find(' ');
        from = stoi(newLine.substr(0, space));
        to = stoi(newLine.substr(space+1));
        if(!graph.isEdge(from, to))
        {
            graph.insertEdge(from, to, rngWeight);
            graph.insertEdge(to, from, rngWeight);
        }
    }

    reader.close();

}