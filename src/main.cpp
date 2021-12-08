#include<iostream>
#include<iomanip>
#include<fstream>
#include<string>
#include<stdlib.h>
#include<chrono>
#include"graph.h"

void readFile(Graph& graph,  std::string& filename);

int main()
{
    Graph graph;
    std::string filename = "roadNet-TX.txt";
    readFile(graph, filename);

    int selection = -1;
    while(selection != 0)
    {

        std::cout << "Select algorithm (enter 0 to exit):" << std::endl;
        std::cout << "1. Dijkstra's Algorithm" << std::endl;
        std::cout << "2. Bellman-Ford Algorithm" << std::endl;
        std::cin >> selection;

        if(selection == 1)
        {
            //run dijkstras
        }
        else if(selection == 2)
        {
            //run bellman-ford
        }
        else if(selection == 0)
        {
            break;
        }
        else
        {
            std::cout << "Invalid selection" << std::endl;
        }

        //whitespace/formatting
        std::cout << selection << std::endl;

    }
}

void readFile(Graph& graph, std::string& filename)
{
    //auto start = std::chrono::high_resolution_clock::now();
    std::ifstream reader(filename);
	std::string newLine = "";

    //skip the first 4 lines
    for(int i = 0; i < 4; i++)
    {
        std::getline(reader, newLine);
    }

    int from, to, space;
    int rngWeight;
    while(std::getline(reader, newLine))
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
    //auto stop = std::chrono::high_resolution_clock::now();
    //auto duration = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
    //std::cout << duration.count() << std::endl;
    reader.close();

}