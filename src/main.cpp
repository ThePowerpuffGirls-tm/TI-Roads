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
    int from, to;
    std::vector<int> distances;
    while(selection != 0)
    {

        std::cout << "Select algorithm (enter 0 to exit):" << std::endl;
        std::cout << "1. Dijkstra's Algorithm" << std::endl;
        std::cout << "2. Bellman-Ford Algorithm" << std::endl;
        std::cin >> selection;

        if(selection == 1)  //run dijkstras
        {
            std::cout << "Starting Vertex: ";
            std::cin >> from;
            std::cout << "Destination Vertex: ";
            std::cin >> to;
            distances = graph.dijkstra(from);
        }
        else if(selection == 2) //run bellman-ford
        {
            std::cout << "Starting Vertex: ";
            std::cin >> from;
            std::cout << "Destination Vertex: ";
            std::cin >> to;
            distances = graph.bellmanFord(from);
            std::cout << distances[to] << std::endl;
        }
        else if(selection == 0) //End program
        {
            break;
        }
        else
        {
            std::cout << "Invalid selection" << std::endl;
        }

        //whitespace/formatting
        //std::cout << selection << std::endl;

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
    int largest = 0;
    while(std::getline(reader, newLine))
    {
        rngWeight = rand()%1584+2113;
        space = newLine.find('	');
        from = stoi(newLine.substr(0, space));
        to = stoi(newLine.substr(space+1));
        std::cout << newLine << std::endl;
        if(!graph.isEdge(from, to))
        {
            graph.insertEdge(from, to, rngWeight);
        }

        if(from > largest)
        {
            largest = from;
        }
        if(to > largest)
        {
            largest = to;
        }
    }

    graph.vertexCorrection(largest);
    //std::cout << "largest ID: " << largest << std::endl;
    //auto stop = std::chrono::high_resolution_clock::now();
    //auto duration = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
    //std::cout << "Time to run: " << duration.count() << "s" <<std::endl;
    reader.close();

}