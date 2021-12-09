#include<iostream>
#include<iomanip>
#include<fstream>
#include<string>
#include<stdlib.h>
#include<chrono>
#include<stack>
#include"graph.h"

void readFile(Graph& graph,  std::string& filename);
void printOutput(std::pair<std::unordered_map<int, int>, std::unordered_map<int, int>>& output, int from, int to);

int main()
{
    Graph graph;
    std::string filename = "roadNet-TX.txt";
    readFile(graph, filename);

    int selection = -1;
    int from, to, degs;
    std::pair<std::unordered_map<int, int>, std::unordered_map<int, int>> output;
    
    while(selection != 0)
    {

        std::cout << "Select algorithm (enter 0 to exit):" << std::endl;
        std::cout << "1. Dijkstra's Algorithm" << std::endl;
        std::cout << "2. Bellman-Ford Algorithm" << std::endl;
        std::cin >> selection;

        if(selection == 0) {break;}

        std::cout << "Starting Vertex: ";
        std::cin >> from;
        std::cout << "Degrees from Source: ";
        std::cin >> degs;
        std::cout << "Destination Vertex: ";
        std::cin >> to;

        if(to == from)
        {
            std::cout<< "Starting from destination (0 feet)" << std::endl;
            std::cout << "Total distance traveled: 0 feet" << std::endl;
        }
        if(selection == 1)  //run dijkstras
        {
            output = graph.dijkstra(from, degs);
            printOutput(output, from, to);
        }
        else if(selection == 2) //run bellman-ford
        {
            output = graph.bellmanFord(from, degs);
            printOutput(output, from, to);
        }
        else
        {
            //std::cout << "Invalid selection" << std::endl;
            std::cout << "Source Vertex: ";
            std::cin >> from;
            std::cout << "Max Degrees Away: ";
            std::cin >> to;
            std::set<int> set = graph.subset(from, to);
            /*
            for(int i : set)
            {
                std::cout << i << ", ";
            }
            std::cout << std::endl;
            */
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
        //randomly generate the weight for this edge
        rngWeight = rand()%1584+2113;

        //parse the line for the two vertices
        space = newLine.find('	');
        from = stoi(newLine.substr(0, space));
        to = stoi(newLine.substr(space+1));

        //std::cout << from << " | " << to << std::endl;
        if(!graph.isEdge(from, to))
        {
            graph.insertEdge(from, to, rngWeight);
        }

        if(from > largest)
            largest = from;
        if(to > largest)
            largest = to;
        
    }

    graph.vertexCorrection(largest);
    //std::cout << "largest ID: " << largest << std::endl;
    //auto stop = std::chrono::high_resolution_clock::now();
    //auto duration = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
    //std::cout << "Time to run: " << duration.count() << "s" <<std::endl;
    reader.close();

}

void printOutput(std::pair<std::unordered_map<int, int>, std::unordered_map<int, int>>& output, int from, int to)
{
    std::unordered_map<int, int> distances = output.first;
    std::unordered_map<int, int> predecessors = output.second;
    std::stack<int> stack;

    if(distances[to] == 0 || distances[to] == INT_MAX)
    {
        std::cout << "No path between " << from << " and " << to << std::endl;
        return;
    }


    stack.push(to);
    int parent = predecessors[to];

    while(parent != -1)
    {
        //std::cout << "pushed: " << parent << " | ";
        stack.push(parent);
        parent = predecessors[parent];
        //std::cout << "Next up: " << parent << std::endl;
    }
    
    if(stack.top() != from)
    {
        std::cout << "No path between " << from << " and " << to << std::endl;
    }
    else
    {
        std::cout << "Shortest path from " << from << " to " << to << ": " << std::endl;
        int curr, next, distance;

        while(stack.size() > 1)
        {
            curr = stack.top();
            stack.pop();
            //std::cout << "Size of stack: " << stack.size() << std::endl;
            next = stack.top();
            distance = distances[next] - distances[curr];
            //std::cout << "Next: " << distances[next] << " | Curr: " << distances[curr] << std::endl;
            std::cout << "Direction:\t" << curr << "\tto\t" << next << "\t(" << distance << " feet)" << std::endl;
        }

        std::cout << "Total distance traveled: " << distances[to] << " feet" << std::endl;
        std::cout << "\nEnd of path" << std::endl;
        std::cout << "-----------------------------------------------\n" << std::endl;
    }

}