#include<iostream>
#include<iomanip>
#include<fstream>
#include<string>
#include<stdlib.h>
#include<chrono>
#include<stack>
#include"graph.h"

void readFile(Graph& graph,  std::string& filename);
void printOutput(std::pair<std::unordered_map<int, int>, std::unordered_map<int, int>>& output, int from, int to, int degs);

int main()
{
    Graph graph;
    std::string filename = "roadNet-TX.txt";
    readFile(graph, filename);

    int selection = -1;
    int from, to, degs;
    std::pair<std::unordered_map<int, int>, std::unordered_map<int, int>> output;

    std::cout << "Welcome to the TI-ROADS interface" << std::endl;
    std::cout << "This is a text-based interface for a graph representation of all intersections and roads in Texas*" << std::endl;
    std::cout << "[brought to you by ThePowerpuffGirls(tm)]\n" << std::endl;
    std::cout << "*(Disclaimer: this project is likely not an accurate representation of the road infrastructure in Texas)\n\n" << std::endl;

    while(selection != 0)
    {

        std::cout << "Please select an algorithm" << std::endl;
        std::cout << "1. Dijkstra's Algorithm" << std::endl;
        std::cout << "2. Bellman-Ford Algorithm" << std::endl;
        std::cout << "3. Find reachable vertices" << std::endl;

        std::cout << "Selection (enter 0 to exit): ";
        std::cin >> selection;

        std::cout << "\n" << std::endl;
        if(selection == 0) {break;}

        if(selection == 1)  //run dijkstras
        {
            std::cout << "Running Dijkstra's Algorithm" << std::endl;
            std::cout << "Starting Vertex: ";
            std::cin >> from;
            std::cout << "Destination Vertex: ";
            std::cin >> to;
            std::cout << "Maximum degrees from Source: ";
            std::cin >> degs;
            if(to == from)
            {
                std::cout << "\n-----------------------------------------------------------------------------------------------------------------------" << std::endl;
                std::cout << "Starting location is equivalent to the destination" << std::endl;
                std::cout << "Total distance traveled: 0 feet" << std::endl;
                std::cout << "-----------------------------------------------------------------------------------------------------------------------\n" << std::endl;
            }
            else
            {
                std::cout << "Calculating..." << std::endl;
                output = graph.dijkstra(from, degs);
                printOutput(output, from, to, degs);
            }
        }
        else if(selection == 2) //run bellman-ford
        {
            std::cout << "Running Bellman-Ford Algorithm" << std::endl;
            std::cout << "Starting Vertex: ";
            std::cin >> from;
            std::cout << "Destination Vertex: ";
            std::cin >> to;
            std::cout << "Maximum degrees from Source: ";
            std::cin >> degs;
            if(to == from)
            {
                std::cout << "\n-----------------------------------------------------------------------------------------------------------------------" << std::endl;
                std::cout << "Starting location is equivalent to the destination" << std::endl;
                std::cout << "Total distance traveled: 0 feet" << std::endl;
                std::cout << "-----------------------------------------------------------------------------------------------------------------------\n" << std::endl;
            }
            else
            {
                std::cout << "Calculating..." << std::endl;
                output = graph.bellmanFord(from, degs);
                printOutput(output, from, to, degs);
            }
        }
        else if(selection == 3) //find reachable vertices
        {
            std::cout << "Finding reachable vertices" << std::endl;
            std::cout << "Source Vertex: ";
            std::cin >> from;
            std::cout << "Maximum degrees from Source: ";
            std::cin >> degs;
            std::cout << "Calculating..." << std::endl;
            std::set<int> set = graph.subset(from, degs);
            std::cout << "-----------------------------------------------------------------------------------------------------------------------" << std::endl;
            std::cout << "All vertices \'" << degs << "\' degrees away from vertex " << from << ": \n" << std::endl;
            std::cout << "[";
            int counter = 1;
            for(int i : set)
            {
                if(i != from)
                {
                    std::cout << i;
                    if(counter < set.size())
                    {
                        std::cout << ", ";
                    }
                }

                counter++;
            }
            std::cout << "]" << std::endl;
            std::cout << "-----------------------------------------------------------------------------------------------------------------------\n" << std::endl;
        }
        else
        {
            std::cout << "Invalid selection" << std::endl;
        }

        //whitespace/formatting
        

    }
    
}

void readFile(Graph& graph, std::string& filename)
{
    //Timer start
    //auto start = std::chrono::high_resolution_clock::now();
    std::cout << "Loading data..." << std::endl;
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
    //Timer End
    /*
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
    std::cout << "Time to run: " << duration.count() << "s" <<std::endl;
    */
   
    reader.close();
    std::cout << "Information fully loaded. Beginning program execution." << std::endl;
    std::cout << "-----------------------------------------------------------------------------------------------------------------------\n\n" << std::endl;
}

void printOutput(std::pair<std::unordered_map<int, int>, std::unordered_map<int, int>>& output, int from, int to, int degs)
{
    std::unordered_map<int, int> distances = output.first;
    std::unordered_map<int, int> predecessors = output.second;
    std::stack<int> stack;
    std::cout << "\n-----------------------------------------------------------------------------------------------------------------------" << std::endl;
    if(distances[to] == 0 || distances[to] == INT_MAX)
    {
        std::cout << "No path was between " << from << " and " << to << " within \'" << degs << "\' degrees" << std::endl;
        std::cout << "-----------------------------------------------------------------------------------------------------------------------\n" << std::endl;
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
        std::cout << "No path was between " << from << " and " << to << " within \'" << degs << "\' degrees" << std::endl;
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
        std::cout << "\nEnd of path\n" << std::endl;
        std::cout << "Total distance traveled: " << distances[to] << " feet" << std::endl;

    }
    std::cout << "-----------------------------------------------------------------------------------------------------------------------\n" << std::endl;

}