#include <algorithm>
#include <limits.h>
#include <set>
#include <unordered_map>
#include <utility>
#include <vector>
#include<iostream>
#include<chrono>

class Graph
{
    // Storage for edges and vertices, adjancency list implementation
    std::unordered_map<int, std::vector<std::pair<int, int>>> mapGraph;
    int numVertices;
    int numEdges;

    public:
    // Constructors and destructors
    Graph();
    Graph(int _vertices, int _edges);           //Dunno if we need this                         //Check?
    ~Graph();                                                                                   //Check?

    // Accessors
    int V();
    int E();

    // Graph functions
    void insertEdge(int from, int to, int weight);                                              //Check?
    bool isEdge(int from, int to);                                                              //Check?
    std::vector<std::pair<int, int>> getAdjacent(int vertex);                                   //Check?
    
    // Shortest s-t path
    std::vector<int> dijkstra(int src);
    std::vector<int> bellmanFord(int src);

    //Misc.
    void vertexCorrection(int largestID);
};

Graph::Graph()
{
    numVertices = 0;
    numEdges = 0;
}

Graph::Graph(int _vertices, int _edges)     //Do we need this?
{
    numVertices = _vertices;
    numEdges = _edges;
}

Graph::~Graph()
{
    mapGraph.clear();
}

//Returns the number of vertices
int Graph::V()  { return this->numVertices; }

//Returns the number of edges
int Graph::E()  { return this->numEdges;    }  

void Graph::vertexCorrection(int largestID)
{
    std::vector<int> extraVertices;
    for(int i = 0; i < largestID; i++)
    {
        if(mapGraph[i].size() == 0)
        {
            extraVertices.push_back(i);
        }
    }

    int rngWeight;
    for(int i = 0; i<extraVertices.size(); i++)
    {
        rngWeight = rand()%1584+2113;
        insertEdge(i, (i+1)%extraVertices.size(), rngWeight);
    }


}

//Creates an edge between two vertices, inserts vertex if it does not already exist.
void Graph::insertEdge(int from, int to, int weight)    //Currently a undirected map, roads can go both ways
{
    //Whether or not vertex 'from' already exists, push_back create the vector or add to an existing vector
    if(mapGraph[from].size() == 0)       //Vertex does not exist.
        numVertices++;
    mapGraph[from].push_back(std::make_pair(to, weight));

    //Create another edge from 'to' to 'from'
    if(mapGraph[to].size() == 0)         //Other vertex does not exist.
        numVertices++;
    mapGraph[to].push_back(std::make_pair(from, weight)); 

    numEdges++;
}

//Returns true if there exists an edge between 'to' and 'from'
bool Graph::isEdge(int from, int to)
{
    //Go to the 'from' node, and search it's vector for 'to'
    auto it = std::find_if(mapGraph[from].begin(), mapGraph[from].end(), [&to](const std::pair<int, int>& element) {return element.first == to;});

    //If 'to' is in the vector, then there is an edge.
    return (it != mapGraph[from].end());
}

//Returns a vector of all adjacent vertices
std::vector<std::pair<int, int>> Graph::getAdjacent(int vertex)
{
    //Vertex has no adjacent vertices
    if(mapGraph[vertex].size() == 0)
        return {};

    return mapGraph[vertex];
}

std::vector<int> Graph::bellmanFord(int src)
{
    //Timer Start
    auto start = std::chrono::high_resolution_clock::now();

    // Step 1 - declare distance and parent arrays and initialize elements accordingly
    std::vector<int> distance(mapGraph.size(), INT_MAX);
    distance[src] = 0;
    std::vector<int> parent(mapGraph.size(), -1);
    
    // Step 2 - relax all edges |V| - 1 times
    for (int i = 0; i < numVertices - 1; i++)
    {
        for (auto j : mapGraph)
        {
            for (std::pair<int, int> k : j.second)
            {
                if (distance[j.first] != INT_MAX && distance[j.first] + k.second < distance[k.first])
                {
                    distance[k.first] = distance[j.first] + k.second;
                    parent[k.first] = j.first;
                }
            }
        }
    }

    // Skip step 3 since there are no negative weights in the graph

    //Timer End
    
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "Time to run: " <<  duration.count() << "ms" << std::endl;
    
    return distance;
}

/*This algorithm finds the shortest path from the source node to all vertices.
    Uses Dijkstra's shortest path algorithm
            Sources: Aman's Lecture Slides
                     https://www.geeksforgeeks.org/dijkstras-shortest-path-algorithm-greedy-algo-7/ */
std::vector<int> Graph::dijkstra(int src)
{
    //Timer Start
    auto start = std::chrono::high_resolution_clock::now();

    std::set<int> visited;      //Set of visited nodes.
    std::set<int> unvisited;    //Set of unvisited nodes.

    //All nodes initialized as unvisited.
    for(auto p : mapGraph)
        unvisited.insert(p.first);

    //Distance initialized as 0 for src, and infinity for the rest.
    std::vector<int> distance(mapGraph.size(), INT_MAX);
    distance.at(src) = 0;

    std::vector<int> predecessor(mapGraph.size(), src);
    predecessor.at(src) = -1;

    //Start from src vertex
    unvisited.erase(src);
    visited.insert(src);
    for(std::pair<int, int> p : mapGraph[src])
    {
        distance[p.first] = p.second;
    }


    //While there are nodes left unvisited:
    auto preAdd = std::chrono::high_resolution_clock::now();
    int counter = 0;
    while(!unvisited.empty())
    {
        counter++;  
        //For all unvisited nodes, find the smallest distance
        int curr = *(unvisited.begin());
        for(int i : unvisited)
        {
            //Check distance values
            if(distance.at(i) < distance.at(curr))
                curr = i;          //Update distance if there exists a smaller distance
        }

        //Mark the unvisited node as visited
        unvisited.erase(curr);
        visited.insert(curr);

        //For all adjacent nodes
        //std::cout << "HI " << curr;
        auto adj = mapGraph.at(curr);
        //std::cout << "| BYE " << curr << std::endl;
        for(std::pair<int, int> p : adj)
        {
            if(unvisited.count(p.first) != 0)
            {
                //Check distance
                int currDistance = distance.at(curr) + p.second;

                //Perform relaxation if necessary
                if(distance.at(p.first) > currDistance)
                {
                    distance.at(p.first) = currDistance;
                    predecessor.at(p.first) = curr;
                }
            }
        }
        if(counter%10000 == 0)
        {
            auto postAdd = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(postAdd - preAdd);
            std::cout << "Adding 10000 vertices: " << duration.count() << "ms" << std::endl;
            preAdd = std::chrono::high_resolution_clock::now();
        }
    }

    //Timer End
    
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "Time to run: " << duration.count() << "ms" << std::endl;
    
    return distance;
}
