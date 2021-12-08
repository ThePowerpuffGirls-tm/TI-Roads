#include <algorithm>
#include <unordered_map>
#include <utility>
#include <vector>
#include <set>

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

    std::vector<int> shortestPath(int src);

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

//Creates an edge between two vertices, inserts vertex if it does not already exist.
void Graph::insertEdge(int from, int to, int weight)    //Currently a undirected map, roads can go both ways
{
    //Whether or not vertex 'from' already exists, push_back create the vector or add to an existing vector
    if(!mapGraph.count(from))       //Vertex does not exist.
        numVertices++;
    mapGraph[from].push_back(std::make_pair(to, weight));

    //Create another edge from 'to' to 'from'
    if(!mapGraph.count(to))         //Other vertex does not exist.
        numVertices++;
    mapGraph[to].push_back(std::make_pair(from, weight)); 

    numEdges++;
}

//Returns true if there exists an edge between 'to' and 'from'
bool Graph::isEdge(int from, int to)
{
    //Go to the 'from' node, and search it's vector for 'to'
    auto it = std::find(mapGraph[from].begin(), mapGraph[from].end(), to);

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


/*This algorithm finds the shortest path from the source node to all vertices.
    Uses Dijkstra's shortest path algorithm
            Sources: Aman's Lecture Slides
                     https://www.geeksforgeeks.org/dijkstras-shortest-path-algorithm-greedy-algo-7/ */
std::vector<int> Graph::shortestPath(int src)
{
    std::set<int> visited;      //Set of visited nodes.
    std::set<int> unvisited;    //Set of unvisited nodes.

    //All nodes initialized as unvisited.
    for(int i = 0; i < mapGraph.size(); i++)
        unvisited.insert(i);

    //Distance initialized as 0 for src, and infinity for the rest.
    std::vector<int> distance(mapGraph.size(), INT_MAX);
    distance.at(src) = 0;

    //While there are nodes left unvisited:
    while(!unvisited.empty())
    {
        //For all unvisited nodes, find the smallest distance
        int curr = *unvisited.begin();
        for(auto i = unvisited.begin(); i != unvisited.end(); i++)
        {
            //Check distance values
            if(distance.at(*i) < distance.at(curr))
                curr = *i;          //Update distance if there exists a smaller distance
        }

        //Mark the unvisited node as visited
        unvisited.erase(curr);
        visited.insert(curr);

        //For all adjacent nodes
        auto adj = mapGraph.at(curr);
        for(std::pair<int, int> p : adj)
        {
            //Check distance
            int currDistance = distance.at(curr) + p.second;

            //Perform relaxation if necessary
            if(distance.at(p.first) > currDistance)
                distance.at(p.first) = currDistance;
        }
    }
    return distance;
}
