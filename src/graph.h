#include <algorithm>
#include <limits.h>
#include <unordered_map>
#include <utility>
#include <vector>

class Graph
{
    // Storage for edges and vertices
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
    std::vector<std::pair<int, int>> getAdjacent(int vertex);                                                   //Check?

    // Shortest s-t path
    void bellmanFord(int src, int dest);
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

void Graph::bellmanFord(int src, int dest)
{
    // Step 1 - declare distance and parent arrays and initialize elements accordingly
    int* distance = new int[numVertices];
    std::fill(distance, distance + numVertices, INT_MAX);
    distance[src] = 0;
    
    int* parent = new int[numVertices];
    std::fill(parent, parent + numVertices, -1);

    // Step 2 - relax all edges |V| - 1 times
    for (int i = 0; i < numVertices - 1; i++)
    {
        for (std::pair<int, int> j : mapGraph[i])
        {
            if (distance[i] != INT_MAX && distance[i] + j.second < distance[j.first])
            {
                distance[j.first] = distance[i] + j.second;
                parent[j.first] = i;
            }
        }
    }

    // Skip step 3 since there are no negative weights in the graph
    delete[] distance;
    delete[] parent;
}
