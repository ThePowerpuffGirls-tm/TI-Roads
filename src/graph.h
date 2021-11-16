#include <unordered_map>
#include <vector>
#include <algorithm>

class Graph
{

    //Storage for edges and vertices
    std::unordered_map<int, std::vector<int>> mapGraph;
    int numVertices;

    public:
    //ctors
    Graph(int _vertices);                           //Check?
    ~Graph();                                       //Check?

    //Other functions
    void insertEdge(int from, int to);              //Check?
    bool isEdge(int from, int to);                  //Check?
    std::vector<int> getAdjacent(int vertex);       //Check?

};

Graph::Graph(int _vertices)
{
    numVertices = _vertices;
}

Graph::~Graph()
{
    mapGraph.clear();
}

void Graph::insertEdge(int from, int to)    //Currently a undirected map, roads can go both ways
{
    //Whether or not vertex 'from' already exists, push_back create the vector or add to an existing vector
    mapGraph[from].push_back(to);

    //Create another edge from 'to' to 'from'
    mapGraph[to].push_back(from); 
}

std::vector<int> Graph::getAdjacent(int vertex)
{
    //Vertex has no adjacent vertices
    if(mapGraph[vertex].size() == 0)
        return {};

    return mapGraph[vertex];
}

bool Graph::isEdge(int from, int to)
{
    //Go to the 'from' node, and search it's vector for 'to'
    auto it = std::find(mapGraph[from].begin(), mapGraph[from].end(), to);

    //If 'to' is in the vector, then there is an edge.
    return (it != mapGraph[from].end());
}