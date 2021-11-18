#include <unordered_map>
#include <vector>
#include <algorithm>

class Graph
{

    //Storage for edges and vertices
    std::unordered_map<int, std::vector<int>> mapGraph;
    int numVertices;
    int numEdges;

    public:
    //ctors
    Graph();
    
    Graph(int _vertices, int _edges);           //Dunno if we need this                         //Check?
    ~Graph();                                                                                   //Check?

    int V();
    int E();

    //Other functions
    void insertEdge(int from, int to);                                                          //Check?
    bool isEdge(int from, int to);                                                              //Check?
    std::vector<int> getAdjacent(int vertex);                                                   //Check?

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

void Graph::insertEdge(int from, int to)    //Currently a undirected map, roads can go both ways
{
    //Whether or not vertex 'from' already exists, push_back create the vector or add to an existing vector
    if(!mapGraph.count(from))       //Vertex does not exist.
        numVertices++;
    mapGraph[from].push_back(to);

    //Create another edge from 'to' to 'from'
    if(!mapGraph.count(to))         //Other vertex does not exist.
        numVertices++;
    mapGraph[to].push_back(from); 

    numEdges++;
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