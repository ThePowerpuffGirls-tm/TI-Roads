#include <unordered_map>
#include <vector>

class Graph
{

    //Storage for edges and vertices
    std::unordered_map<int, std::vector<int>> g;
    int numVertices;

    public:
    //ctors
    Graph();
    ~Graph();

    //Other functions
    void insertEdge(int from, int to);
    bool isEdge(int from, int to);
    std::vector<int> getAdjacent(int vertex);

};

Graph::Graph()
{

}

Graph::~Graph()
{

}