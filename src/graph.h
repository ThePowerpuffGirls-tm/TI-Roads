#include <algorithm>
#include <queue>
#include <stack>
#include <unordered_map>
#include <vector>

class Graph
{

    // Storage for edges and vertices
    std::unordered_map<int, std::vector<int>> mapGraph;
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
    void insertEdge(int from, int to);                                                          //Check?
    bool isEdge(int from, int to);                                                              //Check?
    std::vector<int> getAdjacent(int vertex);                                                   //Check?

    // Shortest s-t path
    bool bfs(int src, int dest, int parent[], int distance[]);
    bool dfs(int src, int dest, int parent[], int distance[]);
    void stPath(int src, int dest);
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

//Returns true if there exists an edge between 'to' and 'from'
bool Graph::isEdge(int from, int to)
{
    //Go to the 'from' node, and search it's vector for 'to'
    auto it = std::find(mapGraph[from].begin(), mapGraph[from].end(), to);

    //If 'to' is in the vector, then there is an edge.
    return (it != mapGraph[from].end());
}

//Returns a vector of all adjacent vertices
std::vector<int> Graph::getAdjacent(int vertex)
{
    //Vertex has no adjacent vertices
    if(mapGraph[vertex].size() == 0)
        return {};

    return mapGraph[vertex];
}

bool Graph::bfs(int src, int dest, int parent[], int distance[]) {
    std::queue<int> q;
    bool* visited = new bool[numVertices] {false};
    std::fill(parent, parent + numVertices, -1);
    std::fill(distance, distance + numVertices, INT_MAX);

    q.push(src);
    visited[src] = true;
    distance[src] = 0;

    while (!q.empty()) {
        int curr = q.front();
        q.pop();

        for (int i : mapGraph[curr]) {
            if (!visited[i]) {
                q.push(i);
                visited[i] = true;
                distance[i] = distance[curr] + 1;
                parent[i] = curr;

                if (i == dest) {
                    return true;
                }
            }
        }
    }

    delete[] visited;
    return false;
}

bool dfs(int src, int dest, int parent[], int distance[]) {
    std::stack<int> s;
    bool* visited = new bool[numVertices] {false};
    std::fill(parent, parent + numVertices, -1);
    std::fill(distance, distance + numVertices, INT_MAX);

    s.push(src);
    visited[src] = true;
    distance[src] = 0;

    while (!s.empty()) {
        int curr = s.front();
        s.pop();

        for (int i : mapGraph[curr]) {
            if (!visited[i]) {
                s.push(i);
                visited[i] = true;
                distance[i] = distance[curr] + 1;
                parent[i] = curr;

                if (i == dest) {
                    return true;
                }
            }
        }
    }

    delete[] visited;
    return false;
}

void Graph::stPath(int src, int dest) {
    int* parent = new int[numVertices];
    int* distance = new int[numVertices];

    if (!bfs(src, dest, parent, distance)) {
        return;
    }

    delete[] parent;
    delete[] distance;
}
