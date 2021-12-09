#include <limits.h>
#include <algorithm>
#include <utility>
#include <iostream>
#include <chrono>
#include <set>
#include <queue>
#include <unordered_map>
#include <vector>


class Graph
{
    private: 
    // Storage for edges and vertices, adjancency list implementation
    std::unordered_map<int, std::vector<std::pair<int, int>>> mapGraph; //Represents a weighted, undirected graph
    int numVertices;
    int numEdges;
    
    public:
    // Constructors and destructors
    Graph();
    Graph(int _vertices, int _edges); //Unused
    ~Graph();                                                                                   

    // Accessors
    int V();
    int E();

    // Graph functions
    void insertEdge(int from, int to, int weight);                                              
    bool isEdge(int from, int to);     
    std::set<int> subset(int src, int degs);                                                         
    std::vector<std::pair<int, int>> getAdjacent(int vertex); //Unused                                   
    
    // Shortest s-t path
    std::pair<std::unordered_map<int, int>, std::unordered_map<int, int>> dijkstra(int src, int degs);
    std::pair<std::unordered_map<int, int>, std::unordered_map<int, int>> bellmanFord(int src, int degs);

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

/***
 * Purpose: Fills up all consecutive vertices up to the largest vertex ID. 
 *          All newly inserted vertices are added to a single disjoint component of the graph
 *          that are connected in one large cycle. This is primarily used so that 
 *          any and all integer values up to the largest ID can be referenced without issue.
 * 
 * Parameters: int largestID - the largest value the graph should be filled up to
 * Return:     N/A
 ***/
void Graph::vertexCorrection(int largestID)
{
    //Identify which vertices to add
    std::vector<int> extraVertices;
    for(int i = 0; i < largestID; i++)
    {
        if(mapGraph[i].size() == 0)
        {
            extraVertices.push_back(i);
        }
    }

    //Add those vertices and connect them to the disjoint section
    int rngWeight;
    for(int i = 0; i<extraVertices.size(); i++)
    {
        rngWeight = rand()%1584+2113;
        insertEdge(extraVertices[i], extraVertices[(i+1)%extraVertices.size()], rngWeight);
    }
}


/***
 * Purpose: Creates an edge between two vertices, inserts vertex if it does not already exist.
 *          This is done by updating the adjacency list of both vertices for the undirected graph.
 * Parameters:  int from - source vertex
 *              int to - destination vertex
 *              int weight - edge weight between the vertices
 * Return:      N/A
 ***/

void Graph::insertEdge(int from, int to, int weight) 
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

/***
 * Purpose: Check to see if there's an edge between the two given vertices
 * Parameters:  int from - Source vertex
 *              int to   - Destination vertex
 * Return:      true if there exists an edge between 'to' and 'from', false otherwise.
 ***/
bool Graph::isEdge(int from, int to)
{
    //Go to the 'from' node, and search it's vector for 'to'
    auto it = std::find_if(mapGraph[from].begin(), mapGraph[from].end(), [&to](const std::pair<int, int>& element) {return element.first == to;});

    //If 'to' is in the vector, then there is an edge.
    return (it != mapGraph[from].end());
}

/***
 * Purpose: Gets the list of all vertices adjacent to the target vertex
 * Parameters: int vertex - the target vertex.
 * Return:     A vector of all <vertex, weight> pairs representing all vertices adjacent to the target
 ***/
std::vector<std::pair<int, int>> Graph::getAdjacent(int vertex)
{
    //Vertex has no adjacent vertices
    if(mapGraph[vertex].size() == 0)
        return {};

    return mapGraph[vertex];
}

/***
 * Purpose: Uses the Bellman-Ford algorithm to find the shortest path from a source vertex to all other
 *          vertices within x degrees/edges from the source vertex. This function utilizes subset()
 *          function to generate a subset of all vertices x degrees from the source and applies the
 *          algorithm to the returned subset.
 * Parameters: int src - Source vertex
 *             int degs - the maximum amount of edges/degrees away from the Source vertex to look from
 * Return:     A pair of maps <distances, predecessors> representing the table of values generated by
 *             the Bellman-Ford algorithm. 
 ***/
std::pair<std::unordered_map<int, int>, std::unordered_map<int, int>>  Graph::bellmanFord(int src, int degs)
{
    //Timer Start
    //auto start = std::chrono::high_resolution_clock::now();

    // Step 1 - declare distance and parent arrays and initialize elements accordingly
    std::unordered_map<int, int> distance;
    std::unordered_map<int, int> parent;
    std::set<int> sub = subset(src, degs); 
    for(int i : sub)
    {   
        distance[i] = INT_MAX;
        parent[i] = -1;
    }
    distance[src] = 0;

    // Step 2 - relax all edges |V| - 1 times
    for (int i = 0; i < sub.size() - 1; i++)
    {
        for (int vertex : sub)
        {
            std::vector<std::pair<int, int>> vector = mapGraph[vertex];
            for (std::pair<int, int> k : vector)
            {
                if (distance[vertex] != INT_MAX && distance[vertex] + k.second < distance[k.first])     
                {
                    distance[k.first] = distance[vertex] + k.second;
                    parent[k.first] = vertex;
                }
            }
        }
    }

    // Skip step 3 since there are no negative weights in the graph

    //Timer End
    /*
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "Time to run: " <<  duration.count() << "ms" << std::endl;
    */
    return make_pair(distance, parent);
}


/***
 * Purpose: Uses Dijkstra's algorithm to find the shortest path from a source vertex to all other
 *          vertices within x degrees/edges from the source vertex. This function utilizes subset()
 *          to generate a subset of all vertices x degrees from the source and applies the
 *          algorithm to the returned subset.
 * Parameters: int src - Source vertex
 *             int degs - the maximum amount of edges/degrees away from the Source vertex to look from
 * Return:     A pair of maps <distances, predecessors> representing the table of values generated by
 *             Dijkstra's algorithm. 
 ***/
/*
References Used:    Aman's Lecture Slides
                    https://www.geeksforgeeks.org/dijkstras-shortest-path-algorithm-greedy-algo-7/ 
*/
std::pair<std::unordered_map<int, int>, std::unordered_map<int, int>> Graph::dijkstra(int src, int degs)
{
    //Timer Start
    //auto start = std::chrono::high_resolution_clock::now();

    std::set<int> visited;      //Set of visited nodes.
    std::set<int> unvisited;    //Set of unvisited nodes.

    //All nodes initialized as unvisited.
    unvisited = subset(src, degs);

    //Distance initialized as 0 for src, and infinity for the rest.
    std::unordered_map<int, int> distance;
    std::unordered_map<int, int> predecessor;

    for(int i : unvisited)
    {   
        distance[i] = INT_MAX;
        predecessor[i] = src;
    }
    distance[src] = 0;
    predecessor[src] = -1;

    //Start from src vertex
    unvisited.erase(src);
    visited.insert(src);
    for(std::pair<int, int> p : mapGraph[src])
    {
        distance[p.first] = p.second;
    }

    //Individual Timer Start
    //auto preAdd = std::chrono::high_resolution_clock::now();

    //While there are nodes left unvisited:
    int counter = 0;
    while(!unvisited.empty())
    {
        counter++;  
        //For all unvisited nodes, find the smallest distance
        int curr = *(unvisited.begin());
        for(int i : unvisited)
        {
            //Check distance values
            if(distance[i] < distance[curr])
                curr = i;          //Update distance if there exists a smaller distance
        }

        //Mark the unvisited node as visited
        unvisited.erase(curr);
        visited.insert(curr);

        //For all adjacent nodes
        auto adj = mapGraph[curr];
        for(std::pair<int, int> p : adj)
        {
            if(unvisited.count(p.first) != 0)
            {
                //Check distance
                int currDistance = distance[curr] + p.second;

                //Perform relaxation if necessary
                if(distance[p.first] > currDistance)
                {
                    distance[p.first] = currDistance;
                    predecessor[p.first] = curr;
                }
            }
        }
        //Individual Timer End
        /*
        if(counter%1 == 0)
        {
            auto postAdd = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(postAdd - preAdd);
            std::cout << "Adding 1 vertices: " << duration.count() << "ms" << std::endl;
            preAdd = std::chrono::high_resolution_clock::now();
        }
        */
    }

    //Timer End
    /*
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "Time to run: " << duration.count() << "ms" << std::endl;
    */
    return make_pair(distance, predecessor);
}

/***
 * Purpose: Performs Breadth First Search to generates a set of all vertices to find all vertices 
 *          that are x degrees/edges away from the source vertex.
 * Parameters:  int src - Source vertex
 *              int degs - Number of degrees/edges an included vertex can be from the source
 * Return:      The generated subset of vertices.  
 ***/
std::set<int> Graph::subset(int src, int degs)
{
    //Timer Start
    //auto start = std::chrono::high_resolution_clock::now();

    std::set<int> set;
    std::queue<int> q;

    //start from src
    q.push(src);
    set.insert(src);

    int currLvlCount = 1;   //Tracks how many vertices are in the current level of BFS
    int nextLvlCount = 0;   //Tracks how many vertices will be considered in the next level of BFS
    int currLvl = 0;        //Tracks how many degrees away from the source we're currently at
  
    //Individual Timer Start
    //auto preAdd = std::chrono::high_resolution_clock::now();

    std::set<int> tempSet;
    while(!q.empty())
    {
        //get+pop the front of the queue.
        int curr = q.front();
        std::vector<std::pair<int,int>> neighbors = mapGraph[curr];

        q.pop();
        currLvlCount--;

        //push all unique adj. vertices into the queue
        for(int i = 0; i < neighbors.size(); i++)
        {
            int vertex = neighbors[i].first;
            //If we haven't reached this vertex yet, add it to the queue and set
            if(!set.count(vertex))  //&& !tempSet.count(vertex)  {Do we need this?}
            {
                q.push(vertex);
                tempSet.insert(vertex);
                nextLvlCount++;
            }
        }


        //If we've traversed all vertices on this level, move to the next level
        if(currLvlCount == 0)
        {
            currLvlCount = nextLvlCount;
            nextLvlCount = 0;
            currLvl++;

            //If we're still within the acceptable range away from src, 
            //merge the gathered vertices into the overall subset
            if(currLvl <= degs)
            {
                std::set<int> uniSet;
                set_union(set.begin(), set.end(), tempSet.begin(), tempSet.end(), std::inserter(uniSet, uniSet.begin()));
                set.swap(uniSet);
                tempSet.clear();
            }
            else
            {
                break;   
            }

            //Individual Timer End
            /*
            auto postAdd = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(postAdd - preAdd);
            std::cout << "Adding Level " << currLvl << ": " << duration.count() << "ms" << std::endl;
            preAdd = std::chrono::high_resolution_clock::now();
            */
        }

    }

    //Timer End
    /*
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "Time to run subset: " << duration.count() << "ms" << std::endl;
    */
    return set;
}