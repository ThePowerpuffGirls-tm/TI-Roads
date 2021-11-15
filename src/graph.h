#include <unordered_map>
#include <vector>

class Graph
{

    std::unordered_map<int, std::vector<int>> g;

    public:
    Graph();
    ~Graph();

};