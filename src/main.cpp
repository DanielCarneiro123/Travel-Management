#include <iostream>
#include <vector>
#include "Graph.h"
using namespace std;

int main() {
    Graph g;
    vector<vector<int>> graph;
    if (!g.readEdges(graph)){
        return 0;
    }

}
