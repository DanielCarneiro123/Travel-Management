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
    vector<int> path;
    path.push_back(0); // Começa na cidade 0
    g.tsp(path, 0, INT32_MAX, 0);

    // Exibe o caminho mais curto encontrado
    cout << "Caminho mais curto: ";
    for (int i = 0; i < bestPath.size(); i++) {
        cout << bestPath[i] << " ";
    }
}
