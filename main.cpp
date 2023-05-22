#include <iostream>
#include <vector>
#include "Graph.h"
#include <sstream>
#include <fstream>
#include "readFiles.h"
using namespace std;

/*void matrixForm(vector<vector<double>> &graph)  {
    const int maxNodes = 11;
    graph.resize(maxNodes, vector<double>(maxNodes, NULL));
    string currentLine;
    ifstream file;
    file.open("C:/Users/Daniel/CLionProjects/Proj2_DA/Project2Graphs/Toy-Graphs/stadiums.csv");
    getline(file, currentLine);
    while (getline(file, currentLine))
    {
        istringstream iss(currentLine);
        string InitialNode, DestinyNode, capacity;
        //file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

        getline(iss, InitialNode, ',');
        getline(iss, DestinyNode, ',');
        getline(iss, capacity, ',');
        try {
            int initialId = stoi(InitialNode);
            int destinyId = stoi(DestinyNode);
            double distance = stod(capacity);
            graph[initialId][destinyId] = distance;
            graph[destinyId][initialId] = distance;
        } catch (const exception &e) {
            return;
        }
    }
    file.close();
}*/

int main() {
    readFiles rf;
    Graph g = rf.smallGraphs("tourism.csv");
    std::vector<int> tspPath = g.solveTSP();

    std::cout << "Caminho: ";
    for (const auto& vertexId : tspPath) {
        std::cout << vertexId << " ";
    }
    std::cout << std::endl;

    double tspCost = g.PathCost(tspPath);
    std::cout << "Custo: " << tspCost << std::endl;

    //vector<vector<double>> graph;
    //matrixForm(graph);
    //vector<int> path;
    //path.push_back(0);
    //vector<vector<double>> memo(graph.size(), vector<double>(graph.size(), -1));
    //g.tsp(path, 0, 0);
    //g.printPath();
    return 0;
}
