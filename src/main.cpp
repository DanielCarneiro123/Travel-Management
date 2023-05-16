#include <iostream>
#include <vector>
#include "Graph.h"
#include <sstream>
#include <fstream>
using namespace std;

void matrixForm(vector<vector<double>> &graph)  {
    const int maxNodes = 5;
    graph.resize(maxNodes, vector<double>(maxNodes, NULL));
    string currentLine;
    ifstream file;
    file.open("C:/Users/Daniel/CLionProjects/Proj2_DA/Project2Graphs/Toy-Graphs/tourism.csv");
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
}

int main() {
    Graph g;
    vector<vector<double>> graph;
    matrixForm(graph);
    vector<int> path;
    path.push_back(0);
    //vector<vector<double>> memo(graph.size(), vector<double>(graph.size(), -1));
    g.tsp(path, 0, 0, graph);
    g.printPath();
}
