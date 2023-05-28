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
    Graph g = rf.edgesGraphs("shipping.csv");
    Graph gextra = rf.extraFullyGraphs("edges_25.csv");
    //funcoes para o 4.2
    /*gextra.Prim();
    vector<Vertex*> mst;
    auto arr = gextra.preOrderTraversal(gextra.findVertex(0), mst);
    cout << gextra.calculatePathDistance(arr) << endl;
    for (auto v: arr){
        cout << v->getId() << " ";
    } */
    //fim das funcoes para o 4.2



    return 0;
    //g.printPath();
}
