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
    //Graph g = rf.edgesGraphs("shipping.csv");
    Graph gextra = rf.edgesGraphs("edges_25.csv");
    //gextra = rf.extraFullyGraphs("edges_25.csv");
    cout << "langonha" << endl;
    //funcoes para o 4.2
    /*gextra.Prim();
    auto arr = gextra.preOrderTraversal(gextra.findVertex(0));
    cout << gextra.calculatePathDistance(arr) << endl;
    for (auto v: arr){
        cout << v->getId() << " ";
    }*/
    //fim das funcoes para o 4.2

    //gextra.tspCombined();


    /*for (auto elem : gextra.getVertexSet()) {
        cout << elem->getId() << " ";
    }*/

    gextra.Prim();
    auto mst = gextra.createMST();
    //gextra.preOrderTraversal(gextra.findVertex(0));
    auto oddV = mst.OddVertex();
    for (auto v: oddV){
        cout << v->getId() << endl;
    }
    auto subgraph = mst.createSubgraph(oddV);
    subgraph.MinimumPerfectMatching();


    //juntar mst ao getMST dos nós do subgraph

    mst.uniteGraphs(subgraph);
    auto cycles = mst.findEulerianCycles();


    for (auto vect: cycles) {
        for (auto v: vect){
            cout << v->getId() << " ";
        }

    }
    cout << endl;



    /*for (auto v : oddVertices) {
        oddGraph.addVertexV2(v->getId(), v->getLongitude(), v->getLatitude());
    }*/

    //auto arr2 = gextra.findMinimumCostPerfectMatching(arr);

    //as arestas do emparelhamento perfeito minimo a MST
    //ciclo eureliano
    return 0;
    //g.printPath();
}
