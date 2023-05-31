//
// Created by Gonçalo Costa on 08/04/2023.
//

#include "readFiles.h"
#include "Graph.h"
#include <sstream>
#include <fstream>
#include <set>
#include "unordered_set"
#include "string"

using namespace std;

Graph readFiles::edgesGraphs(const string &filename){
    string InitialNode, DestinyNode, distance;
    Graph _graph;
    ifstream myFile;
    string currentLine;
    unordered_set<string> set_nodes;
    set<pair<string,string>> set_network;


    myFile.open(filename);

    // Verificar se o nome do arquivo contém "Toy-Graphs" para eliminar a primeira linha
    bool ignoreFirstLine = (filename.find("Toy-Graphs") != string::npos);

    if (ignoreFirstLine) {
        getline(myFile, currentLine);    // ignore first line
    }


    while (getline(myFile, currentLine)) {
        std::stringstream iss(currentLine);
        getline(iss, InitialNode, ',');
        getline(iss, DestinyNode, ',');
        getline(iss, distance, ',');

        int initialNodeId = std::stoi(InitialNode);
        int destinyNodeId = std::stoi(DestinyNode);
        double distanceDoble = std::stod(distance);

        if (set_nodes.insert(InitialNode).second) {
            _graph.addVertexV2(initialNodeId);
        }
        if (set_nodes.insert(DestinyNode).second) {
            _graph.addVertexV2(destinyNodeId);
        }
        if (set_network.insert(std::make_pair(InitialNode, DestinyNode)).second) {
            Vertex* v1 = _graph.findVertex(initialNodeId);
            Vertex* v2 = _graph.findVertex(destinyNodeId);
            _graph.addBidirectionalEdge2(v1, v2, distanceDoble);
        }
    }
    myFile.close();
    return _graph;
}

Graph readFiles::realWorldGraphs(const std::string &filename, const std::string &filename2) {
    std::string Node, longitude, latitude, InitialNode, DestinyNode, distance;
    Graph _graph;
    std::ifstream myFile;
    std::string currentLine;


    myFile.open(filename);
    getline(myFile, currentLine); // ignore first line
    while (getline(myFile, currentLine)) {
        std::stringstream iss(currentLine);
        getline(iss, Node, ',');
        getline(iss, longitude, ',');
        getline(iss, latitude, ',');

        int NodeId = std::stoi(Node);
        double dlongitude = std::stod(longitude);
        double dlatitude = std::stod(latitude);


        _graph.addVertexV2(NodeId, dlongitude, dlatitude);
    }
    myFile.close();

    vector<Vertex*>vertexSet = _graph.getVertexSet();
    myFile.open(filename2);
    getline(myFile, currentLine);
    while (getline(myFile, currentLine)) {
        std::stringstream iss(currentLine);
        getline(iss, InitialNode, ',');
        getline(iss, DestinyNode, ',');
        getline(iss, distance, ',');

        int initialNodeId = std::stoi(InitialNode);
        int destinyNodeId = std::stoi(DestinyNode);
        double distanceDoble = std::stod(distance);

        Vertex* v1 = vertexSet[initialNodeId];
        Vertex* v2 = vertexSet[destinyNodeId];

        _graph.addBidirectionalEdge2(v1, v2, distanceDoble);

    }
    myFile.close();


    return _graph;
}