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

Graph readFiles::smallGraphs(const std::string &filename){
    string InitialNode, DestinyNode, distance;
    Graph _graph;
    ifstream myFile;
    string currentLine;
    unordered_set<string> set_nodes;
    set<pair<string,string>> set_network;

    myFile.open("C:\\Users\\Daniel\\OneDrive\\Documentos\\GitHub\\DAproj2\\ProjectData\\Extra_Fully_Connected_Graphs\\edges_25.csv");
    getline(myFile, currentLine);    // ignore first line
    while (getline(myFile, currentLine)) {
        std::stringstream iss(currentLine);
        getline(iss, InitialNode, ',');
        getline(iss, DestinyNode, ',');
        getline(iss, distance, ',');

        int initialNodeId = std::stoi(InitialNode);
        int destinyNodeId = std::stoi(DestinyNode);
        double distanceDoble = std::stod(distance);

        if (set_nodes.insert(InitialNode).second) {
            _graph.addVertex(initialNodeId);
        }
        if (set_nodes.insert(DestinyNode).second) {
            _graph.addVertex(destinyNodeId);
        }
        if (set_network.insert(std::make_pair(InitialNode, DestinyNode)).second) {
            _graph.addBidirectionalEdge(initialNodeId, destinyNodeId, distanceDoble);
        }
    }
    myFile.close();
    return _graph;
}

Graph readFiles::extraFullyGraphs(const std::string &filename){
    string InitialNode, DestinyNode, distance;
    Graph _graph;
    ifstream myFile;
    string currentLine;
    unordered_set<string> set_nodes;
    set<pair<string,string>> set_network;

    myFile.open("C:\\Users\\Daniel\\OneDrive\\Documentos\\GitHub\\DAproj2\\ProjectData\\Extra_Fully_Connected_Graphs\\edges_75.csv");
    while (getline(myFile, currentLine)) {
        std::stringstream iss(currentLine);
        getline(iss, InitialNode, ',');
        getline(iss, DestinyNode, ',');
        getline(iss, distance, ',');

        int initialNodeId = std::stoi(InitialNode);
        int destinyNodeId = std::stoi(DestinyNode);
        double distanceDoble = std::stod(distance);

        if (set_nodes.insert(InitialNode).second) {
            _graph.addVertex(initialNodeId);
        }
        if (set_nodes.insert(DestinyNode).second) {
            _graph.addVertex(destinyNodeId);
        }
        if (set_network.insert(std::make_pair(InitialNode, DestinyNode)).second) {
            _graph.addBidirectionalEdge(initialNodeId, destinyNodeId, distanceDoble);
        }
    }
    myFile.close();
    return _graph;
}