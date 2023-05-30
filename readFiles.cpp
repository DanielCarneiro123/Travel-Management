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
/*
Graph readFiles::edgesGraphs(const std::string &filename){
    string InitialNode, DestinyNode, distance;
    Graph _graph;
    ifstream myFile;
    string currentLine;
    unordered_set<string> set_nodes;
    set<pair<string,string>> set_network;

    myFile.open("C:\\Users\\danie\\OneDrive\\Documentos\\GitHub\\DAproj2\\ProjectData\\Extra_Fully_Connected_Graphs\\edges.csv");
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

    myFile.open("C:\\Users\\danie\\OneDrive\\Documentos\\GitHub\\DAproj2\\ProjectData\\Extra_Fully_Connected_Graphs\\edges_900.csv");
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
*/
Graph readFiles::edgesGraphs(const std::string &filename){
    string InitialNode, DestinyNode, distance;
    Graph _graph;
    ifstream myFile;
    string currentLine;
    unordered_set<string> set_nodes;
    set<pair<string,string>> set_network;

    myFile.open("C:\\Users\\danie\\OneDrive\\Documentos\\GitHub\\DAproj2\\ProjectData\\Extra_Fully_Connected_Graphs\\edges_75.csv");
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
            _graph.addVertexV2(initialNodeId);
        }
        if (set_nodes.insert(DestinyNode).second) {
            _graph.addVertexV2(destinyNodeId);
        }
        if (set_network.insert(std::make_pair(InitialNode, DestinyNode)).second) {
            _graph.addBidirectionalEdge(initialNodeId, destinyNodeId, distanceDoble);
        }
    }
    myFile.close();
    return _graph;
}
/*
Graph readFiles::vertexGraphs(const std::string &filename) {
    std::string Node, longitude, latitude, InitialNode, DestinyNode, distance;
    Graph _graph;
    std::ifstream myFile;
    std::string currentLine;


    myFile.open("C:\\Users\\danie\\OneDrive\\Documentos\\GitHub\\DAproj2\\ProjectData\\Real-world Graphs\\graph2\\nodes.csv");
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
    myFile.open("C:\\Users\\danie\\OneDrive\\Documentos\\GitHub\\DAproj2\\ProjectData\\Real-world Graphs\\graph2\\edges.csv");
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
}*/