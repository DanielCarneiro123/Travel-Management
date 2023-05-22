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

    myFile.open("C:\\Users\\danie\\OneDrive\\Documentos\\GitHub\\DAproj2\\ProjectData\\Toy-Graphs\\stadiums.csv");
    getline(myFile, currentLine);    // ignore first line
    while (getline(myFile, currentLine)) {
        stringstream iss(currentLine);
        getline(iss, InitialNode, ',');
        getline(iss, DestinyNode, ',');
        getline(iss, distance, ',');
        if((set_nodes.insert(InitialNode).second)){
            _graph.addVertex(stoi(InitialNode));
        }
        if((set_nodes.insert(DestinyNode).second)){
            _graph.addVertex(stoi(DestinyNode));
        }
        if ((set_network.insert(std::make_pair(InitialNode, DestinyNode)).second)) {
            _graph.addBidirectionalEdge(stoi(InitialNode), stoi(DestinyNode), stod(distance));
        }

    }
    myFile.close();
    return _graph;
}