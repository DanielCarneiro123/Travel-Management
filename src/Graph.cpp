//
// Created by Daniel on 04/05/2023.
//

#include <fstream>
#include <mmcobj.h>
#include <sstream>
#include "Graph.h"

using namespace std;

vector<int> bestPath;
double minDist = numeric_limits<double>::max();


void deleteMatrix(int **m, int n) {
    if (m != nullptr) {
        for (int i = 0; i < n; i++)
            if (m[i] != nullptr)
                delete [] m[i];
        delete [] m;
    }
}

void deleteMatrix(double **m, int n) {
    if (m != nullptr) {
        for (int i = 0; i < n; i++)
            if (m[i] != nullptr)
                delete [] m[i];
        delete [] m;
    }
}



/*
bool Graph::readEdges(vector<vector<int>>& paths) {
    string filename;
    cout << "File: ";
    cin >> filename;

    fstream file;
    file.open("../input/" + filename, ios::in);
    if (!file) {
        cerr << "Error: file " << filename << " not found" << endl;
        return false;
    }
    if (file.is_open()) {
        string line;
        int node;
        while (!file.eof()) {
            vector<int> path;
            getline(file, line);
            if (line.empty())
                break;
            stringstream l(line);
            try {
                while (l >> node) {
                    path.push_back(node);
                }
            } catch (const exception& e) {
                cerr << "Invalid input" << endl;
            }
            paths.push_back(path);
        }
        file.close();
    }

    return true;
}*/



void Graph::tsp(vector<int> &currPath, double currDist, int currInd, vector<vector<double>> &graph){
    if (currPath.size() == graph[0].size()){
        currDist += graph[currInd][0];
        if (currDist < minDist) {
            minDist = currDist;
            bestPath = currPath;
        }
    }
    for (int i = 0; i < graph[0].size(); i++) {
        if ((!isVisited(i, currPath)) /*&& (currDist + graph[currPath[currInd]][i] < minDist)*/) {
            currPath.push_back(i);
            tsp(currPath, currDist + graph[currInd][i], i, graph);
            currPath.pop_back();
        }
    }
}


void Graph::printPath(){
    cout << minDist << endl;
    for (int i = 0; i < bestPath.size(); i++) {
        cout << bestPath[i] << " ";
    }
}

bool Graph::isVisited(int currInd, vector<int>& path) {
    for (int i = 0; i < path.size(); i++) {
        if (path[i] == currInd) {
            return true;
        }
    }
    return false;
}


