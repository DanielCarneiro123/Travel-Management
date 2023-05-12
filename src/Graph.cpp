//
// Created by Daniel on 04/05/2023.
//

#include <fstream>
#include <mmcobj.h>
#include <sstream>
#include "Graph.h"

using namespace std;

int Graph::getNumVertex() const {
    return vertexSet.size();
}

std::vector<Vertex *> Graph::getVertexSet() const {
    return vertexSet;
}

/*
 * Auxiliary function to find a vertex with a given content.
 */
Vertex * Graph::findVertex(const int &id) const {
    for (auto v : vertexSet)
        if (v->getId() == id)
            return v;
    return nullptr;
}

/*
 * Finds the index of the vertex with a given content.
 */
int Graph::findVertexIdx(const int &id) const {
    for (unsigned i = 0; i < vertexSet.size(); i++)
        if (vertexSet[i]->getId() == id)
            return i;
    return -1;
}
/*
 *  Adds a vertex with a given content or info (in) to a graph (this).
 *  Returns true if successful, and false if a vertex with that content already exists.
 */
bool Graph::addVertex(const int &id) {
    if (findVertex(id) != nullptr)
        return false;
    vertexSet.push_back(new Vertex(id));
    return true;
}

/*
 * Adds an edge to a graph (this), given the contents of the source and
 * destination vertices and the edge weight (w).
 * Returns true if successful, and false if the source or destination vertex does not exist.
 */
bool Graph::addEdge(const int &sourc, const int &dest, double w) {
    auto v1 = findVertex(sourc);
    auto v2 = findVertex(dest);
    if (v1 == nullptr || v2 == nullptr)
        return false;
    v1->addEdge(v2, w);
    return true;
}

bool Graph::addBidirectionalEdge(const int &sourc, const int &dest, double w) {
    auto v1 = findVertex(sourc);
    auto v2 = findVertex(dest);
    if (v1 == nullptr || v2 == nullptr)
        return false;
    auto e1 = v1->addEdge(v2, w);
    auto e2 = v2->addEdge(v1, w);
    e1->setReverse(e2);
    e2->setReverse(e1);
    return true;
}

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

Graph::~Graph() {
    deleteMatrix(distMatrix, vertexSet.size());
    deleteMatrix(pathMatrix, vertexSet.size());
}



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
}

/*void Graph::readVertex(const std::string &filename)  {

    fstream file;
    file.open("../input/" + filename, ios::in);
    if (!file)
    {
        cerr << "Error: file " << filename << " not found" << endl;
        return;
    }
    if (file.is_open())
    {
        string numNodes, InitialNode, DestinyNode, capacity, duration;
        getline(file, numNodes, ' ');
        this->vertexSet = vector<Vertex *>(stoi(numNodes)+1);
        file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        while (!file.eof())
        {
            getline(file, InitialNode, ' ');
            getline(file, DestinyNode, ' ');
            getline(file, capacity, ' ');
            try {
                this->addEdge(stoi(InitialNode), stoi(DestinyNode), stoi(capacity));
            } catch (const exception &e) {
                return;
            }
        }
        file.close();
    }
}*/


vector<vector<int>> graph;
vector<int> bestPath;

void Graph::tsp(vector<int> currPath, int currDist, int minDist, int currInd){
    if (currPath.size() < graph.size()){
        currDist += graph[currPath[currInd - 1]][currPath[0]];
        if (currDist < minDist){
            minDist = currDist;
            bestPath = currPath;
        }
        return;
    }
    for (int i = 0; i < graph.size(); i++) {
        if (!isVisited(i, currPath) && graph[currPath[currInd - 1]][i]) {
            currPath.push_back(i);
            tsp(currPath, currDist + graph[currPath[currInd - 1]][i], minDist, currInd + 1);
        }
    }
}

bool Graph::isVisited(int city, vector<int>& path) {
    for (int i = 0; i < path.size(); i++) {
        if (path[i] == city) {
            return true;
        }
    }
    return false;
}

