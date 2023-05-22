//
// Created by Daniel on 04/05/2023.
//

#include <fstream>
#include <mmcobj.h>
#include "Graph.h"
#include "VertexEdge.h"
#include <cmath>


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

void Graph::initialize(const string &filename) {

    fstream file;
    file.open("../Toy-Graphs/" + filename, ios::in);
    if (!file)
    {
        cerr << "Error: file " << filename << " not found" << endl;
        return;
    }
    if (file.is_open())
    {
        string numNodes, InitialNode, DestinyNode, capacity, duration;
        getline(file, numNodes, ' ');
        this->addVertex(stoi(numNodes)+1);
        file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        while (!file.eof())
        {
            getline(file, InitialNode, ' ');
            getline(file, DestinyNode, ' ');
            getline(file, capacity, ' ');
            getline(file, duration);
            try {
                this->addEdge(stoi(InitialNode), stoi(DestinyNode), stoi(capacity));
            } catch (const exception &e) {
                return;
            }
        }
        file.close();
    }
}

/*double Graph::haversine(double lat1, double lat2, double lon1, double lon2){
    double dLat = (lat2 - lat1) * M_PI / 180.0;
    double dLon = (lon2 - lon1) * M_PI / 180.0;
    lat1 = (lat1) * M_PI / 180.0;
    lat2 = (lat2) * M_PI / 180.0;
    double a = pow(sin(dLat / 2), 2) + pow(sin(dLon / 2), 2) * cos(lat1) * cos(lat2);
    double rad = 6371;
    double c = 2 * asin(sqrt(a));
    return rad * c;
}*/
/*
void Graph::tsp(vector<int> &currPath, double currDist, int currInd){
    Vertex* initialVertex = findVertex(currInd);
    if (currPath.size() == initialVertex->getAdj().size()){
        int w = initialVertex->getAdj()[0]->getWeight();
        currDist += w;
        if (currDist < minDist) {
            minDist = currDist;
            bestPath = currPath;
        }
    }
    for (int i = 0; i < initialVertex->getAdj().size(); i++) {
        int vert = initialVertex->getAdj()[i]->getOrig()->getId();
        if ((!isVisited(vert, currPath)) && (currDist + (findVertex(currPath[currInd]))->getAdj()[i]->getWeight() < minDist)) {
            currPath.push_back(i);
            tsp(currPath, currDist + (findVertex(currPath[currInd]))->getAdj()[i]->getWeight(), i);
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
}*/

// Função para calcular o custo total de um caminho
double Graph::PathCost(const std::vector<int>& path) {
    double cost = 0.0;
    for (int i = 0; i < path.size() - 1; ++i) {
        auto vertex = findVertex(path[i]);
        auto nextVertex = findVertex(path[i + 1]);
        auto edge = vertex->getEdgeTo(nextVertex);
        cost += edge->getWeight();
    }
    return cost;
}

// Função auxiliar para encontrar todas as permutações possíveis dos vértices
void Graph::permute(std::vector<int>& path, int start, double& minCost, std::vector<int>& bestPath) {
    if (start == path.size() - 1) {
        double cost = PathCost(path);
        if (cost < minCost) {
            minCost = cost;
            bestPath = path;
        }
        return;
    }

    for (int i = start; i < path.size(); ++i) {
        swap(path[start], path[i]);
        permute(path, start + 1, minCost, bestPath);
        swap(path[start], path[i]);
    }
}

// Função principal para resolver o problema do caixeiro-viajante
std::vector<int> Graph::solveTSP() {
    std::vector<int> path;
    for (auto& vertex : getVertexSet()) {
        path.push_back(vertex->getId());
    }

    double minCost = std::numeric_limits<double>::max();
    std::vector<int> bestPath;

    permute(path, 0, minCost, bestPath);

    bestPath.push_back(bestPath[0]);
    return bestPath;
}


