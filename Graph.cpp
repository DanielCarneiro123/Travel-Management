//
// Created by Daniel on 04/05/2023.
//

#include <fstream>
#include "Graph.h"
#include "VertexEdge.h"
#include <cmath>
#include <unordered_set>


using namespace std;

//double minPath;

int Graph::getNumVertex() const {
    return vertexSet.size();
}

std::vector<Vertex *> Graph::getVertexSet() const {
    return vertexSet;
}

/*
 * Auxiliary function to find a vertex with a given content.
 */
Vertex* Graph::findVertex(const int& id) const {
    for (auto vertex : vertexSet) {
        if (vertex->getId() == id) {
            return vertex;
        }
    }
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

/*double Vertex::dfs(Vertex* vertex) {
    cout << vertex << " "; // show node order
    int count = 1;
    vertex->setVisited(true);
    for (auto e : vertex->getAdj()) {
        if (e->getDest()->getPath()) {
                count += dfs(e->getDest());
        }
    }
    return count;
}*/

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

double Graph::tspBT(int initialNode, vector<Vertex*> &path){
    double minDist = numeric_limits<double>::max();
    vector<Vertex*> minPath;

    Vertex* initialVert = findVertex(initialNode);
    initialVert->setVisited(true);

    tsp(initialVert, 0, 0, minDist ,path, minPath);
    cout<< "Path: ";
    for (auto v : minPath) {
        cout << v->getId() << " ";
    }
    cout << endl;

    return minDist;
}

void Graph::tsp(Vertex* currVert, int visitedCount, double currDist , double &minDist, vector<Vertex*> &path, vector<Vertex*> &minPath) {
    if (visitedCount == vertexSet.size() - 1){
        for (auto e : currVert->getAdj()) {
            if (e->getDest()->getId() == 0) {
                currDist += e->getWeight();
                break;
            }
        }
        if (currDist < minDist) {
            minDist = currDist;
            minPath = path;
            minPath.push_back(currVert);
            //cout << vert->getPath() << endl;
        }
        return;
    }
    for (auto edge: currVert->getAdj()) {
        Vertex* dest = edge->getDest();
        double edgeDist = edge->getWeight();
        if (!dest->isVisited()) {
            dest->setVisited(true);
            path.push_back(currVert);
            tsp(dest, visitedCount + 1, currDist + edge->getWeight(), minDist, path, minPath);
            path.pop_back();
            dest->setVisited(false);
        }
    }
}

std::vector<Vertex*> Graph::preOrderTraversal(Vertex* vertex, std::vector<Vertex*>& mst) {
    for(auto *v : vertexSet){
        v->setVisited(false);
        v->setPath(nullptr);
    }
    std::vector<Vertex *> res;
    dfs(vertex,res);
    return res;
}

void Graph::Prim() {

    // Reset auxiliary info
    for (auto v : vertexSet) {
        v->setDist(INF);
        v->setPath(nullptr);
        v->setVisited(false);
    }

    // Start with node 0
    Vertex* s ;
    for (auto v: vertexSet){
        if (v->getId() == 0){
            s = v;
        }
    }
    s->setDist(0);

    // Initialize priority queue
    MutablePriorityQueue<Vertex> q;
    q.insert(s);
    // Process vertices in the priority queue
    while (!q.empty()) {
        auto v = q.extractMin();
        v->setVisited(true);
        if (v->getPath() != nullptr){
            v->getPath()->getOrig()->addMSTEdge(v, v->getDist());
        }
        for (auto& edge : v->getAdj()) { // Loop variable changed to 'edge'
            Vertex* w = edge->getDest(); // Access the destination vertex using 'edge'
            if (!w->isVisited()) {
                auto oldDist = w->getDist();
                if (edge->getWeight() < oldDist) {
                    w->setDist(edge->getWeight());
                    w->setPath(edge);
                    if (oldDist == INF) {
                        q.insert(w);
                    } else {
                        q.decreaseKey(w);
                    }
                }
            }
        }
    }
}

void Graph::dfs(Vertex *currentVertex, std::vector<Vertex*> &path) {
    currentVertex->setVisited(true);
    path.push_back(currentVertex); // Add current vertex to MST

    for (auto& e : currentVertex->getMST()) {
        Vertex *w = e->getDest();
        if (!w->isVisited()) {
            if (currentVertex->getPath() == nullptr) {
                currentVertex->setPath(e);
            }
            dfs(w, path);
        }
    }
}

double Graph::calculatePathDistance(const std::vector<Vertex*>& path) {
    double distance = 0.0;
    for (int i = 0; i < path.size() ; i++) {
        Vertex* currentVertex = path[i];
        if (currentVertex->getPath() != nullptr){
            distance += currentVertex->getPath()->getWeight();
        }
        else{
            Vertex* nextVertex = path[(i + 1) % path.size()];
            bool entered = false;
            for (auto *e: currentVertex->getAdj()){
                if (e->getDest() == nextVertex){
                    distance += e->getWeight();
                    entered = true;
                    break;
                }
            }
            if (entered == false) {
                for (auto e: currentVertex->getAdj()){
                    if (e->getDest()->getId() == 0){
                        distance += e->getWeight();
                    }
                }
            }
        }
    }
    return distance;
}








/*
// Função para calcular o custo total de um caminho
double Graph::PathCost(const std::vector<int>& path) {
    double cost = 0.0;
    for (int i = 0; i < path.size() - 1; ++i) {
        auto vertex = findVertex(path[i]);
        auto nextVertex = findVertex(path[i + 1]);
        for (auto v: vertex->getAdj()){
            if (v->getDest() == nextVertex){
                auto edge = v;
                cost += edge->getWeight();
                break;
            }
        }
    }

    auto vertex1 = findVertex(path.back());
    auto vertex2 = findVertex(0);
    for (auto v: vertex1->getAdj()){
        if (v->getDest() == vertex2){
            auto edge = v;
            cost += edge->getWeight();
            break;
        }
    }

return cost;
}


// Função auxiliar para encontrar todas as permutações possíveis dos vértices
void Graph::permute(std::vector<int>& path, int start, double& minCost, std::vector<int>& bestPath) {
    if (start == path.size()) {
        double cost = PathCost(path);
        if (cost < minCost) {
            minCost = cost;
            bestPath = path;
        }
        return;
    }

    for (int i = start; i < path.size(); ++i) {
        std::swap(path[start], path[i]);
        permute(path, start + 1, minCost, bestPath);
        std::swap(path[start], path[i]);
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

    permute(path, 1, minCost, bestPath);

    bestPath.push_back(bestPath[0]);
    return bestPath;
}
*/