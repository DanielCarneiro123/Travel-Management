//
// Created by Daniel on 04/05/2023.
//

#include <fstream>
#include "Graph.h"
#include "VertexEdge.h"
#include <cmath>
#include <unordered_set>
#include <stack>
#include <list>


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
/*bool Graph::addVertex(const int &id) {
    if (findVertex(id) != nullptr)
        return false;
    vertexSet.push_back(new Vertex(id));
    return true;
}*/

bool Graph::addVertexV2(const int &id, double longitude, double latitude) {
    if (findVertex(id) != nullptr)
        return false;
    vertexSet.push_back(new Vertex(id, longitude, latitude));
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

bool Graph::addBidirectionalEdge2(Vertex* &v1, Vertex* &v2, double w) {

    if (v1 == nullptr || v2 == nullptr){
        return false;}

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


double Graph::haversine(Vertex* v1, Vertex* v2){
    double lat1 = v1->getLatitude();
    double lon1 = v1->getLongitude();
    double lat2 = v2->getLatitude();
    double lon2 = v1->getLongitude();
    double dLat = (lat2 - lat1) * M_PI / 180.0;
    double dLon = (lon2 - lon1) * M_PI / 180.0;
    lat1 = (lat1) * M_PI / 180.0;
    lat2 = (lat2) * M_PI / 180.0;
    double a = pow(sin(dLat / 2), 2) + pow(sin(dLon / 2), 2) * cos(lat1) * cos(lat2);
    double rad = 6371;
    double c = 2 * asin(sqrt(a));
    return rad * c;
}

bool Graph::hasCon(Vertex* v1, Vertex* v2){
    bool connection = false;

    for (auto edge: v1->getAdj()){
        if (edge->getDest() == v2){
            connection = true;
            break;
        }
    }
    return connection;
}




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

std::vector<Vertex*> Graph::preOrderTraversal(Vertex* vertex) {
    for(auto *v : vertexSet){
        v->setVisited(false);
        v->setPath(nullptr);
    }
    std::vector<Vertex *> res;
    dfs(vertex,res);
    return res;
}

vector<Vertex*> Graph::OddVertex(Graph mst) {
    vector<Vertex*> result;
    for (auto v: mst.getVertexSet()){
        Vertex* v1 = findVertex(v->getId());
        if (v->getAdj().size() % 2 != 0){
            result.push_back(v1);
        }
    }
    return result;
}

bool Graph::IsOdd(Vertex *v, vector<Vertex*>oddVertices) {
    for (auto vert : oddVertices) {
        if (vert == v) return true;
    }
    return false;
}


vector<Edge*> Graph::MinimumPerfectMatching(vector<Vertex*>oddVert) {
    vector<Vertex*>oddVertices = oddVert;
    vector<Edge*>finalEdges;

    while (!oddVertices.empty()) {
        Vertex* u = oddVertices.back(); // Escolha um vértice ímpar arbitrário
        oddVertices.pop_back();

        double minWeight = INF;
        Vertex* minVertex = nullptr;
        // Encontre a aresta de menor peso conectada ao vértice u

        for (auto& edge : u->getAdj()) {
            Vertex* v = edge->getDest();


            if (v->isVisited() || !IsOdd(v, oddVertices)) {
                continue; // Ignore arestas já visitadas
            }

            if (edge->getWeight() < minWeight) {
                minWeight = edge->getWeight();
                minVertex = v;
            }
        }
        if (minVertex != nullptr) {
            // Adicione a aresta de menor peso ao emparelhamento perfeito mínimo

            auto newEdge = new Edge(u, minVertex, minWeight);

            finalEdges.push_back(newEdge);
            u->addMSTEdge(minVertex, minWeight);

            cout << u->getMST().back()->getOrig()->getId() << " - ";
            cout << u->getMST().back()->getWeight() << " - ";
            cout << u->getMST().back()->getDest()->getId() << endl;

            // Marque os vértices como visitados
            u->setVisited(true);
            minVertex->setVisited(true);

            oddVertices.erase(std::remove(oddVertices.begin(), oddVertices.end(), minVertex), oddVertices.end());
        }
    }
    return finalEdges;
}

void Graph::uniteGraphs(vector<Edge*>finalEdges){
    for (auto &e : finalEdges) {
        Vertex* orig = e->getOrig();
        Vertex* dest = e->getDest();
        addBidirectionalEdge2(orig, dest, e->getWeight());
    }
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

double Graph::calculatePathDistance(const std::vector<Vertex*>& path) { //testar soma dos paths
    double distance = 0.0;
    for (int i = 0; i < path.size() ; i++) {
        Vertex* currentVertex = path[i];
        if (currentVertex->getPath() != nullptr){
            distance += currentVertex->getPath()->getWeight();
        }
        else{
            Vertex* nextVertex = path[(i + 1) % path.size()];
            distance += getDistance(currentVertex, nextVertex);
        }
    }
    return distance;
}

double Graph::getDistance(Vertex* v1, Vertex* v2){

    for (auto edge: v1->getAdj()){
        if (edge->getDest() == v2){
            return edge->getWeight();
        }
    }
    return haversine(v1,v2);
}

Graph Graph::createMST() {
    Graph subgraph;

    // Adicione os vértices da MST original ao subgrafo
    for (auto &v : vertexSet) {
        subgraph.addVertexV2(v->getId(), v->getLongitude(), v->getLatitude());
    }

    // Adicione as arestas da MST original ao subgrafo
    for (auto &v : vertexSet) {
        for (auto& edge : v->getMST()) {
            subgraph.addBidirectionalEdge(edge->getOrig()->getId(), edge->getDest()->getId(), edge->getWeight());
        }
    }

    return subgraph;
}


vector<Vertex*> Graph::findEulerianCycle(Vertex* startVertex) {
    vector<Vertex *> res;

    for (Vertex *v: vertexSet) {
        for(Edge* e : v->getAdj()) {
            e->setSelected(false);
            e->getReverse()->setSelected(false);
        }
    }

    stack<Vertex *> stack;
    stack.push(startVertex);

    while (!stack.empty()) {
        Vertex *v = stack.top();

        vector<Edge *> unvisitedEdges;
        for (Edge *e: v->getAdj()) {
            if (!e->isSelected()) {
                unvisitedEdges.push_back(e);
            }
        }
        if (!unvisitedEdges.empty()) {
            Edge *e = unvisitedEdges.front();
            e->setSelected(true);
            e->getReverse()->setSelected(true);
            stack.push(e->getDest());
        }else{
            res.push_back(v);
            stack.pop();
        }
    }

    return res;
}

std::vector<Vertex*> Graph::findEulerianCircuit() {
    std::vector<Vertex*> cycle;

    Vertex* startVertex = findVertex(0);

    std::stack<Edge*> edgeStack;
    edgeStack.push(startVertex->getAdj()[0]);

    while (!edgeStack.empty()) {
        Edge* currentEdge = edgeStack.top();

        if (currentEdge->isSelected()) {

            edgeStack.pop();
            continue;
        }
        currentEdge->setSelected(true);
        Vertex* currentVertex = currentEdge->getDest();

        cycle.push_back(currentVertex);
        edgeStack.pop();

        bool foundUnvisitedEdge = false;

        for (Edge* edge : currentVertex->getAdj()) {
            if (!edge->isSelected()) {
                edgeStack.push(edge);
                foundUnvisitedEdge = true;
                break;
            }
        }
        if (!foundUnvisitedEdge && currentVertex != startVertex) {

            cycle.pop_back();
            edgeStack.push(currentEdge);
        }

    }

    return cycle;
}

double Graph::hamiltonPath(vector<Vertex*> eurlerian){
    double result = 0.0;
    for (auto e: findVertex(0)->getAdj()){
        if (e->getDest() == eurlerian[0]){
            result += e->getWeight();
        }
    }
    cout << "0" << " ";
    cout << eurlerian[0]->getId() << " ";
    for (int i = 0; i < eurlerian.size() - 1; i++){
        if (eurlerian[i+1]->isVisited()){
            continue;
        }
        else {
            for (auto e: eurlerian[i]->getAdj()){
                if (e->getDest() == eurlerian[i+1]){
                    result += e->getWeight();
                }
            }
            eurlerian[i+1]->setVisited(true);
            cout << eurlerian[i+1]->getId() << " ";
        }

    }
    return result;
}

