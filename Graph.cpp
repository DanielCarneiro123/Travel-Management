//
// Created by Daniel on 04/05/2023.
//

#include <fstream>
#include "Graph.h"
#include "VertexEdge.h"
#include <cmath>
#include <unordered_set>
#include <stack>


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

/*void Graph::initialize(const string &filename) {

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
}*/

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



/*double Graph::connectingVertex(int v1_id, int v2_id){
    auto v1 = findVertex(v1_id);
    auto v2 = findVertex(v2_id);

    if (!hasCon(v1_id, v2_id)) {
        double distance = haversine(v1,v2);
        return distance;
    }

    else{
        for (auto edge :v1->getAdj()){
            if(edge->getDest() == v2){
                return edge->getWeight();
            }
        }
    }
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

std::vector<Vertex*> Graph::preOrderTraversal(Vertex* vertex) {
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

Graph Graph::createSubgraph() {
    Graph subgraph;

    // Adicione os vértices da MST original ao subgrafo
    for (auto v : vertexSet) {
        subgraph.addVertexV2(v->getId(), v->getLongitude(), v->getLatitude());
    }

    // Adicione as arestas da MST original ao subgrafo
    for (auto v : vertexSet) {
        for (auto& edge : v->getMST()) {
            subgraph.addBidirectionalEdge(edge->getOrig()->getId(), edge->getDest()->getId(), edge->getWeight());
        }
    }

    return subgraph;
}

std::vector<Vertex*> Graph::findEulerianCircuit() {
    std::vector<Vertex*> circuit;

    Vertex* startVertex = nullptr;
    for (auto v : vertexSet) {
        if (v->getIndegree() > 0) {
            startVertex = v;
            break;
        }
    }


    if (startVertex == nullptr) {
        return circuit;
    }


    std::stack<Vertex*> vertexStack;
    vertexStack.push(startVertex);

    while (!vertexStack.empty()) {
        Vertex* currentVertex = vertexStack.top();
        if (currentVertex->getAdj().empty()) {
            circuit.push_back(currentVertex);
            vertexStack.pop();
        } else {
            Edge* edge = currentVertex->getAdj().front();
            edge->setSelected(true);
            vertexStack.push(edge->getDest());
            currentVertex->getIncoming().push_back(edge);
        }
    }

    return circuit;
}

std::vector<Vertex*> Graph::convertToHamiltonianPath() { //mal implementada
    
    std::vector<Vertex*> hamiltonianPath;


    Vertex* startVertex = findVertex(0);
    hamiltonianPath.push_back(startVertex);


    while (hamiltonianPath.size() < vertexSet.size()) {
        Vertex* currentVertex = hamiltonianPath.back();
        for (auto& edge : currentVertex->getAdj()) {
            Vertex* nextVertex = edge->getDest();
            if (std::find(hamiltonianPath.begin(), hamiltonianPath.end(), nextVertex) == hamiltonianPath.end()) {
                hamiltonianPath.push_back(nextVertex);
                break;
            }
        }
    }

    return hamiltonianPath;
}



/*
Vertex* findClosestVertex(Vertex* v, const vector<Vertex*>& vertices) {
    Vertex* closest = nullptr;
    double minDistance = INF;

    for (auto vertex : vertices) {
        if (v != vertex) {
            for (auto e: vertex->getAdj()) {
                double distance = calculateDistance(v, e->getDest());
                if (distance < minDistance) {
                    minDistance = distance;
                    closest = vertex;
                }
            }
        }
    }
    return closest;
}

vector<Vertex*> Graph::getEulerianTour(vector<Vertex*> mst) {
    vector<Vertex*> tour;

    stack<Vertex*> verticesStack;
    vector<Vertex*> currentTour;

    verticesStack.push(mst[0]);

    while (!verticesStack.empty()) {
        Vertex* currentVertex = verticesStack.top();
        bool foundUnvisitedEdge = false;

        for (Edge* edge : currentVertex->getAdj()) {
            if (!edge->getVisit()) {
                edge->setVisit(true);
                verticesStack.push(edge->getDest());
                currentTour.push_back(currentVertex);
                foundUnvisitedEdge = true;
                break;
            }
        }

        if (!foundUnvisitedEdge) {

            tour.push_back(currentVertex);
            verticesStack.pop();
        }
    }


    reverse(currentTour.begin(), currentTour.end());
    tour.insert(tour.end(), currentTour.begin(), currentTour.end());

    return tour;
}

std::vector<Vertex*> findMinimumCostPerfectMatching(const std::vector<Vertex*>& oddVertices) {
    int numOddVertices = oddVertices.size();
    std::vector<Vertex*> matching(numOddVertices, nullptr);
    std::vector<bool> visited(numOddVertices, false);
    std::vector<double> slack(numOddVertices, std::numeric_limits<double>::max());

    for (int i = 0; i < numOddVertices; i++) {
        double minWeight = std::numeric_limits<double>::max();
        int minIndex = -1;

        for (int j = 0; j < numOddVertices; j++) {
            if (!visited[j]) {
                double weight = 0;
                //double weight = oddVertices[i]->getWeightTo(oddVertices[j]);
                for(auto *e : oddVertices[i]->getAdj()){
                    if(e->getDest() == oddVertices[j]){
                        weight = e->getWeight();
                    }
                }
                if (weight < slack[j]) {
                    slack[j] = weight;
                }

                if (slack[j] < minWeight) {
                    minWeight = slack[j];
                    minIndex = j;
                }
            }
        }

        for (int j = 0; j < numOddVertices; j++) {

            double weight3 = 0;
            //double weight = oddVertices[j]->getWeightTo(oddVertices[minIndex]);
            for(auto *e : oddVertices[j]->getAdj()){
                if(e->getDest() == oddVertices[minIndex]){
                    weight3 = e->getWeight();
                }
            }

            if (visited[j]) {
                oddVertices[j]->setDist(minWeight);
            } else if (weight3 == minWeight) {
                visited[j] = true;
            }
        }
    }

    for (int i = 0; i < numOddVertices; i++) {
        if (matching[i] == nullptr) {
            double weight2 = 0;
            for (int j = 0; j < numOddVertices; j++) {

                for(auto *e : oddVertices[i]->getAdj()){
                    if(e->getDest() == oddVertices[j]){
                        weight2 = e->getWeight();
                    }
                }

                if (matching[j] == nullptr && weight2 == 0) {
                    matching[i] = oddVertices[j];
                    matching[j] = oddVertices[i];
                    break;
                }
            }
        }
    }

    return matching;
}
/*
//Função base

void Graph::tspCombined() {
    //dividir o grapho em subpaths usando divide and conquer
    vector<vector<Vertex*>> subPaths = divideGraph(vertexSet);

    vector<Vertex*> finalPath;

    for (const vector<Vertex*>& subPath : subPaths) {
        //dividir em clusters
        vector<vector<Vertex*>> clusters = clusteringFunction(subPath);

        vector<Vertex*> clusterPath;
        //fazer 2step para cada cluster
        for (const vector<Vertex*>& cluster : clusters) {
            vector<Vertex *> clusterPath = tspTwoStepApproximation(cluster);
            finalPath.insert(finalPath.end(), clusterPath.begin(), clusterPath.end());
        }
    }
    for (int i = 0; i<finalPath.size()-1;i++) {
        cout << finalPath[i]->getId() << " ";
    }
}

vector<vector<Vertex*>> Graph::divideGraph(const vector<Vertex*> vertexes) {
    vector<vector<Vertex*>> subPaths;
    const size_t subPathSize = vertexes.size() / 2;

    for (int i = 0; i< vertexes.size(); i+=subPathSize) {
        vector<Vertex*> subPath;

        for (int j = 1; j < i + subPathSize; j++) {
            subPath.push_back(vertexes[j]);
        }

        subPaths.push_back(subPath);
    }

    return subPaths;
}

vector<Vertex*> Graph::tspTwoStepApproximation(const vector<Vertex*>& cluster) {
    // Step 1: Find the vertex with the minimum degree in the cluster
    Vertex* minDegreeVertex = nullptr;
    int minDegree = INT_MAX;

    for (Vertex* vertex : cluster) {
        int degree = vertex->getAdj().size();
        if (degree < minDegree) {
            minDegree = degree;
            minDegreeVertex = vertex;
        }
    }

    // Step 2: Perform a Preorder Traversal starting from the vertex with the minimum degree
    vector<Vertex*> clusterPath;
    clusterPath = preOrderTraversal2(minDegreeVertex, cluster);

    return clusterPath;
}

std::vector<Vertex*> Graph::preOrderTraversal2(Vertex* vertex, vector<Vertex*> cluster) {
    for(auto *v : cluster){
        v->setVisited(false);
        v->setPath(nullptr);
    }
    std::vector<Vertex *> res;
    dfs(vertex,res);
    return res;
}

vector<vector<Vertex*>> Graph::clusteringFunction(const vector<Vertex*>& vertices) {
    vector<vector<Vertex*>> clusters;

    // Definir o número de clusters com base no tamanho do grafo
    int numClusters = ceil(sqrt(vertices.size()));  // Número de clusters proporcional à raiz quadrada do tamanho do grafo

    // Implementar a lógica para agrupar os vértices em clusters
    // com base nas características do grafo e nos novos dados

    // Exemplo: Dividir os vértices em clusters aproximadamente iguais
    int verticesPerCluster = vertices.size() / numClusters;

    int currentIndex = 0;
    for (int i = 0; i < numClusters; i++) {
        vector<Vertex*> cluster;
        for (int j = 0; j < verticesPerCluster; j++) {
            cluster.push_back(vertices[currentIndex]);
            currentIndex++;
        }
        clusters.push_back(cluster);
    }

    return clusters;
}*/

