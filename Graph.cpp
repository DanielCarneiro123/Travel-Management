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

vector<Vertex*> Graph::OddVertex() {
    vector<Vertex*> result;
    for (auto v: vertexSet){
        if (v->getAdj().size() % 2 == 1){
            cout << v->getId() << endl;
            result.push_back(v);
        }
    }
    return result;
}

void Graph::MinimumPerfectMatching() {
    vector<Vertex*> oddVertices = OddVertex(); // Obtém os vértices ímpares da MST

    while (!oddVertices.empty()) {
        Vertex* u = oddVertices.back(); // Escolha um vértice ímpar arbitrário
        oddVertices.pop_back();

        double minWeight = INF;
        Vertex* minVertex = nullptr;

        // Encontre a aresta de menor peso conectada ao vértice u
        for (auto& edge : u->getAdj()) {
            Vertex* v = edge->getDest();

            if (v->isVisited()) {
                continue; // Ignore arestas já visitadas
            }

            if (edge->getWeight() < minWeight) {
                minWeight = edge->getWeight();
                minVertex = v;
            }
        }

        if (minVertex != nullptr) {
            // Adicione a aresta de menor peso ao emparelhamento perfeito mínimo
            u->addMSTEdge(minVertex, minWeight);
            cout << u->getMST().back()->getOrig()->getId() << " - ";
            cout << u->getMST().back()->getWeight() << " - ";
            cout << u->getMST().back()->getDest()->getId() << endl;
            // Marque os vértices como visitados
            u->setVisited(true);
            minVertex->setVisited(true);

            // Remova os vértices do vetor de vértices ímpares
            oddVertices.erase(std::remove(oddVertices.begin(), oddVertices.end(), u), oddVertices.end());
            oddVertices.erase(std::remove(oddVertices.begin(), oddVertices.end(), minVertex), oddVertices.end());
        }
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

Graph Graph::createSubgraph(const vector<Vertex*> &oddV) {
    Graph subgraph;

    for (auto v : oddV) {
        subgraph.addVertexV2(v->getId(), v->getLongitude(), v->getLatitude());
    }

    for (auto v : oddV) {
        for (auto& edge : v->getAdj()) {
            subgraph.addEdge(edge->getOrig()->getId(), edge->getDest()->getId(), edge->getWeight());
        }
    }

    return subgraph;
}

void Graph::uniteGraphs(Graph &graph){
    for (auto v : graph.getVertexSet()) {
        for (auto& edge : v->getMST()) {
            addBidirectionalEdge(edge->getOrig()->getId(), edge->getDest()->getId(), edge->getWeight());
        }
    }
}

/*std::vector<Vertex*> Graph::findEulerianCircuit() {
    std::vector<Vertex*> cycle;

    Vertex* startVertex = vertexSet[0];

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
        edgeStack.pop()

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
}*/

std::vector<Vertex*> Graph::convertToHamiltonianPath() {
    std::vector<Vertex*> hamiltonianPath;
    std::unordered_set<Vertex*> visitedVertices;

    Vertex* startVertex = findVertex(0);
    hamiltonianPath.push_back(startVertex);
    visitedVertices.insert(startVertex);

    while (hamiltonianPath.size() < vertexSet.size()) {
        Vertex* currentVertex = hamiltonianPath.back();
        for (auto& edge : currentVertex->getAdj()) {
            Vertex* nextVertex = edge->getDest();
            if (visitedVertices.count(nextVertex) == 0) {  // Verificar se o próximo vértice não está no conjunto de visitados
                hamiltonianPath.push_back(nextVertex);
                visitedVertices.insert(nextVertex);  // Adicionar o próximo vértice aos visitados
                break;
            }
        }
    }

    return hamiltonianPath;
}

void Graph::DFS(Vertex* v, vector<Edge*>& visitedEdges, vector<vector<Vertex*>>& cycles) {
    for (auto& edge : v->getAdj()) {
        if (!edge->isSelected()) {
            edge->setSelected(true);
            visitedEdges.push_back(edge);

            Vertex* nextVertex = edge->getDest();
            DFS(nextVertex, visitedEdges, cycles);
        }
    }

    if (!visitedEdges.empty()) {
        Edge* lastEdge = visitedEdges.back();
        visitedEdges.pop_back();

        Vertex* lastVertex = lastEdge->getOrig();
        if (lastVertex->getAdj().size() > 0) {
            DFS(lastVertex, visitedEdges, cycles);
        }
    } else if (!v->getAdj().empty()) {
        // Form a cycle
        vector<Vertex*> cycle;
        for (auto& edge : visitedEdges) {
            cycle.push_back(edge->getOrig());
        }
        cycle.push_back(visitedEdges.back()->getDest()); // Adiciona o destino da última aresta
        cycles.push_back(cycle);
    }
}

vector<vector<Vertex*>> Graph::findEulerianCycles() {
    vector<vector<Vertex*>> cycles;
    vector<Edge*> visitedEdges;

    // Find vertices with odd degree
    vector<Vertex*> oddVertices;
    for (auto v : vertexSet) {
        if (v->getAdj().size() % 2 == 1) {
            oddVertices.push_back(v);
        }
    }

    // Perform DFS from odd degree vertices
    for (auto* v : oddVertices) {
        DFS(v, visitedEdges, cycles);
    }

    return cycles;
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
