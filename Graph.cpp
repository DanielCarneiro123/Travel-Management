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
#include <chrono>


using namespace std;
using namespace std::chrono;



/**
 * @brief Retorna o número de vértices do grafo.
 *
 * @return O número de vértices do grafo.
 *
 * Complexidade: O(1)
 */
int Graph::getNumVertex() const {
    return vertexSet.size();
}

/**
 * @brief Retorna um vetor com os ponteiros para os vértices do grafo.
 *
 * @return Um vetor com os ponteiros para os vértices do grafo.
 *
 * Complexidade: O(1)
 */
std::vector<Vertex *> Graph::getVertexSet() const {
    return vertexSet;
}

/**
 * @brief Procura um vértice com o ID especificado no grafo.
 *
 * @param id O ID do vértice a ser procurado.
 * @return Um ponteiro para o vértice encontrado ou nullptr se não for encontrado.
 *
 * Complexidade: O(V), onde V é o número de vértices no grafo.
 */
Vertex* Graph::findVertex(const int& id) const {
    for (auto vertex : vertexSet) {
        if (vertex->getId() == id) {
            return vertex;
        }
    }
    return nullptr;
}

/**
 * @brief Procura o índice de um vértice com o ID especificado no grafo.
 *
 * @param id O ID do vértice a ser procurado.
 * @return O índice do vértice encontrado ou -1 se não for encontrado.
 *
 * Complexidade: O(V), onde V é o número de vértices no grafo.
 */
int Graph::findVertexIdx(const int &id) const {
    for (unsigned i = 0; i < vertexSet.size(); i++)
        if (vertexSet[i]->getId() == id)
            return i;
    return -1;
}

/**
 * @brief Adiciona um vértice ao grafo com o ID, longitude e latitude especificados.
 *
 * @param id O ID do vértice a ser adicionado.
 * @param longitude A longitude do vértice a ser adicionado.
 * @param latitude A latitude do vértice a ser adicionado.
 * @return True se o vértice for adicionado com sucesso, False se um vértice com o mesmo ID já existe.
 *
 * Complexidade: O(V), onde V é o número de vértices no grafo.
 */
bool Graph::addVertexV2(const int &id, double longitude, double latitude) {
    if (findVertex(id) != nullptr)
        return false;
    vertexSet.push_back(new Vertex(id, longitude, latitude));
    return true;
}

/**
 * @brief Adiciona uma aresta direcionada ao grafo com o ID de origem, o ID de destino e o peso especificados.
 *
 * @param sourc O ID do vértice de origem da aresta.
 * @param dest O ID do vértice de destino da aresta.
 * @param w O peso da aresta.
 * @return True se a aresta for adicionada com sucesso, False se o vértice de origem ou o vértice de destino não existirem.
 *
 * Complexidade: O(V), onde V é o número de vértices no grafo.
 */
bool Graph::addEdge(const int &sourc, const int &dest, double w) {
    auto v1 = findVertex(sourc);
    auto v2 = findVertex(dest);
    if (v1 == nullptr || v2 == nullptr)
        return false;
    v1->addEdge(v2, w);
    return true;
}

/**
 * @brief Adiciona uma aresta bidirecional ao grafo com o ID de origem, o ID de destino e o peso especificados.
 *
 * @param sourc O ID do vértice de origem da aresta.
 * @param dest O ID do vértice de destino da aresta.
 * @param w O peso da aresta.
 * @return True se a aresta for adicionada com sucesso, False se o vértice de origem ou o vértice de destino não existirem.
 *
 * Complexidade: O(V), onde V é o número de vértices no grafo.
 */
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

/**
 * @brief Adiciona uma aresta bidirecional ao grafo usando ponteiros para os vértices.
 *
 * @param v1 O ponteiro para o vértice de origem da aresta.
 * @param v2 O ponteiro para o vértice de destino da aresta.
 * @param w O peso da aresta.
 * @return True se a aresta for adicionada com sucesso, False se o ponteiro para o vértice de origem ou o ponteiro para o vértice de destino forem nulos.
 *
 * Complexidade: O(1)
 */
bool Graph::addBidirectionalEdge2(Vertex* &v1, Vertex* &v2, double w) {
    if (v1 == nullptr || v2 == nullptr) {
        return false;
    }
    auto e1 = v1->addEdge(v2, w);
    auto e2 = v2->addEdge(v1, w);
    e1->setReverse(e2);
    e2->setReverse(e1);
    return true;
}

/**
 * @brief Calcula a distância em metros entre duas coordenadas geográficas utilizando a fórmula de Haversine.
 *
 * @param lat1 A latitude da primeira coordenada.
 * @param lon1 A longitude da primeira coordenada.
 * @param lat2 A latitude da segunda coordenada.
 * @param lon2 A longitude da segunda coordenada.
 * @return A distância em metros entre as duas coordenadas.
 *
 * Complexidade: O(1)
 */
double Graph::haversine(double lat1, double lon1, double lat2, double lon2) {
    double dLat = (lat2 - lat1) * M_PI / 180.0;
    double dLon = (lon2 - lon1) * M_PI / 180.0;

    // convert to radians
    lat1 = (lat1) * M_PI / 180.0;
    lat2 = (lat2) * M_PI / 180.0;

    // apply formula
    double a = pow(sin(dLat / 2), 2) + pow(sin(dLon / 2), 2) * cos(lat1) * cos(lat2);
    double rad = 6371;
    double c = 2 * asin(sqrt(a));
    return rad * c * 1000;
}

/**
 * @brief Verifica se dois vértices têm uma conexão direta.
 *
 * @param v1 O ponteiro para o primeiro vértice.
 * @param v2 O ponteiro para o segundo vértice.
 * @return True se os vértices tiverem uma conexão direta, False caso contrário.
 *
 * Complexidade: O(E), onde E é o número de arestas no grafo.
 */
bool Graph::hasCon(Vertex* v1, Vertex* v2) {
    bool connection = false;

    for (auto edge : v1->getAdj()) {
        if (edge->getDest() == v2) {
            connection = true;
            break;
        }
    }
    return connection;
}

/**
 * @brief Implementa o algoritmo de busca exaustiva para resolver o problema do caixeiro-viajante (TSP).
 *
 * @param initialNode O ID do vértice inicial.
 * @param path O vetor de ponteiros de vértices que armazenará o caminho mínimo encontrado.
 * @return A distância total percorrida no caminho mínimo.
 *
 * Complexidade: O(n!), onde n é o número de vértices no grafo.
 */
double Graph::tspBT(int initialNode, vector<Vertex*>& path) {
    double minDist = numeric_limits<double>::max();
    vector<Vertex*> minPath;

    Vertex* initialVert = findVertex(initialNode);
    initialVert->setVisited(true);

    tsp(initialVert, 0, 0, minDist, path, minPath);
    cout << "Path: ";
    for (auto v : minPath) {
        cout << v->getId() << " ";
    }
    cout << endl;

    return minDist;
}


/**
 * @brief Implementa o algoritmo de busca exaustiva para resolver o problema do caixeiro-viajante (TSP).
 *
 * @param currVert O ponteiro para o vértice atual.
 * @param visitedCount O número de vértices visitados até o momento.
 * @param currDist A distância percorrida até o momento.
 * @param minDist A menor distância encontrada até o momento.
 * @param path O vetor de ponteiros de vértices que armazena o caminho atual.
 * @param minPath O vetor de ponteiros de vértices que armazena o caminho mínimo encontrado.
 *
 * Complexidade: O(n!), onde n é o número de vértices no grafo.
 */
void Graph::tsp(Vertex* currVert, int visitedCount, double currDist, double &minDist, vector<Vertex*> &path, vector<Vertex*> &minPath) {
    if (visitedCount == vertexSet.size() - 1) {
        bool hasCon = false;
        for (auto e : currVert->getAdj()) {
            if (e->getDest()->getId() == 0) {
                currDist += e->getWeight();
                hasCon = true;
                break;
            }
        }
        if (currDist < minDist && hasCon) {
            minDist = currDist;
            minPath = path;
            minPath.push_back(currVert);
        }
        return;
    }
    for (auto edge : currVert->getAdj()) {
        Vertex* dest = edge->getDest();
        if (!dest->isVisited()) {
            dest->setVisited(true);
            path.push_back(currVert);
            tsp(dest, visitedCount + 1, currDist + edge->getWeight(), minDist, path, minPath);
            path.pop_back();
            dest->setVisited(false);
        }
    }
}

/**
 * @brief Realiza uma travessia em pré-ordem (DFS) a partir de um vértice dado.
 *
 * @param vertex O ponteiro para o vértice inicial da travessia.
 * @return O vetor de ponteiros de vértices na ordem pré-ordem.
 *
 * Complexidade: O(V + E), onde V é o número de vértices e E é o número de arestas no grafo.
 */
std::vector<Vertex*> Graph::preOrderTraversal(Vertex* vertex) {
    std::vector<Vertex*> preOrder;
    dfs(vertex, preOrder);
    return preOrder;
}

/**
 * @brief Retorna os vértices de grau ímpar em uma árvore geradora mínima.
 *
 * @param mst O grafo da árvore geradora mínima.
 * @return O vetor de ponteiros de vértices de grau ímpar.
 *
 * Complexidade: O(V), onde V é o número de vértices no grafo.
 */
vector<Vertex*> Graph::OddVertex(Graph mst) {
    vector<Vertex*> result;
    for (auto v : mst.getVertexSet()) {
        Vertex* v1 = findVertex(v->getId());
        if (v->getAdj().size() % 2 != 0) {
            result.push_back(v1);
        }
    }
    return result;
}

/**
 * @brief Verifica se um vértice está presente em um vetor de vértices ímpares.
 *
 * @param v O ponteiro para o vértice a ser verificado.
 * @param oddVertices O vetor de vértices ímpares.
 * @return true se o vértice está presente no vetor, false caso contrário.
 *
 * Complexidade: O(V), onde V é o número de vértices no grafo.
 */
bool Graph::IsOdd(Vertex* v, vector<Vertex*> oddVertices) {
    for (auto vert : oddVertices) {
        if (vert == v)
            return true;
    }
    return false;
}

/**
 * @brief Encontra o emparelhamento perfeito mínimo em um conjunto de vértices ímpares.
 *
 * @param oddVert O vetor de vértices ímpares.
 * @return O vetor de ponteiros de arestas que formam o emparelhamento perfeito mínimo.
 *
 * Complexidade: O(V^2), onde V é o número de vértices no grafo.
 */
vector<Edge*> Graph::MinimumPerfectMatching(vector<Vertex*> oddVert) {
    vector<Vertex*> oddVertices = oddVert;
    vector<Edge*> finalEdges;

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

            // Marque os vértices como visitados
            u->setVisited(true);
            minVertex->setVisited(true);

            oddVertices.erase(std::remove(oddVertices.begin(), oddVertices.end(), minVertex), oddVertices.end());
        }
    }
    return finalEdges;
}

/**
 * @brief Une os grafos adicionando as arestas bidirecionais especificadas.
 *
 * @param finalEdges O vetor de ponteiros de arestas que serão adicionadas como bidirecionais.
 *
 * Complexidade: O(E), onde E é o número de arestas no vetor.
 */
void Graph::uniteGraphs(vector<Edge*> finalEdges) {
    for (auto& e : finalEdges) {
        addBidirectionalEdge(e->getOrig()->getId(), e->getDest()->getId(), e->getWeight());
    }
}

/**
 * @brief Executa o algoritmo de Prim para encontrar a árvore geradora mínima do grafo.
 *
 * Complexidade: O((V + E) log V), onde V é o número de vértices e E é o número de arestas no grafo.
 */
void Graph::Prim() {
    // Reset auxiliary info
    for (auto v : vertexSet) {
        v->setDist(INF);
        v->setPath(nullptr);
        v->setVisited(false);
    }

    // Start with node 0
    Vertex* s = nullptr;
    for (auto v : vertexSet) {
        if (v->getId() == 0) {
            s = v;
            break;
        }
    }
    s->setDist(0);

    // Initialize priority queue
    MutablePriorityQueue<Vertex> q;
    q.insert(s);

    // Process vertices in the priority queue
    while (!q.empty()) {
        auto v = q.extractMin();
        if (v->getPath() != nullptr) {
            v->getPath()->getOrig()->addMSTEdge(v, v->getDist());
        }
        v->setVisited(true);

        for (auto& edge : v->getAdj()) {
            Vertex* w = edge->getDest();
            if (!w->isVisited()) {
                auto oldDist = w->getDist();
                if (edge->getWeight() < oldDist) {
                    w->setPath(edge);
                    w->setDist(edge->getWeight());
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

/**
 * @brief Realiza uma busca em profundidade (DFS) a partir do vértice atual, adicionando os vértices ao caminho.
 *
 * @param currentVertex O vértice atual da busca.
 * @param path O vetor de vértices que representa o caminho percorrido.
 *
 * Complexidade: O(V + E), onde V é o número de vértices e E é o número de arestas no grafo.
 */
void Graph::dfs(Vertex* currentVertex, std::vector<Vertex*>& path) {
    currentVertex->setVisited(true);
    path.push_back(currentVertex); // Add current vertex to MST

    for (auto& e : currentVertex->getMST()) {
        Vertex* w = e->getDest();
        if (!w->isVisited()) {
            if (currentVertex->getPath() == nullptr) {
                currentVertex->setPath(e);
            }
            dfs(w, path);
        }
    }
}

/**
 * @brief Calcula a distância total percorrida ao longo do caminho especificado.
 *
 * @param path O vetor de vértices que representa o caminho.
 * @return A distância total percorrida ao longo do caminho.
 *
 * Complexidade: O(N), onde N é o número de vértices no caminho.
 */
double Graph::calculatePathDistance(const std::vector<Vertex*>& path) {
    double distance = 0.0;

    for (int i = 0; i < path.size(); i++) {
        Vertex* currentVertex = path[i];
        Vertex* nextVertex = path[(i + 1) % path.size()];
        distance += getDistance(currentVertex, nextVertex);
    }

    return distance;
}

/**
 * @brief Obtém a distância entre dois vértices.
 *
 * @param v1 O primeiro vértice.
 * @param v2 O segundo vértice.
 * @return A distância entre os dois vértices.
 *
 * Complexidade: O(V + E), onde V é o número de vértices e E é o número de arestas no grafo.
 */
double Graph::getDistance(Vertex* v1, Vertex* v2) {
    for (auto v : vertexSet) {
        if (v->getId() == v1->getId()) {
            for (auto edge : v->getAdj()) {
                if (edge->getDest()->getId() == v2->getId()) {
                    return edge->getWeight();
                }
            }
        }
    }

    return haversine(v1->getLatitude(), v1->getLongitude(), v2->getLatitude(), v2->getLongitude());
}

/**
 * @brief Cria uma subárvore geradora mínima (MST) do grafo atual.
 *
 * @return Um objeto Graph que representa a subárvore geradora mínima.
 *
 * Complexidade: O(V + E), onde V é o número de vértices e E é o número de arestas no grafo.
 */
Graph Graph::createMST() {
    Graph subgraph;

    // Adicione os vértices da MST original ao subgrafo
    for (auto& v : vertexSet) {
        subgraph.addVertexV2(v->getId(), v->getLongitude(), v->getLatitude());
    }

    // Adicione as arestas da MST original ao subgrafo
    for (auto& v : vertexSet) {
        for (auto& edge : v->getMST()) {
            subgraph.addBidirectionalEdge(edge->getOrig()->getId(), edge->getDest()->getId(), edge->getWeight());
        }
    }

    return subgraph;
}

/**
 * @brief Encontra um ciclo euleriano no grafo, iniciando a partir do vértice especificado.
 *
 * @param startVertex O vértice de partida para o ciclo euleriano.
 * @return Um vetor de vértices que representa o ciclo euleriano.
 *
 * Complexidade: O(V + E), onde V é o número de vértices e E é o número de arestas no grafo.
 */
vector<Vertex*> Graph::findEulerianCycle(Vertex* startVertex) {
    vector<Vertex*> res;

    for (Vertex* v : vertexSet) {
        for (Edge* e : v->getAdj()) {
            e->setSelected(false);
        }
    }
    Vertex* currVertex = startVertex;
    stack<Vertex*> vertexStack;
    vertexStack.push(currVertex);

    while (!vertexStack.empty()) {
        vector<Edge*> unvisitedEdges;
        for (Edge* e : currVertex->getAdj()) {
            if (!e->isSelected()) {
                unvisitedEdges.push_back(e);
            }
        }
        if (!unvisitedEdges.empty()) {
            Edge* e = unvisitedEdges.front();
            e->setSelected(true);
            vertexStack.push(e->getDest());
        } else {
            res.push_back(currVertex);
            currVertex = vertexStack.top();
            vertexStack.pop();
        }
    }

    return res;
}

/**
 * @brief Encontra um circuito euleriano no grafo a partir do vértice de origem 0.
 *
 * @return Um vetor de vértices que representa o circuito euleriano.
 */
std::vector<Vertex*> Graph::findEulerianCircuit() {
    std::vector<Vertex*> cycle;

    Vertex* startVertex = findVertex(0);
    cycle.push_back(startVertex);
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

/**
 * @brief Calcula o custo do caminho hamiltoniano dado um circuito euleriano.
 *
 * @param eulerian O vetor de vértices que representa o circuito euleriano.
 * @return O custo do caminho hamiltoniano.
 */
double Graph::hamiltonPath(vector<Vertex*> eulerian) {
    double result = 0.0;

    cout << eulerian[0]->getId() << " ";
    eulerian[0]->setVisited(true);
    for (int i = 0; i < eulerian.size() - 1; i++) {
        if (eulerian[i+1]->isVisited()) {
            continue;
        } else {
            for (auto e : eulerian[i]->getAdj()) {
                if (e->getDest() == eulerian[i+1]) {
                    result += e->getWeight();
                }
            }
            eulerian[i+1]->setVisited(true);
            cout << eulerian[i+1]->getId() << " ";
        }
    }

    for (auto e : eulerian[eulerian.size() - 1]->getAdj()) {
        if (e->getDest()->getId() == 0) {
            result += e->getWeight();
        }
    }
    cout << "0" << " ";
    cout << endl << "e o seu custo é: ";
    return result;
}


/**
 * @brief Implementa o algoritmo do vizinho mais próximo para encontrar um caminho hamiltoniano aproximado no grafo.
 */
void Graph::nearestNeighbour() {
    vector<Vertex*> path;  // Caminho percorrido pelo caixeiro
    unordered_set<Vertex*> unvisited;  // Conjunto de vértices não visitados

    // Adicionar todos os vértices do grafo ao conjunto de não visitados
    for (auto vertex : vertexSet) {
        unvisited.insert(vertex);
    }

    // Escolher o primeiro vértice aleatoriamente
    Vertex* current = findVertex(0);
    unvisited.erase(current);
    path.push_back(current);

    // Enquanto ainda houver vértices não visitados
    while (!unvisited.empty()) {
        double minDistance = numeric_limits<double>::max();
        Vertex* next;

        // Encontrar o vértice não visitado mais próximo do vértice atual
        for (auto vertex : unvisited) {
            double distance = getDistance(current, vertex);
            if (distance < minDistance) {
                minDistance = distance;
                next = vertex;
            }
        }

        // Adicionar o vértice mais próximo ao caminho e removê-lo dos não visitados
        unvisited.erase(next);
        path.push_back(next);
        current = next;
    }

    // Adicionar a aresta de retorno ao primeiro vértice
    path.push_back(path[0]);

    // Imprimir o caminho e o peso total
    cout << "Caminho: ";
    double totalWeight = 0.0;
    for (int i = 0; i < path.size() - 1; i++) {
        Vertex* v1 = path[i];
        Vertex* v2 = path[i + 1];
        double weight = getDistance(v1, v2);
        totalWeight += weight;
        cout << v1->getId() << " -> ";
    }
    cout << path.back()->getId() << endl;
    cout << "Peso total: " << totalWeight << endl;
}

/**
 * @brief Remove os vértices repetidos de um circuito euleriano e calcula o peso do caminho hamiltoniano.
 *
 * @param eulerian O vetor de vértices que representa o circuito euleriano.
 * @param gextra O grafo extra contendo as informações das distâncias entre os vértices.
 * @return O peso do caminho hamiltoniano resultante.
 */
double Graph::removeRepeatedVertices(vector<Vertex*> eulerian, Graph &gextra) {
    vector<Vertex*> hamiltonianPath;
    double pathWeight = 0;
    double distance = 0;

    for (auto vertex : eulerian) {
        if (!vertex->isVisited()) {
            vertex->setVisited(true);
            hamiltonianPath.push_back(vertex);
        }
    }

    for (int i = 0; i < hamiltonianPath.size(); i++) {
        Vertex* nextVertex;
        Vertex* currVertex = findVertex(hamiltonianPath[i]->getId());
        if (i == hamiltonianPath.size() - 1) {
            nextVertex = findVertex(0);
        } else {
            nextVertex = findVertex(hamiltonianPath[i + 1]->getId());
        }

        cout << currVertex->getId() << " --(";

        distance += gextra.getDistance(currVertex, nextVertex);

        cout << gextra.getDistance(currVertex, nextVertex) << ")-- ";

        /*
        for (auto edge : currVertex->getAdj()) {
            if (edge->getDest() == nextVertex) {
                pathWeight += edge->getWeight();
                cout << edge->getWeight() << ")-- ";
            }
        }
         */
    }
    cout << endl << " distancia: " << distance << endl;
    return distance;
}


/**
 * @brief Função auxiliar para resolver o problema do caixeiro viajante utilizando busca exaustiva.
 *
 * @param gextra O grafo extra contendo as informações das distâncias entre os vértices.
 */
void Graph::aux4_1(Graph &gextra) {
    auto start = high_resolution_clock::now();

    vector<Vertex*> path;

    double custo = gextra.tspBT(0, path);
    cout << "e o seu custo é: " << custo << endl;

    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);

    cout << "Tempo de execução: " << duration.count() / 1000000.0 << " segundos" << endl << endl;
}

/**
 * @brief Função auxiliar para resolver o problema do caixeiro viajante utilizando o algoritmo de Prim.
 *
 * @param gextra O grafo extra contendo as informações das distâncias entre os vértices.
 */
void Graph::aux4_2(Graph &gextra) {
    auto start = high_resolution_clock::now();

    gextra.Prim();
    for (auto *v : gextra.getVertexSet()) {
        v->setVisited(false);
        v->setPath(nullptr);
    }
    auto arr = gextra.preOrderTraversal(gextra.findVertex(0));
    for (auto v : arr) {
        cout << v->getId() << " ";
    }
    cout << endl;
    cout << "E o seu custo é: " << gextra.calculatePathDistance(arr) << endl;

    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);

    cout << "Tempo de execução: " << duration.count() / 1000000.0 << " segundos" << endl << endl;
}

/**
 * @brief Função final para resolver o problema do caixeiro viajante utilizando algoritmos diversos.
 *
 * @param gextra O grafo extra contendo as informações das distâncias entre os vértices.
 */
void Graph::final4_3(Graph &gextra) {
    auto start = high_resolution_clock::now();

    gextra.Prim();

    auto mst = gextra.createMST();

    vector<Vertex*> oddV = gextra.OddVertex(mst);

    for (auto v : oddV) {
        v->setVisited(false);
    }

    vector<Edge*> oddEdges = mst.MinimumPerfectMatching(oddV);

    mst.uniteGraphs(oddEdges);

    vector<Vertex*> oddVert = gextra.OddVertex(mst);

    auto cycles = mst.findEulerianCycle(mst.findVertex(0));

    cout << mst.removeRepeatedVertices(cycles, gextra) << endl;

    /*
    for (auto v: cycles){
        cout << v->getId() << " ";
    }
    cout << endl;
    */

    //gextra.nearestNeighbour();

    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);

    cout << "Tempo de execução: " << duration.count() / 1000000.0 << " segundos" << endl << endl;
}

