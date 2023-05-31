#include <iostream>
#include <vector>
#include "Graph.h"
#include "readFiles.h"

using namespace std;

void exibirMenu(const string& opcao) {
    if (opcao == "1") {
        cout << "============================ MENU ================================" << endl;
        cout << "Testar com os Toy-Graphs?  - [PRESS 1]" << endl;
        cout << "Voltar para tras - [PRESS e] " << endl;
        cout << "==================================================================" << endl;
    } else if (opcao == "2") {
        cout << "============================ MENU ================================" << endl;
        cout << "Testar com os Toy-Graphs?  - [PRESS 1]" << endl;
        cout << "Testar com os Extra Fully Connected Graphs?  - [PRESS 2]" << endl;
        cout << "Testar com os Real Word Graphs?  - [PRESS 3]" << endl;
        cout << "Voltar para tras - [PRESS e] " << endl;
        cout << "==================================================================" << endl;
    } else if (opcao == "3") {
        cout << "============================ MENU ================================" << endl;
        cout << "Testar com os Toy-Graphs?  - [PRESS 1]" << endl;
        cout << "Testar com os Extra Fully Connected Graphs?  - [PRESS 2]" << endl;
        cout << "Voltar para tras - [PRESS e] " << endl;
        cout << "==================================================================" << endl;
    }
}

void exibirCustos(const string& opcao) {
    if (opcao == "1") {
        cout << "O custo para o Graph Shipping é: " << endl;
        // Toy-Graph 1
        cout << "O custo para o Graph Stadiums é: " << endl;
        // Toy-Graph 2
        cout << "O custo para o Graph Tourism é: " << endl;
        // Toy-Graph 3
    } else if (opcao == "2") {
        cout << "O custo para o Extra Fully Connected de 25 edges é: " << endl;
        // Extra Fully Connected de 25 edges
        cout << "O custo para o Extra Fully Connected de 50 edges é: " << endl;
        // Extra Fully Connected de 50 edges
        cout << "O custo para o Extra Fully Connected de 75 edges é: " << endl;
        // Extra Fully Connected de 75 edges
        cout << "O custo para o Extra Fully Connected de 100 edges é: " << endl;
        // Extra Fully Connected de 100 edges
        cout << "O custo para o Extra Fully Connected de 200 edges é: " << endl;
        // Extra Fully Connected de 200 edges
        cout << "O custo para o Extra Fully Connected de 300 edges é: " << endl;
        // Extra Fully Connected de 300 edges
        cout << "O custo para o Extra Fully Connected de 400 edges é: " << endl;
        // Extra Fully Connected de 400 edges
        cout << "O custo para o Extra Fully Connected de 500 edges é: " << endl;
        // Extra Fully Connected de 500 edges
        cout << "O custo para o Extra Fully Connected de 600 edges é: " << endl;
        // Extra Fully Connected de 600 edges
        cout << "O custo para o Extra Fully Connected de 700 edges é: " << endl;
        // Extra Fully Connected de 700 edges
        cout << "O custo para o Extra Fully Connected de 800 edges é: " << endl;
        // Extra Fully Connected de 800 edges
        cout << "O custo para o Extra Fully Connected de 900 edges é: " << endl;
        // Extra Fully Connected de 900 edges
    } else if (opcao == "3") {
        cout << "O custo para o Real-World Graph 1 é: " << endl;
        // Real-World Graphs 1
        cout << "O custo para o Real-World Graph 2 é: " << endl;
        // Real-World Graphs 2
        cout << "O custo para o Real-World Graph 3 é: " << endl;
        // Real-World Graphs 3
    }
}


int main() {
    readFiles rf;
    //Graph g = rf.edgesGraphs("shipping.csv");
    Graph gextra = rf.edgesGraphs("edges_25.csv");
    //gextra = rf.extraFullyGraphs("edges_25.csv");
    cout << "langonha" << endl;
    //funcoes para o 4.2
    /*gextra.Prim();
    auto arr = gextra.preOrderTraversal(gextra.findVertex(0));
    cout << gextra.calculatePathDistance(arr) << endl;
    for (auto v: arr){
        cout << v->getId() << " ";
    }*/
    //fim das funcoes para o 4.2

    //gextra.tspCombined();


    /*for (auto elem : gextra.getVertexSet()) {
        cout << elem->getId() << " ";
    }*/

    gextra.Prim();
    auto mst = gextra.createMST();
    cout << "First MST:" << endl;
    for (auto v : mst.getVertexSet()) {
        for (auto e : v->getAdj()) {
            cout << e->getOrig()->getId() << " " << e->getDest()->getId() << " " << e->getWeight() << endl;
        }
    }

    //gextra.preOrderTraversal(gextra.findVertex(0));


    vector<Vertex*> oddV = gextra.OddVertex(mst);
    cout << "Odd Vertexes:" << endl;
    for (auto v : oddV) cout << v->getId() << " ";
    cout << endl;
    for (auto v : oddV) {
        v->setVisited(false);
    }

    cout << endl;
    cout << endl;
    vector<Edge*> oddEdges = mst.MinimumPerfectMatching(oddV);
    cout << endl;
    cout << endl;
    mst.uniteGraphs(oddEdges);
    cout << "Second MST:" << endl;
    for (auto v : mst.getVertexSet()) {
        for (auto e : v->getAdj()) {
            cout << e->getOrig()->getId() << " " << e->getDest()->getId() << " " << e->getWeight() << endl;
        }
    }

    cout << "Second Odds:" << endl;
    vector<Vertex*> oddVert = gextra.OddVertex(mst);
    for (auto v : oddVert) cout << v->getId() << " ";
    cout << endl;



    //juntar mst ao getMST dos nós do subgraph


    auto cycles = mst.findEulerianCycles();


    for (auto vect: cycles) {
        for (auto v: vect){
            cout << v->getId() << " ";
        }

    }
    cout << endl;



    /*for (auto v : oddVertices) {
        oddGraph.addVertexV2(v->getId(), v->getLongitude(), v->getLatitude());
    }*/

    //auto arr2 = gextra.findMinimumCostPerfectMatching(arr);

    //as arestas do emparelhamento perfeito minimo a MST
    //ciclo eureliano
    //g.printPath();


    string stringResposta;

    while (stringResposta != "q") {
        cout << "===================================== MENU ========================================" << endl;
        cout << "Testar o ponto 4.1? - [PRESS 1]" << endl;
        cout << "Testar o ponto 4.2? - [PRESS 2]" << endl;
        cout << "Testar o ponto 4.3? - [PRESS 3]" << endl;
        cout << "Para terminar - [PRESS q]" << endl;
        cout << "==================================================================================" << endl;
        cin >> stringResposta;
        cin.ignore();

        if (stringResposta == "q") {
            break;
        }

        if (stringResposta == "1" || stringResposta == "2" || stringResposta == "3") {
            while (stringResposta != "e") {
                exibirMenu(stringResposta);
                cin >> stringResposta;
                cin.ignore();

                if (stringResposta == "e") {
                    break;
                }

                exibirCustos(stringResposta);
                cout << "Voltar para tras - [PRESS e] " << endl;
                cout << "==================================================================" << endl;
                cin >> stringResposta;
                cin.ignore();
            }
        }
    }

    return 0;
}
