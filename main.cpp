#include <iostream>
#include <vector>
#include "Graph.h"
#include "readFiles.h"

using namespace std;


int main() {
    readFiles rf;
    //Graph g = rf.edgesGraphs("shipping.csv");
    Graph gextra = rf.edgesGraphs("../ProjectData/Extra_Fully_Connected_Graphs/edges_25.csv");
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
    // tsp - currVert = findVertex(0), visitedCount = 0, currDist = 0, minDist = numericLimeitsmax double, criar dois vetores de VERTEX*

    /*for (auto elem : gextra.getVertexSet()) {
        cout << elem->getId() << " ";
    }*/

    //inicio das funcoes 4.3

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

    auto cycles = mst.findEulerianCycle(mst.findVertex(0));


    for (auto v: cycles) {
        cout << v->getId() << " ";
    }
    cout << endl;
    cout << endl;

    cout << mst.hamiltonPath(cycles) << endl;

    //fim do 4.3

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

        if (stringResposta == "1") {
            while (stringResposta != "e") {
                cout << "============================ MENU ================================" << endl;
                cout << "Testar com os Toy-Graphs?  - [PRESS 1]" << endl;
                cout << "Voltar para tras - [PRESS e] " << endl;
                cout << "==================================================================" << endl;
                cin >> stringResposta;
                if (stringResposta == "e") break;
                cin.ignore();
                cout << endl;
                if (stringResposta == "1") {

                    vector<Vertex*> path;

                    while (stringResposta != "e") {
                        Graph gShipping = rf.edgesGraphs("../ProjectData/Toy-Graphs/shipping.csv");
                        cout << "O caminho do Graph Shipping é: ";
                        double custoShipping = gShipping.tspBT(0, path);
                        cout << endl << "e o seu custo é: " << custoShipping << endl << endl;

                        Graph gStadiums = rf.edgesGraphs("../ProjectData/Toy-Graphs/stadiums.csv");
                        cout << "O caminho do Graph Stadiums é: ";
                        double custoStadiums = gStadiums.tspBT(0, path);
                        cout << endl << "e o seu custo é: " << custoStadiums << endl << endl;

                        Graph gTourism = rf.edgesGraphs("../ProjectData/Toy-Graphs/tourism.csv");
                        cout << "O caminho do Graph Tourism é: ";
                        double custoTourism = gTourism.tspBT(0, path);
                        cout << endl << "e o seu custo é: " << custoTourism << endl << endl;

                        cout << "Voltar para tras - [PRESS e] " << endl;
                        cout << "==================================================================" << endl;
                        cin >> stringResposta;
                        if (stringResposta == "e") break;
                        cin.ignore();
                    }
                }
            }
        }

        if (stringResposta == "2") {
            while (stringResposta != "e") {
                cout << "============================ MENU ================================" << endl;
                cout << "Testar com os Toy-Graphs?  - [PRESS 1]" << endl;
                cout << "Testar com os Extra Fully Connected Graphs?  - [PRESS 2]" << endl;
                cout << "Testar com os Real Word Graphs?  - [PRESS 3]" << endl;
                cout << "Voltar para tras - [PRESS e] " << endl;
                cout << "==================================================================" << endl;
                cin >> stringResposta;
                if (stringResposta == "e") break;
                cin.ignore();
                cout << endl;

                if (stringResposta == "1") {

                    while (stringResposta != "e") {

                        Graph gShipping = rf.edgesGraphs("../ProjectData/Toy-Graphs/shipping.csv");
                        gShipping.Prim();
                        auto arrShipping = gShipping.preOrderTraversal(gShipping.findVertex(0));
                        cout << "O caminho do Graph Shipping é: ";
                        for (auto v: arrShipping){
                            cout << v->getId() << " ";
                        }
                        cout << endl;
                        cout << "E o seu custo é: " << gShipping.calculatePathDistance(arrShipping) << endl;
                        cout << endl;


                        Graph gStadiums = rf.edgesGraphs("../ProjectData/Toy-Graphs/stadiums.csv");
                        gStadiums.Prim();
                        auto arrStadiums = gStadiums.preOrderTraversal(gStadiums.findVertex(0));
                        cout << "O caminho do Graph Stadiums é: ";
                        for (auto v: arrStadiums){
                            cout << v->getId() << " ";
                        }
                        cout << endl;
                        cout << "E o seu custo é: " << gStadiums.calculatePathDistance(arrStadiums) << endl;
                        cout << endl;


                        Graph gTourism = rf.edgesGraphs("../ProjectData/Toy-Graphs/tourism.csv");
                        gTourism.Prim();
                        auto arrTourism = gTourism.preOrderTraversal(gTourism.findVertex(0));
                        cout << "O caminho do Graph Tourism é: ";
                        for (auto v: arrTourism){
                            cout << v->getId() << " ";
                        }
                        cout << endl;
                        cout << "E o seu custo é: " << gTourism.calculatePathDistance(arrTourism) << endl;
                        cout << endl;

                        cout << "==================================================================" << endl;
                        cin >> stringResposta;
                        if (stringResposta == "e") break;
                        cin.ignore();
                    }
                }
                if (stringResposta == "2") {
                    while (stringResposta != "e") {
                        Graph g25 = rf.edgesGraphs("../ProjectData/Extra_Fully_Connected_Graphs/edges_25.csv");
                        g25.Prim();
                        auto arr25 = g25.preOrderTraversal(g25.findVertex(0));
                        cout << "O caminho do Graph de 25 edges é: ";
                        for (auto v: arr25){
                            cout << v->getId() << " ";
                        }
                        cout << endl;
                        cout << "E o seu custo é: " << g25.calculatePathDistance(arr25) << endl;
                        cout << endl;


                        Graph g50 = rf.edgesGraphs("../ProjectData/Extra_Fully_Connected_Graphs/edges_50.csv");
                        g50.Prim();
                        auto arr50 = g50.preOrderTraversal(g50.findVertex(0));
                        cout << "O caminho do Graph de 50 edges é: ";
                        for (auto v: arr50){
                            cout << v->getId() << " ";
                        }
                        cout << endl;
                        cout << "E o seu custo é: " << g50.calculatePathDistance(arr50) << endl;
                        cout << endl;


                        Graph g75 = rf.edgesGraphs("../ProjectData/Extra_Fully_Connected_Graphs/edges_75.csv");
                        g75.Prim();
                        auto arr75 = g75.preOrderTraversal(g75.findVertex(0));
                        cout << "O caminho do Graph de 75 edges é: ";
                        for (auto v: arr75){
                            cout << v->getId() << " ";
                        }
                        cout << endl;
                        cout << "E o seu custo é: " << g75.calculatePathDistance(arr75) << endl;
                        cout << endl;

                        Graph g100 = rf.edgesGraphs("../ProjectData/Extra_Fully_Connected_Graphs/edges_100.csv");
                        g100.Prim();
                        auto arr100 = g100.preOrderTraversal(g100.findVertex(0));
                        cout << "O caminho do Graph de 100 edges é: ";
                        for (auto v: arr100){
                            cout << v->getId() << " ";
                        }
                        cout << endl;
                        cout << "E o seu custo é: " << g100.calculatePathDistance(arr100) << endl;
                        cout << endl;


                        Graph g200 = rf.edgesGraphs("../ProjectData/Extra_Fully_Connected_Graphs/edges_200.csv");
                        g200.Prim();
                        auto arr200 = g200.preOrderTraversal(g200.findVertex(0));
                        cout << "O caminho do Graph de 200 edges é: ";
                        for (auto v: arr200){
                            cout << v->getId() << " ";
                        }
                        cout << endl;
                        cout << "E o seu custo é: " << g200.calculatePathDistance(arr200) << endl;
                        cout << endl;


                        Graph g300 = rf.edgesGraphs("../ProjectData/Extra_Fully_Connected_Graphs/edges_300.csv");
                        g300.Prim();
                        auto arr300 = g300.preOrderTraversal(g300.findVertex(0));
                        cout << "O caminho do Graph de 300 edges é: ";
                        for (auto v: arr300){
                            cout << v->getId() << " ";
                        }
                        cout << endl;
                        cout << "E o seu custo é: " << g300.calculatePathDistance(arr300) << endl;
                        cout << endl;


                        Graph g400 = rf.edgesGraphs("../ProjectData/Extra_Fully_Connected_Graphs/edges_400.csv");
                        g400.Prim();
                        auto arr400 = g400.preOrderTraversal(g400.findVertex(0));
                        cout << "O caminho do Graph de 400 edges é: ";
                        for (auto v: arr400){
                            cout << v->getId() << " ";
                        }
                        cout << endl;
                        cout << "E o seu custo é: " << g400.calculatePathDistance(arr400) << endl;
                        cout << endl;


                        Graph g500 = rf.edgesGraphs("../ProjectData/Extra_Fully_Connected_Graphs/edges_500.csv");
                        g500.Prim();
                        auto arr500 = g500.preOrderTraversal(g500.findVertex(0));
                        cout << "O caminho do Graph de 500 edges é: ";
                        for (auto v: arr500){
                            cout << v->getId() << " ";
                        }
                        cout << endl;
                        cout << "E o seu custo é: " << g500.calculatePathDistance(arr500) << endl;
                        cout << endl;


                        Graph g600 = rf.edgesGraphs("../ProjectData/Extra_Fully_Connected_Graphs/edges_600.csv");
                        g600.Prim();
                        auto arr600 = g600.preOrderTraversal(g600.findVertex(0));
                        cout << "O caminho do Graph de 600 edges é: ";
                        for (auto v: arr600){
                            cout << v->getId() << " ";
                        }
                        cout << endl;
                        cout << "E o seu custo é: " << g600.calculatePathDistance(arr600) << endl;
                        cout << endl;


                        Graph g700 = rf.edgesGraphs("../ProjectData/Extra_Fully_Connected_Graphs/edges_700.csv");
                        g700.Prim();
                        auto arr700 = g700.preOrderTraversal(g700.findVertex(0));
                        cout << "O caminho do Graph de 700 edges é: ";
                        for (auto v: arr700){
                            cout << v->getId() << " ";
                        }
                        cout << endl;
                        cout << "E o seu custo é: " << g700.calculatePathDistance(arr700) << endl;
                        cout << endl;


                        Graph g800 = rf.edgesGraphs("../ProjectData/Extra_Fully_Connected_Graphs/edges_800.csv");
                        g800.Prim();
                        auto arr800 = g800.preOrderTraversal(g800.findVertex(0));
                        cout << "O caminho do Graph de 800 edges é: ";
                        for (auto v: arr800){
                            cout << v->getId() << " ";
                        }
                        cout << endl;
                        cout << "E o seu custo é: " << g800.calculatePathDistance(arr800) << endl;
                        cout << endl;


                        Graph g900 = rf.edgesGraphs("../ProjectData/Extra_Fully_Connected_Graphs/edges_900.csv");
                        g900.Prim();
                        auto arr900 = g900.preOrderTraversal(g900.findVertex(0));
                        cout << "O caminho do Graph de 900 edges é: ";
                        for (auto v: arr900){
                            cout << v->getId() << " ";
                        }
                        cout << endl;
                        cout << "E o seu custo é: " << g900.calculatePathDistance(arr900) << endl;
                        cout << endl;


                        cout << "Voltar para tras - [PRESS e] " << endl;
                        cout << "==================================================================" << endl;
                        cin >> stringResposta;
                        if (stringResposta == "e") break;
                        cin.ignore();
                    }
                }
                if (stringResposta == "3") {
                    while (stringResposta != "e") {

                        Graph gWorld1 = rf.realWorldGraphs("../ProjectData/Real-world Graphs/graph1/nodes.csv","../ProjectData/Real-world Graphs/graph1/edges.csv");
                        gWorld1.Prim();
                        auto arrWorld1 = gWorld1.preOrderTraversal(gWorld1.findVertex(0));
                        cout << "O caminho do Real-World Graph 1 é: ";
                        for (auto v: arrWorld1){
                            cout << v->getId() << " ";
                        }
                        cout << endl;
                        cout << "E o seu custo é: " << gWorld1.calculatePathDistance(arrWorld1) << endl;
                        cout << endl;


                        Graph gWorld2 = rf.realWorldGraphs("../ProjectData/Real-world Graphs/graph2/nodes.csv","../ProjectData/Real-world Graphs/graph2/edges.csv");
                        gWorld2.Prim();
                        auto arrWorld2 = gWorld2.preOrderTraversal(gWorld2.findVertex(0));
                        cout << "O caminho do Real-World Graph 2 é: ";
                        for (auto v: arrWorld2){
                            cout << v->getId() << " ";
                        }
                        cout << endl;
                        cout << "E o seu custo é: " << gWorld2.calculatePathDistance(arrWorld2) << endl;
                        cout << endl;


                        Graph gWorld3 = rf.realWorldGraphs("../ProjectData/Real-world Graphs/graph3/nodes.csv","../ProjectData/Real-world Graphs/graph3/edges.csv");
                        gWorld3.Prim();
                        auto arrWorld3 = gWorld3.preOrderTraversal(gWorld3.findVertex(0));
                        cout << "O caminho do Real-World Graph 3 é: ";
                        for (auto v: arrWorld3){
                            cout << v->getId() << " ";
                        }
                        cout << endl;
                        cout << "E o seu custo é: " << gWorld3.calculatePathDistance(arrWorld3) << endl;
                        cout << endl;

                        cout << "Voltar para tras - [PRESS e] " << endl;
                        cout << "==================================================================" << endl;
                        cin >> stringResposta;
                        if (stringResposta == "e") break;
                        cin.ignore();
                    }
                }
            }
        }

        if (stringResposta == "3") {
            while (stringResposta != "e") {
                cout << "============================ MENU ================================" << endl;
                cout << "Testar com os Toy-Graphs?  - [PRESS 1]" << endl;
                cout << "Testar com os Extra Fully Connected Graphs?  - [PRESS 2]" << endl;
                cout << "Voltar para tras - [PRESS e] " << endl;
                cout << "==================================================================" << endl;
                cin >> stringResposta;
                if (stringResposta == "e") break;
                cin.ignore();
                cout << endl;

                if (stringResposta == "1") {
                    while (stringResposta != "e") {
                        cout << "O custo para o Graph Shipping é: ";
                        // Toy-Graph 1
                        cout << endl;
                        cout << "O custo para o Graph Stadiums é: ";
                        // Toy-Graph 2
                        cout << endl;
                        cout << "O custo para o Graph Tourism é: ";
                        // Toy-Graph 3
                        cout << endl;
                        cout << "Voltar para tras - [PRESS e] " << endl;
                        cout << "==================================================================" << endl;
                        cin >> stringResposta;
                        if (stringResposta == "e") break;
                        cin.ignore();
                    }
                }
                if (stringResposta == "2") {
                    while (stringResposta != "e") {
                        cout << "O custo para o Extra Fully Connected de 25 edges é: ";
                        // Extra Fully Connected de 25 edges
                        cout << endl;
                        cout << "O custo para o Extra Fully Connected de 50 edges é: ";
                        // Extra Fully Connected de 50 edges
                        cout << endl;
                        cout << "O custo para o Extra Fully Connected de 75 edges é: ";
                        // Extra Fully Connected de 75 edges
                        cout << endl;
                        cout << "O custo para o Extra Fully Connected de 100 edges é: ";
                        // Extra Fully Connected de 100 edges
                        cout << endl;
                        cout << "O custo para o Extra Fully Connected de 200 edges é: ";
                        // Extra Fully Connected de 200 edges
                        cout << endl;
                        cout << "O custo para o Extra Fully Connected de 300 edges é: ";
                        // Extra Fully Connected de 300 edges
                        cout << endl;
                        cout << "O custo para o Extra Fully Connected de 400 edges é: ";
                        // Extra Fully Connected de 400 edges
                        cout << endl;
                        cout << "O custo para o Extra Fully Connected de 500 edges é: ";
                        // Extra Fully Connected de 500 edges
                        cout << endl;
                        cout << "O custo para o Extra Fully Connected de 600 edges é: ";
                        // Extra Fully Connected de 600 edges
                        cout << endl;
                        cout << "O custo para o Extra Fully Connected de 700 edges é: ";
                        // Extra Fully Connected de 700 edges
                        cout << endl;
                        cout << "O custo para o Extra Fully Connected de 800 edges é: ";
                        // Extra Fully Connected de 800 edges
                        cout << endl;
                        cout << "O custo para o Extra Fully Connected de 900 edges é: ";
                        // Extra Fully Connected de 900 edges
                        cout << endl;
                        cout << "Voltar para tras - [PRESS e] " << endl;
                        cout << "==================================================================" << endl;
                        cin >> stringResposta;
                        if (stringResposta == "e") break;
                        cin.ignore();
                    }
                }
            }
        }
    }

    return 0;
}
