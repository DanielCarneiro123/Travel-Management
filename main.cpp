#include <iostream>
#include <vector>
#include "Graph.h"
#include "readFiles.h"
#include <chrono>

using namespace std;


int main() {
    readFiles rf;

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

                    while (stringResposta != "e") {
                        Graph gShipping = rf.edgesGraphs("../ProjectData/Toy-Graphs/shipping.csv");
                        cout << "O caminho do Graph Shipping é: ";

                        gShipping.aux4_1(gShipping);

                        Graph gStadiums = rf.edgesGraphs("../ProjectData/Toy-Graphs/stadiums.csv");
                        cout << "O caminho do Graph Stadiums é: ";
                        gShipping.aux4_1(gStadiums);

                        Graph gTourism = rf.edgesGraphs("../ProjectData/Toy-Graphs/tourism.csv");
                        cout << "O caminho do Graph Tourism é: ";
                        gTourism.aux4_1(gTourism);

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
                cout << "Testar com os Real World Graphs?  - [PRESS 3]" << endl;
                cout << "Voltar para tras - [PRESS e] " << endl;
                cout << "==================================================================" << endl;
                cin >> stringResposta;
                if (stringResposta == "e") break;
                cin.ignore();
                cout << endl;

                if (stringResposta == "1") {

                    while (stringResposta != "e") {

                        cout << "No graph Shipping por não ser Fully Connected então não é suposto correr, no entanto a(s) edge(s) que não estão conectadas somamos 0." << endl;
                        Graph gShipping = rf.edgesGraphs("../ProjectData/Toy-Graphs/shipping.csv");
                        cout << "O caminho do Graph Shipping é: ";
                        gShipping.aux4_2(gShipping);


                        Graph gStadiums = rf.edgesGraphs("../ProjectData/Toy-Graphs/stadiums.csv");
                        cout << "O caminho do Graph Stadiums é: ";
                        gStadiums.aux4_2(gStadiums);


                        Graph gTourism = rf.edgesGraphs("../ProjectData/Toy-Graphs/tourism.csv");
                        cout << "O caminho do Graph Tourism é: ";
                        gTourism.aux4_2(gTourism);

                        cout << "==================================================================" << endl;
                        cin >> stringResposta;
                        if (stringResposta == "e") break;
                        cin.ignore();
                    }
                }
                if (stringResposta == "2") {
                    while (stringResposta != "e") {
                        Graph g25 = rf.edgesGraphs("../ProjectData/Extra_Fully_Connected_Graphs/edges_25.csv");
                        cout << "O caminho do Graph de 25 edges é: ";
                        g25.aux4_2(g25);


                        Graph g50 = rf.edgesGraphs("../ProjectData/Extra_Fully_Connected_Graphs/edges_50.csv");
                        cout << "O caminho do Graph de 50 edges é: ";
                        g50.aux4_2(g50);



                        Graph g75 = rf.edgesGraphs("../ProjectData/Extra_Fully_Connected_Graphs/edges_75.csv");
                        cout << "O caminho do Graph de 75 edges é: ";
                        g75.aux4_2(g75);


                        Graph g100 = rf.edgesGraphs("../ProjectData/Extra_Fully_Connected_Graphs/edges_100.csv");
                        cout << "O caminho do Graph de 100 edges é: ";
                        g100.aux4_2(g100);



                        Graph g200 = rf.edgesGraphs("../ProjectData/Extra_Fully_Connected_Graphs/edges_200.csv");
                        cout << "O caminho do Graph de 200 edges é: ";
                        g200.aux4_2(g200);



                        Graph g300 = rf.edgesGraphs("../ProjectData/Extra_Fully_Connected_Graphs/edges_300.csv");
                        cout << "O caminho do Graph de 300 edges é: ";
                        g300.aux4_2(g300);



                        Graph g400 = rf.edgesGraphs("../ProjectData/Extra_Fully_Connected_Graphs/edges_400.csv");
                         cout << "O caminho do Graph de 400 edges é: ";
                        g400.aux4_2(g400);



                        Graph g500 = rf.edgesGraphs("../ProjectData/Extra_Fully_Connected_Graphs/edges_500.csv");
                        cout << "O caminho do Graph de 500 edges é: ";
                        g500.aux4_2(g500);


                        Graph g600 = rf.edgesGraphs("../ProjectData/Extra_Fully_Connected_Graphs/edges_600.csv");
                        cout << "O caminho do Graph de 600 edges é: ";
                        g600.aux4_2(g600);



                        Graph g700 = rf.edgesGraphs("../ProjectData/Extra_Fully_Connected_Graphs/edges_700.csv");
                        cout << "O caminho do Graph de 700 edges é: ";
                        g700.aux4_2(g700);



                        Graph g800 = rf.edgesGraphs("../ProjectData/Extra_Fully_Connected_Graphs/edges_800.csv");
                         cout << "O caminho do Graph de 800 edges é: ";
                        g800.aux4_2(g800);



                        Graph g900 = rf.edgesGraphs("../ProjectData/Extra_Fully_Connected_Graphs/edges_900.csv");
                        cout << "O caminho do Graph de 900 edges é: ";
                        g900.aux4_2(g900);


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
                        cout << "O caminho do Real-World Graph 1 é: ";
                        gWorld1.aux4_2(gWorld1);



                        Graph gWorld2 = rf.realWorldGraphs("../ProjectData/Real-world Graphs/graph2/nodes.csv","../ProjectData/Real-world Graphs/graph2/edges.csv");
                        cout << "O caminho do Real-World Graph 2 é: ";
                        gWorld2.aux4_2(gWorld2);



                        Graph gWorld3 = rf.realWorldGraphs("../ProjectData/Real-world Graphs/graph3/nodes.csv","../ProjectData/Real-world Graphs/graph3/edges.csv");
                        cout << "O caminho do Real-World Graph 3 é: ";
                        gWorld3.aux4_2(gWorld3);


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
                cout << "Testar com os Real World Graphs?  - [PRESS 3]" << endl;
                cout << "Voltar para tras - [PRESS e] " << endl;
                cout << "==================================================================" << endl;
                cin >> stringResposta;
                if (stringResposta == "e") break;
                cin.ignore();
                cout << endl;

                if (stringResposta == "1") {
                    while (stringResposta != "e") {
                        Graph gShipping = rf.edgesGraphs("../ProjectData/Toy-Graphs/shipping.csv");
                        cout << "O caminho do Graph Shipping é: ";
                        gShipping.aux4_2(gShipping);

                        Graph gStadiums = rf.edgesGraphs("../ProjectData/Toy-Graphs/stadiums.csv");
                        cout << "O caminho do Graph Stadiums é: ";
                        gStadiums.aux4_2(gStadiums);

                        Graph gTourism = rf.edgesGraphs("../ProjectData/Toy-Graphs/tourism.csv");
                        cout << "O caminho do Graph Tourism é: ";
                        gTourism.aux4_2(gTourism);

                        cout << "Voltar para tras - [PRESS e] " << endl;
                        cout << "==================================================================" << endl;
                        cin >> stringResposta;
                        if (stringResposta == "e") break;
                        cin.ignore();
                    }
                }
                if (stringResposta == "2") {
                    while (stringResposta != "e") {
                        Graph g25 = rf.edgesGraphs("../ProjectData/Extra_Fully_Connected_Graphs/edges_25.csv");
                        cout << "O caminho do Graph de 25 edges é: ";
                        g25.final4_3(g25);

                        Graph g50 = rf.edgesGraphs("../ProjectData/Extra_Fully_Connected_Graphs/edges_50.csv");
                        cout << "O caminho do Graph de 50 edges é: ";
                        g50.final4_3(g50);

                        Graph g75 = rf.edgesGraphs("../ProjectData/Extra_Fully_Connected_Graphs/edges_75.csv");
                        cout << "O caminho do Graph de 75 edges é: ";
                        g75.final4_3(g75);

                        Graph g100 = rf.edgesGraphs("../ProjectData/Extra_Fully_Connected_Graphs/edges_100.csv");
                        cout << "O caminho do Graph de 100 edges é: ";
                        g100.final4_3(g100);

                        Graph g200 = rf.edgesGraphs("../ProjectData/Extra_Fully_Connected_Graphs/edges_200.csv");
                        cout << "O caminho do Graph de 200 edges é: ";
                        g200.final4_3(g200);

                        Graph g300 = rf.edgesGraphs("../ProjectData/Extra_Fully_Connected_Graphs/edges_300.csv");
                        cout << "O caminho do Graph de 300 edges é: ";
                        g300.final4_3(g300);

                        Graph g400 = rf.edgesGraphs("../ProjectData/Extra_Fully_Connected_Graphs/edges_400.csv");
                        cout << "O caminho do Graph de 400 edges é: ";
                        g400.final4_3(g400);

                        Graph g500 = rf.edgesGraphs("../ProjectData/Extra_Fully_Connected_Graphs/edges_500.csv");
                        cout << "O caminho do Graph de 500 edges é: ";
                        g500.final4_3(g500);

                        Graph g600 = rf.edgesGraphs("../ProjectData/Extra_Fully_Connected_Graphs/edges_600.csv");
                        cout << "O caminho do Graph de 600 edges é: ";
                        g600.final4_3(g600);

                        Graph g700 = rf.edgesGraphs("../ProjectData/Extra_Fully_Connected_Graphs/edges_700.csv");
                        cout << "O caminho do Graph de 700 edges é: ";
                        g700.final4_3(g700);

                        Graph g800 = rf.edgesGraphs("../ProjectData/Extra_Fully_Connected_Graphs/edges_800.csv");
                        cout << "O caminho do Graph de 800 edges é: ";
                        g800.final4_3(g800);

                        Graph g900 = rf.edgesGraphs("../ProjectData/Extra_Fully_Connected_Graphs/edges_900.csv");
                        cout << "O caminho do Graph de 900 edges é: ";
                        g900.final4_3(g900);

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
                        cout << "O caminho do Real-World Graph 1 é: ";
                        gWorld1.final4_3(gWorld1);

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
