//
// Created by Daniel on 18/05/2023.
//

#ifndef PROJ2_DA_READFILES_H
#define PROJ2_DA_READFILES_H


/**
 * @file readFiles.h
 *
 * @brief Este arquivo contém as definições das funções utilizadas para ler arquivos de dados e criar um grafo.
 *
 */

#ifndef READFILES_H
#define READFILES_H

#include "Graph.h"
#include <string>

/**
 * @brief A classe readFiles contém funções para ler arquivos de dados e criar um grafo.
 */
class readFiles {
public:
    /**
     * @brief Lê um arquivo contendo informações de arestas e cria um grafo.
     *
     * @param filename O nome do arquivo a ser lido.
     * @return O grafo criado a partir das informações do arquivo.
     *
     * A função lê um arquivo de dados contendo informações de arestas, e cria um grafo com base nessas informações.
     * Cada linha do arquivo deve conter três valores separados por vírgula: o nó inicial da aresta, o nó destino da
     * aresta e a distância entre eles. O grafo criado é retornado como resultado da função.
     *
     * Complexidade: O(E), onde E é o número de arestas no arquivo.
     */
    Graph edgesGraphs(const std::string &filename);

    /**
     * @brief Lê dois arquivos contendo informações de vértices e arestas, e cria um grafo.
     *
     * @param filename O nome do arquivo contendo informações dos vértices.
     * @param filename2 O nome do arquivo contendo informações das arestas.
     * @return O grafo criado a partir das informações dos arquivos.
     *
     * A função lê dois arquivos de dados, um contendo informações dos vértices e outro contendo informações das
     * arestas, e cria um grafo com base nessas informações. O arquivo de vértices deve conter uma linha de cabeçalho,
     * seguida por linhas contendo três valores separados por vírgula: o identificador do vértice, sua longitude e
     * latitude. O arquivo de arestas deve conter uma linha de cabeçalho, seguida por linhas contendo três valores
     * separados por vírgula: o nó inicial da aresta, o nó destino da aresta e a distância entre eles. O grafo criado é
     * retornado como resultado da função.
     *
     * Complexidade: O(V + E), onde V é o número de vértices no arquivo de vértices e E é o número de arestas no arquivo
     * de arestas.
     */
    Graph realWorldGraphs(const std::string &filename, const std::string &filename2);
};

#endif // READFILES_H



#endif //PROJ2_DA_READFILES_H
