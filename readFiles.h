//
// Created by Daniel on 18/05/2023.
//

#ifndef PROJ2_DA_READFILES_H
#define PROJ2_DA_READFILES_H


#include "Graph.h"

class readFiles {
public:
    Graph edgesGraphs(const std::string &filename);

    Graph realWorldGraphs(const std::string &filename, const std::string &filename2);
};


#endif //PROJ2_DA_READFILES_H
