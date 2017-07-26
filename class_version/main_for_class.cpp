//
// Created by Valifirt on 13/07/17.
//

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <ctime>

#include "Graph.h"

int main(int argc, char **argv) {

    if (argc < 6){
        std::cout << "Error arg" << std::endl;
        exit(1);
    }

    std::string path = argv[1];

    std::vector<std::pair<float,float>> vector_nodes;

    for(int i = 2; i < argc ; i += 2){
        vector_nodes.push_back({std::stof(argv[i]), std::stof(argv[i+1])});
    }

    Graph graph(path, vector_nodes);

    return 0;
}