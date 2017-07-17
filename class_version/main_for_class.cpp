//
// Created by Valifirt on 13/07/17.
//

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <ctime>

#include "Graph.h"

int main(int argc, char **argv) {

    if (argc < 4){
        std::cout << "Error arg" << std::endl;
        exit(1);
    }

    std::string path = argv[1];

    Graph graph(argv[1]);

    std::vector<std::string> vector_nodes;

    for(int i = 2; i < argc ; i++){
        std::string point = argv[i];
        vector_nodes.push_back(point);
    }

    graph.short_way(vector_nodes);

    return 0;
}