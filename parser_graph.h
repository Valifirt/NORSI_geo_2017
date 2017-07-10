//
// Created by valya on 10/07/17.
//

#ifndef NORSI_GEO_2017_PARSER_GRAPH_H
#define NORSI_GEO_2017_PARSER_GRAPH_H

#include <vector>
#include <string>
#include <fstream>

struct Node {
    float lat, lon;
    unsigned long id;
};

struct Way {
    long id;
    Node start, finish;
    float len = 0;

    bool operator<(const Way rhs) const {
        return len < rhs.len;
    }
};

void parser(std::ifstream &in);

#endif //NORSI_GEO_2017_PARSER_GRAPH_H
